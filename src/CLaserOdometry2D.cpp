/** ****************************************************************************************
*  This node presents a fast and precise method to estimate the planar motion of a lidar
*  from consecutive range scans. It is very useful for the estimation of the robot odometry from
*  2D laser range measurements.
*  This module is developed for mobile robots with innacurate or inexistent built-in odometry.
*  It allows the estimation of a precise odometry with low computational cost.
*  For more information, please refer to:
*
*  Planar Odometry from a Radial Laser Scanner. A Range Flow-based Approach. ICRA'16.
*  Available at: http://mapir.isa.uma.es/mapirwebsite/index.php/mapir-downloads/papers/217
*
* Maintainer: Javier G. Monroy
* MAPIR group: http://mapir.isa.uma.es/
*
* Modifications: Jeremie Deray
******************************************************************************************** */

#include "rf2o_laser_odometry/CLaserOdometry2D.h"

namespace rf2o {

// --------------------------------------------
// CLaserOdometry2D
//---------------------------------------------

CLaserOdometry2D::CLaserOdometry2D() :
  verbose(false),
  module_initialized(false),
  first_laser_scan(true),
  last_increment_(Pose3d::Identity()),
  laser_pose_on_robot_(Pose3d::Identity()),
  laser_pose_on_robot_inv_(Pose3d::Identity()),
  laser_pose_(Pose3d::Identity()),
  laser_oldpose_(Pose3d::Identity()),
  robot_pose_(Pose3d::Identity()),
  robot_oldpose_(Pose3d::Identity())
{
  //
}

void CLaserOdometry2D::setLaserPose(const Pose3d& laser_pose)
{
  //Set laser pose on the robot

  laser_pose_on_robot_     = laser_pose;
  laser_pose_on_robot_inv_ = laser_pose_on_robot_.inverse();
}

bool CLaserOdometry2D::is_initialized()
{
  return module_initialized;
}

void CLaserOdometry2D::init(const sensor_msgs::LaserScan& scan,
                            const geometry_msgs::Pose& initial_robot_pose)
{
  //Got an initial scan laser, obtain its parametes
  ROS_INFO_COND(verbose, "[rf2o] Got first Laser Scan .... Configuring node");

  width = scan.ranges.size();    // Num of samples (size) of the scan laser

  cols = width;						// Max resolution. Should be similar to the width parameter
  fovh = std::abs(scan.angle_max - scan.angle_min); // Horizontal Laser's FOV
  ctf_levels = 5;                     // Coarse-to-Fine levels
  iter_irls  = 5;                      //Num iterations to solve iterative reweighted least squares

  Pose3d robot_initial_pose = Pose3d::Identity();

  robot_initial_pose = Eigen::Quaterniond(initial_robot_pose.orientation.w,
                                          initial_robot_pose.orientation.x,
                                          initial_robot_pose.orientation.y,
                                          initial_robot_pose.orientation.z);

  robot_initial_pose.translation()(0) = initial_robot_pose.position.x;
  robot_initial_pose.translation()(1) = initial_robot_pose.position.y;

  ROS_INFO_STREAM_COND(verbose, "[rf2o] Setting origin at:\n"
                       << robot_initial_pose.matrix());

  //Set the initial pose
  laser_pose_    = robot_initial_pose * laser_pose_on_robot_;
  laser_oldpose_ = laser_oldpose_;

  // Init module (internal)
  //------------------------
  range_wf = Eigen::MatrixXf::Constant(1, width, 1);

  //Resize vectors according to levels
  transformations.resize(ctf_levels);
  for (unsigned int i = 0; i < ctf_levels; i++)
    transformations[i].resize(3, 3);

  //Resize pyramid
  unsigned int s, cols_i;
  const unsigned int pyr_levels = std::round(std::log2(round(float(width) / float(cols)))) + ctf_levels;
  range.resize(pyr_levels);
  range_old.resize(pyr_levels);
  range_inter.resize(pyr_levels);
  xx.resize(pyr_levels);
  xx_inter.resize(pyr_levels);
  xx_old.resize(pyr_levels);
  yy.resize(pyr_levels);
  yy_inter.resize(pyr_levels);
  yy_old.resize(pyr_levels);
  range_warped.resize(pyr_levels);
  xx_warped.resize(pyr_levels);
  yy_warped.resize(pyr_levels);

  for (unsigned int i = 0; i < pyr_levels; i++)
  {
    s = std::pow(2.f, int(i));
    cols_i = std::ceil(float(width) / float(s));

    range[i] = Eigen::MatrixXf::Constant(1, cols_i, 0.f);
    range_old[i] = Eigen::MatrixXf::Constant(1, cols_i, 0.f);
    range_inter[i].resize(1, cols_i);

    xx[i] = Eigen::MatrixXf::Constant(1, cols_i, 0.f);
    xx_old[i] = Eigen::MatrixXf::Constant(1, cols_i, 0.f);

    yy[i] = Eigen::MatrixXf::Constant(1, cols_i, 0.f);
    yy_old[i] = Eigen::MatrixXf::Constant(1, cols_i, 0.f);

    xx_inter[i].resize(1, cols_i);
    yy_inter[i].resize(1, cols_i);

    if (cols_i <= cols)
    {
      range_warped[i].resize(1, cols_i);
      xx_warped[i].resize(1, cols_i);
      yy_warped[i].resize(1, cols_i);
    }
  }

  dt.resize(1, cols);
  dtita.resize(1, cols);
  normx.resize(1, cols);
  normy.resize(1, cols);
  norm_ang.resize(1, cols);
  weights.resize(1, cols);

  null    = Eigen::MatrixXi::Constant(1, cols, 0);
  cov_odo = IncrementCov::Zero();

  fps = 1.f;		//In Hz
  num_valid_range = 0;

  //Compute gaussian mask
  g_mask[0] = 1.f / 16.f;
  g_mask[1] = 0.25f;
  g_mask[2] = 6.f / 16.f;
  g_mask[3] = g_mask[1];
  g_mask[4] = g_mask[0];

  kai_abs_     = MatrixS31::Zero();
  kai_loc_old_ = MatrixS31::Zero();

  module_initialized = true;
  last_odom_time = scan.header.stamp;
}

const CLaserOdometry2D::Pose3d& CLaserOdometry2D::getIncrement() const
{
  return last_increment_;
}

const Eigen::Matrix<float, 3, 3>& CLaserOdometry2D::getIncrementCovariance() const
{
  return cov_odo;
}

CLaserOdometry2D::Pose3d& CLaserOdometry2D::getPose()
{
  return robot_pose_;
}

const CLaserOdometry2D::Pose3d& CLaserOdometry2D::getPose() const
{
  return robot_pose_;
}

bool CLaserOdometry2D::odometryCalculation(const sensor_msgs::LaserScan& scan)
{
  //==================================================================================
  //						DIFERENTIAL  ODOMETRY  MULTILEVEL
  //==================================================================================

  //copy laser scan to internal variable
  range_wf = Eigen::Map<const Eigen::MatrixXf>(scan.ranges.data(), width, 1);

  ros::WallTime start = ros::WallTime::now();

  createImagePyramid();

  //Coarse-to-fine scheme
  for (unsigned int i=0; i<ctf_levels; i++)
  {
    //Previous computations
    transformations[i].setIdentity();

    level = i;
    unsigned int s = std::pow(2.f,int(ctf_levels-(i+1)));
    cols_i = std::ceil(float(cols)/float(s));
    image_level = ctf_levels - i + std::round(std::log2(std::round(float(width)/float(cols)))) - 1;

    //1. Perform warping
    if (i == 0)
    {
      range_warped[image_level] = range[image_level];
      xx_warped[image_level]    = xx[image_level];
      yy_warped[image_level]    = yy[image_level];
    }
    else
      performWarping();

    //2. Calculate inter coords
    calculateCoord();

    //3. Find null points
    findNullPoints();

    //4. Compute derivatives
    calculaterangeDerivativesSurface();

    //5. Compute normals
    //computeNormals();

    //6. Compute weights
    computeWeights();

    //7. Solve odometry
    if (num_valid_range > 3)
    {
      solveSystemNonLinear();
      //solveSystemOneLevel();    //without robust-function
    }
    else
    {
      /// @todo At initialization something
      /// isn't properly initialized so that
      /// uninitialized values get propagated
      /// from 'filterLevelSolution' first call
      /// Throughout the whole execution. Thus
      /// this 'continue' that surprisingly works.
      continue;
    }

    //8. Filter solution
    if (!filterLevelSolution()) return false;
  }

  m_runtime = ros::WallTime::now() - start;

  ROS_INFO_COND(verbose, "[rf2o] execution time (ms): %f",
                m_runtime.toSec()*double(1000));

  //Update poses
  PoseUpdate();

  return true;
}

void CLaserOdometry2D::createImagePyramid()
{
  const float max_range_dif = 0.3f;

  //Push the frames back
  range_old.swap(range);
  xx_old.swap(xx);
  yy_old.swap(yy);

  //The number of levels of the pyramid does not match the number of levels used
  //in the odometry computation (because we sometimes want to finish with lower resolutions)

  unsigned int pyr_levels = std::round(std::log2(std::round(float(width)/float(cols)))) + ctf_levels;

  //Generate levels
  for (unsigned int i = 0; i<pyr_levels; i++)
  {
    unsigned int s = std::pow(2.f,int(i));
    cols_i = std::ceil(float(width)/float(s));

    const unsigned int i_1 = i-1;

    //First level -> Filter (not downsampling);
    if (i == 0)
    {
      for (unsigned int u = 0; u < cols_i; u++)
      {
        const float dcenter = range_wf(u);

        //Inner pixels
        if ((u>1)&&(u<cols_i-2))
        {
          if (std::isfinite(dcenter) && dcenter > 0.f)
          {
            float sum = 0.f;
            float weight = 0.f;

            for (int l=-2; l<3; l++)
            {
              const float abs_dif = std::abs(range_wf(u+l)-dcenter);
              if (abs_dif < max_range_dif)
              {
                const float aux_w = g_mask[2+l]*(max_range_dif - abs_dif);
                weight += aux_w;
                sum += aux_w*range_wf(u+l);
              }
            }
            range[i](u) = sum/weight;
          }
          else
            range[i](u) = 0.f;
        }

        //Boundary
        else
        {
          if (std::isfinite(dcenter) && dcenter > 0.f)
          {
            float sum = 0.f;
            float weight = 0.f;

            for (int l=-2; l<3; l++)
            {
              const int indu = u+l;
              if ((indu>=0)&&(indu<cols_i))
              {
                const float abs_dif = std::abs(range_wf(indu)-dcenter);
                if (abs_dif < max_range_dif)
                {
                  const float aux_w = g_mask[2+l]*(max_range_dif - abs_dif);
                  weight += aux_w;
                  sum += aux_w*range_wf(indu);
                }
              }
            }
            range[i](u) = sum/weight;
          }
          else
            range[i](u) = 0.f;
        }
      }
    }

    //                              Downsampling
    //-----------------------------------------------------------------------------
    else
    {
      for (unsigned int u = 0; u < cols_i; u++)
      {
        const int u2 = 2*u;
        const float dcenter = range[i_1](u2);

        //Inner pixels
        if ((u>0)&&(u<cols_i-1))
        {
          if (dcenter > 0.f)
          {
            float sum = 0.f;
            float weight = 0.f;

            for (int l=-2; l<3; l++)
            {
              const float abs_dif = std::abs(range[i_1](u2+l)-dcenter);
              if (abs_dif < max_range_dif)
              {
                const float aux_w = g_mask[2+l]*(max_range_dif - abs_dif);
                weight += aux_w;
                sum += aux_w*range[i_1](u2+l);
              }
            }
            range[i](u) = sum/weight;
          }
          else
            range[i](u) = 0.f;

        }

        //Boundary
        else
        {
          if (dcenter > 0.f)
          {
            float sum = 0.f;
            float weight = 0.f;
            const unsigned int cols_i2 = range[i_1].cols();


            for (int l=-2; l<3; l++)
            {
              const int indu = u2+l;
              if ((indu>=0)&&(indu<cols_i2))
              {
                const float abs_dif = std::abs(range[i_1](indu)-dcenter);
                if (abs_dif < max_range_dif)
                {
                  const float aux_w = g_mask[2+l]*(max_range_dif - abs_dif);
                  weight += aux_w;
                  sum += aux_w*range[i_1](indu);
                }
              }
            }
            range[i](u) = sum/weight;
          }
          else
            range[i](u) = 0.f;

        }
      }
    }

    //Calculate coordinates "xy" of the points
    for (unsigned int u = 0; u < cols_i; u++)
    {
      if (range[i](u) > 0.f)
      {
        const float tita = -0.5*fovh + float(u)*fovh/float(cols_i-1);
        xx[i](u) = range[i](u)*std::cos(tita);
        yy[i](u) = range[i](u)*std::sin(tita);
      }
      else
      {
        xx[i](u) = 0.f;
        yy[i](u) = 0.f;
      }
    }
  }
}

void CLaserOdometry2D::calculateCoord()
{
  for (unsigned int u = 0; u < cols_i; u++)
  {
    if ((range_old[image_level](u) == 0.f) || (range_warped[image_level](u) == 0.f))
    {
      range_inter[image_level](u) = 0.f;
      xx_inter[image_level](u)    = 0.f;
      yy_inter[image_level](u)    = 0.f;
    }
    else
    {
      range_inter[image_level](u) = 0.5f*(range_old[image_level](u) + range_warped[image_level](u));
      xx_inter[image_level](u)    = 0.5f*(xx_old[image_level](u)    + xx_warped[image_level](u));
      yy_inter[image_level](u)    = 0.5f*(yy_old[image_level](u)    + yy_warped[image_level](u));
    }
  }
}

void CLaserOdometry2D::calculaterangeDerivativesSurface()
{
  //The gradient size ir reserved at the maximum size (at the constructor)

  //Compute connectivity

  //Defined in a different way now, without inversion
  rtita = Eigen::MatrixXf::Constant(1, cols_i, 1.f);

  for (unsigned int u = 0; u < cols_i-1; u++)
  {
    float dista = xx_inter[image_level](u+1) - xx_inter[image_level](u);
    dista *= dista;
    float distb = yy_inter[image_level](u+1) - yy_inter[image_level](u);
    distb *= distb;
    const float dist = dista + distb;

    if (dist  > 0.f)
      rtita(u) = std::sqrt(dist);
  }

  //Spatial derivatives
  for (unsigned int u = 1; u < cols_i-1; u++)
    dtita(u) = (rtita(u-1)*(range_inter[image_level](u+1)-
                range_inter[image_level](u)) + rtita(u)*(range_inter[image_level](u) -
        range_inter[image_level](u-1)))/(rtita(u)+rtita(u-1));

  dtita(0) = dtita(1);
  dtita(cols_i-1) = dtita(cols_i-2);

  //Temporal derivative
  for (unsigned int u = 0; u < cols_i; u++)
    dt(u) = fps*(range_warped[image_level](u) - range_old[image_level](u));


  //Apply median filter to the range derivatives
  //MatrixXf dtitamed = dtita, dtmed = dt;
  //vector<float> svector(3);
  //for (unsigned int u=1; u<cols_i-1; u++)
  //{
  //	svector.at(0) = dtita(u-1); svector.at(1) = dtita(u); svector.at(2) = dtita(u+1);
  //	std::sort(svector.begin(), svector.end());
  //	dtitamed(u) = svector.at(1);

  //	svector.at(0) = dt(u-1); svector.at(1) = dt(u); svector.at(2) = dt(u+1);
  //	std::sort(svector.begin(), svector.end());
  //	dtmed(u) = svector.at(1);
  //}

  //dtitamed(0) = dtitamed(1);
  //dtitamed(cols_i-1) = dtitamed(cols_i-2);
  //dtmed(0) = dtmed(1);
  //dtmed(cols_i-1) = dtmed(cols_i-2);

  //dtitamed.swap(dtita);
  //dtmed.swap(dt);
}

void CLaserOdometry2D::computeNormals()
{
  normx.setConstant(1, cols, 0.f);
  normy.setConstant(1, cols, 0.f);
  norm_ang.setConstant(1, cols, 0.f);

  const float incr_tita = fovh/float(cols_i-1);
  for (unsigned int u=0; u<cols_i; u++)
  {
    if (null(u) == 0.f)
    {
      const float tita = -0.5f*fovh + float(u)*incr_tita;
      const float alfa = -std::atan2(2.f*dtita(u), 2.f*range[image_level](u)*incr_tita);
      norm_ang(u) = tita + alfa;
      if (norm_ang(u) < -M_PI)
        norm_ang(u) += 2.f*M_PI;
      else if (norm_ang(u) < 0.f)
        norm_ang(u) += M_PI;
      else if (norm_ang(u) > M_PI)
        norm_ang(u) -= M_PI;

      normx(u) = std::cos(tita + alfa);
      normy(u) = std::sin(tita + alfa);
    }
  }
}

void CLaserOdometry2D::computeWeights()
{
  //The maximum weight size is reserved at the constructor
  weights.setConstant(1, cols, 0.f);

  //Parameters for error_linearization
  const float kdtita = 1.f;
  const float kdt = kdtita / (fps*fps);
  const float k2d = 0.2f;

  for (unsigned int u = 1; u < cols_i-1; u++)
    if (null(u) == 0)
    {
      //							Compute derivatives
      //-----------------------------------------------------------------------
      const float ini_dtita = range_old[image_level](u+1) - range_old[image_level](u-1);
      const float final_dtita = range_warped[image_level](u+1) - range_warped[image_level](u-1);

      const float dtitat = ini_dtita - final_dtita;
      const float dtita2 = dtita(u+1) - dtita(u-1);

      const float w_der = kdt*(dt(u)*dt(u)) +
          kdtita*(dtita(u)*dtita(u)) +
          k2d*(std::abs(dtitat) + std::abs(dtita2));

      weights(u) = std::sqrt(1.f/w_der);
    }

  const float inv_max = 1.f / weights.maxCoeff();
  weights = inv_max*weights;
}

void CLaserOdometry2D::findNullPoints()
{
  //Size of null matrix is set to its maximum size (constructor)
  num_valid_range = 0;

  for (unsigned int u = 1; u < cols_i-1; u++)
  {
    if (range_inter[image_level](u) == 0.f)
      null(u) = 1;
    else
    {
      num_valid_range++;
      null(u) = 0;
    }
  }
}

// Solves the system without considering any robust-function
void CLaserOdometry2D::solveSystemOneLevel()
{
  A.resize(num_valid_range, 3);
  B.resize(num_valid_range, 1);

  unsigned int cont = 0;
  const float kdtita = (cols_i-1)/fovh;

  //Fill the matrix A and the vector B
  //The order of the variables will be (vx, vy, wz)

  for (unsigned int u = 1; u < cols_i-1; u++)
    if (null(u) == 0)
    {
      // Precomputed expressions
      const float tw = weights(u);
      const float tita = -0.5*fovh + u/kdtita;

      //Fill the matrix A
      A(cont, 0) = tw*(std::cos(tita) + dtita(u)*kdtita*std::sin(tita)/range_inter[image_level](u));
      A(cont, 1) = tw*(std::sin(tita) - dtita(u)*kdtita*std::cos(tita)/range_inter[image_level](u));
      A(cont, 2) = tw*(-yy[image_level](u)*std::cos(tita) + xx[image_level](u)*std::sin(tita) - dtita(u)*kdtita);
      B(cont, 0) = tw*(-dt(u));

      cont++;
    }

  //Solve the linear system of equations using a minimum least squares method
  Eigen::MatrixXf AtA, AtB;
  AtA = A.transpose()*A;
  AtB = A.transpose()*B;
  Var = AtA.ldlt().solve(AtB);

  //Covariance matrix calculation 	Cov Order -> vx,vy,wz
  Eigen::MatrixXf res(num_valid_range,1);
  res = A*Var - B;
  cov_odo = (1.f/float(num_valid_range-3))*AtA.inverse()*res.squaredNorm();

  kai_loc_level_ = Var;
}

// Solves the system by considering the Cauchy M-estimator robust-function
void CLaserOdometry2D::solveSystemNonLinear()
{
  A.resize(num_valid_range, 3); Aw.resize(num_valid_range, 3);
  B.resize(num_valid_range, 1); Bw.resize(num_valid_range, 1);
  unsigned int cont = 0;
  const float kdtita = float(cols_i-1)/fovh;

  //Fill the matrix A and the vector B
  //The order of the variables will be (vx, vy, wz)

  for (unsigned int u = 1; u < cols_i-1; u++)
    if (null(u) == 0)
    {
      // Precomputed expressions
      const float tw = weights(u);
      const float tita = -0.5*fovh + u/kdtita;

      //Fill the matrix A
      A(cont, 0) = tw*(std::cos(tita) + dtita(u)*kdtita*std::sin(tita)/range_inter[image_level](u));
      A(cont, 1) = tw*(std::sin(tita) - dtita(u)*kdtita*std::cos(tita)/range_inter[image_level](u));
      A(cont, 2) = tw*(-yy[image_level](u)*std::cos(tita) + xx[image_level](u)*std::sin(tita) - dtita(u)*kdtita);
      B(cont, 0) = tw*(-dt(u));

      cont++;
    }

  //Solve the linear system of equations using a minimum least squares method
  Eigen::MatrixXf AtA, AtB;
  AtA = A.transpose()*A;
  AtB = A.transpose()*B;
  Var = AtA.ldlt().solve(AtB);

  //Covariance matrix calculation 	Cov Order -> vx,vy,wz
  Eigen::MatrixXf res(num_valid_range,1);
  res = A*Var - B;
  //cout << endl << "max res: " << res.maxCoeff();
  //cout << endl << "min res: " << res.minCoeff();

  ////Compute the energy
  //Compute the average dt
  float aver_dt = 0.f, aver_res = 0.f; unsigned int ind = 0;
  for (unsigned int u = 1; u < cols_i-1; u++)
    if (null(u) == 0)
    {
      aver_dt  += std::abs(dt(u));
      aver_res += std::abs(res(ind++));
    }
  aver_dt /= cont; aver_res /= cont;
  //    printf("\n Aver dt = %f, aver res = %f", aver_dt, aver_res);


  const float k = 10.f/aver_dt; //200
  //float energy = 0.f;
  //for (unsigned int i=0; i<res.rows(); i++)
  //	energy += log(1.f + mrpt::math::square(k*res(i)));
  //printf("\n\nEnergy(0) = %f", energy);

  //Solve iterative reweighted least squares
  //===================================================================
  for (unsigned int i=1; i<=iter_irls; i++)
  {
    cont = 0;

    for (unsigned int u = 1; u < cols_i-1; u++)
      if (null(u) == 0)
      {
        const float res_weight = std::sqrt(1.f/(1.f + ((k*res(cont))*(k*res(cont)))));

        //Fill the matrix Aw
        Aw(cont,0) = res_weight*A(cont,0);
        Aw(cont,1) = res_weight*A(cont,1);
        Aw(cont,2) = res_weight*A(cont,2);
        Bw(cont)   = res_weight*B(cont);
        cont++;
      }

    //Solve the linear system of equations using a minimum least squares method
    AtA = Aw.transpose()*Aw;
    AtB = Aw.transpose()*Bw;
    Var = AtA.ldlt().solve(AtB);
    res = A*Var - B;

    ////Compute the energy
    //energy = 0.f;
    //for (unsigned int j=0; j<res.rows(); j++)
    //	energy += log(1.f + mrpt::math::square(k*res(j)));
    //printf("\nEnergy(%d) = %f", i, energy);
  }

  cov_odo = (1.f/float(num_valid_range-3))*AtA.inverse()*res.squaredNorm();
  kai_loc_level_ = Var;

  ROS_INFO_STREAM_COND(verbose && false, "[rf2o] COV_ODO:\n" << cov_odo);
}

void CLaserOdometry2D::Reset(const Pose3d& ini_pose/*, CObservation2DRangeScan scan*/)
{
  //Set the initial pose
  laser_pose_    = ini_pose;
  laser_oldpose_ = ini_pose;

  //readLaser(scan);
  createImagePyramid();
}

void CLaserOdometry2D::performWarping()
{
  Eigen::Matrix3f acu_trans;

  acu_trans.setIdentity();
  for (unsigned int i=1; i<=level; i++)
    acu_trans = transformations[i-1]*acu_trans;

  Eigen::MatrixXf wacu = Eigen::MatrixXf::Constant(1, cols_i, 0.f);

  range_warped[image_level].setConstant(1, cols_i, 0.f);

  const float cols_lim = float(cols_i-1);
  const float kdtita = cols_lim/fovh;

  for (unsigned int j = 0; j<cols_i; j++)
  {
    if (range[image_level](j) > 0.f)
    {
      //Transform point to the warped reference frame
      const float x_w = acu_trans(0,0)*xx[image_level](j) + acu_trans(0,1)*yy[image_level](j) + acu_trans(0,2);
      const float y_w = acu_trans(1,0)*xx[image_level](j) + acu_trans(1,1)*yy[image_level](j) + acu_trans(1,2);
      const float tita_w = std::atan2(y_w, x_w);
      const float range_w = std::sqrt(x_w*x_w + y_w*y_w);

      //Calculate warping
      const float uwarp = kdtita*(tita_w + 0.5*fovh);

      //The warped pixel (which is not integer in general) contributes to all the surrounding ones
      if (( uwarp >= 0.f)&&( uwarp < cols_lim))
      {
        const int uwarp_l = uwarp;
        const int uwarp_r = uwarp_l + 1;
        const float delta_r = float(uwarp_r) - uwarp;
        const float delta_l = uwarp - float(uwarp_l);

        //Very close pixel
        if (std::abs(std::round(uwarp) - uwarp) < 0.05f)
        {
          range_warped[image_level](round(uwarp)) += range_w;
          wacu(std::round(uwarp)) += 1.f;
        }
        else
        {
          const float w_r = delta_l*delta_l;
          range_warped[image_level](uwarp_r) += w_r*range_w;
          wacu(uwarp_r) += w_r;

          const float w_l = delta_r*delta_r;
          range_warped[image_level](uwarp_l) += w_l*range_w;
          wacu(uwarp_l) += w_l;
        }
      }
    }
  }

  //Scale the averaged range and compute coordinates
  for (unsigned int u = 0; u<cols_i; u++)
  {
    if (wacu(u) > 0.f)
    {
      const float tita = -0.5f*fovh + float(u)/kdtita;
      range_warped[image_level](u) /= wacu(u);
      xx_warped[image_level](u) = range_warped[image_level](u)*std::cos(tita);
      yy_warped[image_level](u) = range_warped[image_level](u)*std::sin(tita);
    }
    else
    {
      range_warped[image_level](u) = 0.f;
      xx_warped[image_level](u) = 0.f;
      yy_warped[image_level](u) = 0.f;
    }
  }
}

bool CLaserOdometry2D::filterLevelSolution()
{
  //		Calculate Eigenvalues and Eigenvectors
  //----------------------------------------------------------
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eigensolver(cov_odo);
  if (eigensolver.info() != Eigen::Success)
  {
    ROS_WARN_COND(verbose, "[rf2o] ERROR: Eigensolver couldn't find a solution. Pose is not updated");
    return false;
  }

  //First, we have to describe both the new linear and angular speeds in the "eigenvector" basis
  //-------------------------------------------------------------------------------------------------
  Eigen::Matrix<float,3,3> Bii;
  Eigen::Matrix<float,3,1> kai_b;
  Bii = eigensolver.eigenvectors();

  kai_b = Bii.colPivHouseholderQr().solve(kai_loc_level_);

  assert((kai_loc_level_).isApprox(Bii*kai_b, 1e-5) && "Ax=b has no solution." && __LINE__);

  //Second, we have to describe both the old linear and angular speeds in the "eigenvector" basis too
  //-------------------------------------------------------------------------------------------------
  MatrixS31 kai_loc_sub;

  //Important: we have to substract the solutions from previous levels
  Eigen::Matrix3f acu_trans;
  acu_trans.setIdentity();
  for (unsigned int i=0; i<level; i++)
    acu_trans = transformations[i]*acu_trans;

  kai_loc_sub(0) = -fps*acu_trans(0,2);
  kai_loc_sub(1) = -fps*acu_trans(1,2);
  if (acu_trans(0,0) > 1.f)
    kai_loc_sub(2) = 0.f;
  else
  {
    kai_loc_sub(2) = -fps*std::acos(acu_trans(0,0))*rf2o::sign(acu_trans(1,0));
  }
  kai_loc_sub += kai_loc_old_;

  Eigen::Matrix<float,3,1> kai_b_old;
  kai_b_old = Bii.colPivHouseholderQr().solve(kai_loc_sub);

  assert((kai_loc_sub).isApprox(Bii*kai_b_old, 1e-5) && "Ax=b has no solution." && __LINE__);

  //Filter speed
  const float cf = 15e3f*std::exp(-float(int(level))),
              df = 0.05f*std::exp(-float(int(level)));

  Eigen::Matrix<float,3,1> kai_b_fil;
  for (unsigned int i=0; i<3; i++)
  {
    kai_b_fil(i) = (kai_b(i) + (cf*eigensolver.eigenvalues()(i,0) + df)*kai_b_old(i))/(1.f + cf*eigensolver.eigenvalues()(i,0) + df);
    //kai_b_fil_f(i,0) = (1.f*kai_b(i,0) + 0.f*kai_b_old_f(i,0))/(1.0f + 0.f);
  }

  //Transform filtered speed to local reference frame and compute transformation
  Eigen::Matrix<float, 3, 1> kai_loc_fil = Bii.inverse().colPivHouseholderQr().solve(kai_b_fil);

  assert((kai_b_fil).isApprox(Bii.inverse()*kai_loc_fil, 1e-5) && "Ax=b has no solution." && __LINE__);

  //transformation
  const float incrx = kai_loc_fil(0)/fps;
  const float incry = kai_loc_fil(1)/fps;
  const float rot   = kai_loc_fil(2)/fps;

  transformations[level](0,0) = std::cos(rot);
  transformations[level](0,1) = -std::sin(rot);
  transformations[level](1,0) = std::sin(rot);
  transformations[level](1,1) = std::cos(rot);
  transformations[level](0,2) = incrx;
  transformations[level](1,2) = incry;

  return true;
}

void CLaserOdometry2D::PoseUpdate()
{
  //  First, compute the overall transformation
  //---------------------------------------------------
  Eigen::Matrix3f acu_trans;
  acu_trans.setIdentity();

  for (unsigned int i=1; i<=ctf_levels; i++)
    acu_trans = transformations[i-1]*acu_trans;

  //				Compute kai_loc and kai_abs
  //--------------------------------------------------------
  kai_loc_(0) = fps*acu_trans(0,2);
  kai_loc_(1) = fps*acu_trans(1,2);

  if (acu_trans(0,0) > 1.f)
    kai_loc_(2) = 0.f;
  else
  {
    kai_loc_(2) = fps*std::acos(acu_trans(0,0))*rf2o::sign(acu_trans(1,0));
  }

  //cout << endl << "Arc cos (incr tita): " << kai_loc_(2);

  float phi = rf2o::getYaw(laser_pose_.rotation());

  kai_abs_(0) = kai_loc_(0)*std::cos(phi) - kai_loc_(1)*std::sin(phi);
  kai_abs_(1) = kai_loc_(0)*std::sin(phi) + kai_loc_(1)*std::cos(phi);
  kai_abs_(2) = kai_loc_(2);


  //						Update poses
  //-------------------------------------------------------
  laser_oldpose_ = laser_pose_;

  //  Eigen::Matrix3f aux_acu = acu_trans;
  Pose3d pose_aux_2D = Pose3d::Identity();

  pose_aux_2D = rf2o::matrixYaw(double(kai_loc_(2)/fps));
  pose_aux_2D.translation()(0) = acu_trans(0,2);
  pose_aux_2D.translation()(1) = acu_trans(1,2);

  laser_pose_ = laser_pose_ * pose_aux_2D;

  last_increment_ = pose_aux_2D;

  //				Compute kai_loc_old
  //-------------------------------------------------------
  phi = rf2o::getYaw(laser_pose_.rotation());
  kai_loc_old_(0) =  kai_abs_(0)*std::cos(phi) + kai_abs_(1)*std::sin(phi);
  kai_loc_old_(1) = -kai_abs_(0)*std::sin(phi) + kai_abs_(1)*std::cos(phi);
  kai_loc_old_(2) =  kai_abs_(2);

  ROS_INFO_COND(verbose, "[rf2o] LASERodom = [%f %f %f]",
                laser_pose_.translation()(0),
                laser_pose_.translation()(1),
                rf2o::getYaw(laser_pose_.rotation()));

  //Compose Transformations
  robot_pose_ = laser_pose_ * laser_pose_on_robot_inv_;

  ROS_INFO_COND(verbose, "BASEodom = [%f %f %f]",
                robot_pose_.translation()(0),
                robot_pose_.translation()(1),
                rf2o::getYaw(robot_pose_.rotation()));

  // Estimate linear/angular speeds (mandatory for base_local_planner)
  // last_scan -> the last scan received
  // last_odom_time -> The time of the previous scan lasser used to estimate the pose
  //-------------------------------------------------------------------------------------
  double time_inc_sec = (current_scan_time - last_odom_time).toSec();
  last_odom_time = current_scan_time;
  lin_speed = acu_trans(0,2) / time_inc_sec;
  //double lin_speed = sqrt( mrpt::math::square(robot_oldpose.x()-robot_pose.x()) + mrpt::math::square(robot_oldpose.y()-robot_pose.y()) )/time_inc_sec;

  double ang_inc = rf2o::getYaw(robot_pose_.rotation()) -
      rf2o::getYaw(robot_oldpose_.rotation());

  if (ang_inc > 3.14159)
    ang_inc -= 2*3.14159;
  if (ang_inc < -3.14159)
    ang_inc += 2*3.14159;

  ang_speed = ang_inc/time_inc_sec;
  robot_oldpose_ = robot_pose_;

  //filter speeds
  /*
    last_m_lin_speeds.push_back(lin_speed);
    if (last_m_lin_speeds.size()>4)
        last_m_lin_speeds.erase(last_m_lin_speeds.begin());
    double sum = std::accumulate(last_m_lin_speeds.begin(), last_m_lin_speeds.end(), 0.0);
    lin_speed = sum / last_m_lin_speeds.size();

    last_m_ang_speeds.push_back(ang_speed);
    if (last_m_ang_speeds.size()>4)
        last_m_ang_speeds.erase(last_m_ang_speeds.begin());
    double sum2 = std::accumulate(last_m_ang_speeds.begin(), last_m_ang_speeds.end(), 0.0);
    ang_speed = sum2 / last_m_ang_speeds.size();
    */
}

} /* namespace rf2o */
