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
******************************************************************************************** */

#include "rf2o_laser_odometry/CLaserOdometry2D.h"
using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace std;
using namespace Eigen;


// --------------------------------------------
// CLaserOdometry2D
//---------------------------------------------

CLaserOdometry2D::CLaserOdometry2D()
{	
    ROS_INFO("Initializing RF2O node...");

    //Read Parameters
    //----------------
    ros::NodeHandle pn("~");
    pn.param<std::string>("laser_scan_topic",laser_scan_topic,"/laser_scan");
    pn.param<std::string>("base_frame_id", base_frame_id, "/base_link");
    pn.param<std::string>("odom_frame_id", odom_frame_id, "/odom");
    pn.param<double>("freq",freq,10.0);

    //Publishers and Subscribers
    //--------------------------    
    odom_pub = pn.advertise<nav_msgs::Odometry>(odom_frame_id, 5);
    laser_sub = n.subscribe<sensor_msgs::LaserScan>(laser_scan_topic,1,&CLaserOdometry2D::LaserCallBack,this);

    //Init variables
    module_initialized = false;
    first_laser_scan = true;
}


CLaserOdometry2D::~CLaserOdometry2D()
{
}


bool CLaserOdometry2D::is_initialized()
{
    return module_initialized;
}

bool CLaserOdometry2D::scan_available()
{
    return new_scan_available;
}

void CLaserOdometry2D::Init()
{
    //Got an initial scan laser, obtain its parametes
    ROS_INFO("Got first Laser Scan .... Configuring node");
    width = last_scan.ranges.size();    // Num of samples (size) of the scan laser
    cols = width;						// Max reolution. Should be similar to the width parameter
    fovh = fabs(last_scan.angle_max - last_scan.angle_min); // Horizontal Laser's FOV
    ctf_levels = 5;                     // Coarse-to-Fine levels
    iter_irls = 5;                      //Num iterations to solve iterative reweighted least squares

    //Set laser pose on the robot (through tF)
    // This allow estimation of the odometry with respect to the robot base reference system.
    mrpt::poses::CPose3D LaserPoseOnTheRobot;
    tf::StampedTransform transform;
    try
    {
        tf_listener.lookupTransform("/base_link", last_scan.header.frame_id, ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    //TF:transform -> mrpt::CPose3D (see mrpt-ros-bridge)
    const tf::Vector3 &t = transform.getOrigin();
    LaserPoseOnTheRobot.x() = t[0];
    LaserPoseOnTheRobot.y() = t[1];
    LaserPoseOnTheRobot.z() = t[2];
    const tf::Matrix3x3 &basis = transform.getBasis();
    mrpt::math::CMatrixDouble33 R;
    for(int r = 0; r < 3; r++)
        for(int c = 0; c < 3; c++)
            R(r,c) = basis[r][c];
    LaserPoseOnTheRobot.setRotationMatrix(R);


    //Set the initial pose
    laser_pose = LaserPoseOnTheRobot;
    laser_oldpose = LaserPoseOnTheRobot;

    // Init module
    //-------------
    range_wf.setSize(1, width);

    //Resize vectors according to levels
    transformations.resize(ctf_levels);
    for (unsigned int i = 0; i < ctf_levels; i++)
        transformations[i].resize(3, 3);

    //Resize pyramid
    unsigned int s, cols_i;
    const unsigned int pyr_levels = round(log2(round(float(width) / float(cols)))) + ctf_levels;
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
        s = pow(2.f, int(i));
        cols_i = ceil(float(width) / float(s));

        range[i].resize(1, cols_i);
        range_inter[i].resize(1, cols_i);
        range_old[i].resize(1, cols_i);
        range[i].assign(0.0f);
        range_old[i].assign(0.0f);
        xx[i].resize(1, cols_i);
        xx_inter[i].resize(1, cols_i);
        xx_old[i].resize(1, cols_i);
        xx[i].assign(0.0f);
        xx_old[i].assign(0.0f);
        yy[i].resize(1, cols_i);
        yy_inter[i].resize(1, cols_i);
        yy_old[i].resize(1, cols_i);
        yy[i].assign(0.f);
        yy_old[i].assign(0.f);

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
    weights.setSize(1, cols);
    null.setSize(1, cols);
    null.assign(0);
    cov_odo.assign(0.f);


    fps = 1.f;		//In Hz
    num_valid_range = 0;

    //Compute gaussian mask
    g_mask[0] = 1.f / 16.f; g_mask[1] = 0.25f; g_mask[2] = 6.f / 16.f; g_mask[3] = g_mask[1]; g_mask[4] = g_mask[0];

    kai_abs.assign(0.f);
    kai_loc_old.assign(0.f);

    module_initialized = true;
    last_odom_time = ros::Time::now();
}


void CLaserOdometry2D::odometryCalculation()
{
    //==================================================================================
    //						DIFERENTIAL  ODOMETRY  MULTILEVEL
    //==================================================================================

    m_clock.Tic();
    createImagePyramid();

    //Coarse-to-fine scheme
    for (unsigned int i=0; i<ctf_levels; i++)
    {
        //Previous computations
        transformations[i].setIdentity();

        level = i;
        unsigned int s = pow(2.f,int(ctf_levels-(i+1)));
        cols_i = ceil(float(cols)/float(s));
        image_level = ctf_levels - i + round(log2(round(float(width)/float(cols)))) - 1;

        //1. Perform warping
        if (i == 0)
        {
            range_warped[image_level] = range[image_level];
            xx_warped[image_level] = xx[image_level];
            yy_warped[image_level] = yy[image_level];
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

        //8. Filter solution
        filterLevelSolution();
    }

    m_runtime = 1000*m_clock.Tac();
    ROS_INFO("Time odometry (ms): %f", m_runtime);

    //Update poses
    PoseUpdate();
    new_scan_available = false;     //avoids the possibility to run twice on the same laser scan
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

    unsigned int pyr_levels = round(log2(round(float(width)/float(cols)))) + ctf_levels;

    //Generate levels
    for (unsigned int i = 0; i<pyr_levels; i++)
    {
        unsigned int s = pow(2.f,int(i));
        cols_i = ceil(float(width)/float(s));
		
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
					if (dcenter > 0.f)
					{	
						float sum = 0.f;
						float weight = 0.f;

						for (int l=-2; l<3; l++)
						{
							const float abs_dif = abs(range_wf(u+l)-dcenter);
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
                    if (dcenter > 0.f)
					{						
						float sum = 0.f;
						float weight = 0.f;

						for (int l=-2; l<3; l++)	
						{
							const int indu = u+l;
							if ((indu>=0)&&(indu<cols_i))
							{
								const float abs_dif = abs(range_wf(indu)-dcenter);										
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
							const float abs_dif = abs(range[i_1](u2+l)-dcenter);
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
								const float abs_dif = abs(range[i_1](indu)-dcenter);										
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
				xx[i](u) = range[i](u)*cos(tita);
				yy[i](u) = range[i](u)*sin(tita);
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
			xx_inter[image_level](u) = 0.f;
			yy_inter[image_level](u) = 0.f;
		}
		else
		{
			range_inter[image_level](u) = 0.5f*(range_old[image_level](u) + range_warped[image_level](u));
			xx_inter[image_level](u) = 0.5f*(xx_old[image_level](u) + xx_warped[image_level](u));
			yy_inter[image_level](u) = 0.5f*(yy_old[image_level](u) + yy_warped[image_level](u));
		}
	}
}


void CLaserOdometry2D::calculaterangeDerivativesSurface()
{	
	//The gradient size ir reserved at the maximum size (at the constructor)

    //Compute connectivity
	rtita.resize(1,cols_i); 		//Defined in a different way now, without inversion
    rtita.assign(1.f); 

	for (unsigned int u = 0; u < cols_i-1; u++)
    {
		const float dist = square(xx_inter[image_level](u+1) - xx_inter[image_level](u))
							+ square(yy_inter[image_level](u+1) - yy_inter[image_level](u));
		if (dist  > 0.f)
			rtita(u) = sqrt(dist);
	}

    //Spatial derivatives
    for (unsigned int u = 1; u < cols_i-1; u++)
		dtita(u) = (rtita(u-1)*(range_inter[image_level](u+1)-range_inter[image_level](u)) + rtita(u)*(range_inter[image_level](u) - range_inter[image_level](u-1)))/(rtita(u)+rtita(u-1));

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
	normx.assign(0.f);
	normy.assign(0.f);
	norm_ang.assign(0.f);

	const float incr_tita = fovh/float(cols_i-1);
	for (unsigned int u=0; u<cols_i; u++)
	{
		if (null(u) == 0.f)
		{
			const float tita = -0.5f*fovh + float(u)*incr_tita;
			const float alfa = -atan2(2.f*dtita(u), 2.f*range[image_level](u)*incr_tita);
			norm_ang(u) = tita + alfa;
			if (norm_ang(u) < -M_PI)
				norm_ang(u) += 2.f*M_PI;
			else if (norm_ang(u) < 0.f)
				norm_ang(u) += M_PI;
			else if (norm_ang(u) > M_PI)
				norm_ang(u) -= M_PI;

			normx(u) = cos(tita + alfa);
			normy(u) = sin(tita + alfa);
		}
	}
}


void CLaserOdometry2D::computeWeights()
{
	//The maximum weight size is reserved at the constructor
	weights.assign(0.f);
	
	//Parameters for error_linearization
	const float kdtita = 1.f;
	const float kdt = kdtita/square(fps);
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

			const float w_der = kdt*square(dt(u)) + kdtita*square(dtita(u)) + k2d*(abs(dtitat) + abs(dtita2));

			weights(u) = sqrt(1.f/w_der);
		}

	const float inv_max = 1.f/weights.maximum();
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
	A.resize(num_valid_range,3);
	B.setSize(num_valid_range,1);
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
			A(cont, 0) = tw*(cos(tita) + dtita(u)*kdtita*sin(tita)/range_inter[image_level](u));
			A(cont, 1) = tw*(sin(tita) - dtita(u)*kdtita*cos(tita)/range_inter[image_level](u));
			A(cont, 2) = tw*(-yy[image_level](u)*cos(tita) + xx[image_level](u)*sin(tita) - dtita(u)*kdtita);
			B(cont,0) = tw*(-dt(u));

			cont++;
		}
	
	//Solve the linear system of equations using a minimum least squares method
	MatrixXf AtA, AtB;
	AtA.multiply_AtA(A);
	AtB.multiply_AtB(A,B);
	Var = AtA.ldlt().solve(AtB);

	//Covariance matrix calculation 	Cov Order -> vx,vy,wz
	MatrixXf res(num_valid_range,1);
	res = A*Var - B;
	cov_odo = (1.f/float(num_valid_range-3))*AtA.inverse()*res.squaredNorm();

	kai_loc_level = Var;
}


// Solves the system by considering the Cauchy M-estimatorrobust-function
void CLaserOdometry2D::solveSystemNonLinear()
{
    A.resize(num_valid_range,3); Aw.resize(num_valid_range,3);
    B.setSize(num_valid_range,1); Bw.setSize(num_valid_range,1);
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
            A(cont, 0) = tw*(cos(tita) + dtita(u)*kdtita*sin(tita)/range_inter[image_level](u));
            A(cont, 1) = tw*(sin(tita) - dtita(u)*kdtita*cos(tita)/range_inter[image_level](u));
            A(cont, 2) = tw*(-yy[image_level](u)*cos(tita) + xx[image_level](u)*sin(tita) - dtita(u)*kdtita);
            B(cont,0) = tw*(-dt(u));

            cont++;
        }

    //Solve the linear system of equations using a minimum least squares method
    MatrixXf AtA, AtB;
    AtA.multiply_AtA(A);
    AtB.multiply_AtB(A,B);
    Var = AtA.ldlt().solve(AtB);

    //Covariance matrix calculation 	Cov Order -> vx,vy,wz
    MatrixXf res(num_valid_range,1);
    res = A*Var - B;
    //cout << endl << "max res: " << res.maxCoeff();
    //cout << endl << "min res: " << res.minCoeff();

    ////Compute the energy
    //Compute the average dt
    float aver_dt = 0.f, aver_res = 0.f; unsigned int ind = 0;
    for (unsigned int u = 1; u < cols_i-1; u++)
        if (null(u) == 0)
        {
            aver_dt += fabsf(dt(u));
            aver_res += fabsf(res(ind++));
        }
    aver_dt /= cont; aver_res /= cont;
//    printf("\n Aver dt = %f, aver res = %f", aver_dt, aver_res);


    const float k = 10.f/aver_dt; //200
    //float energy = 0.f;
    //for (unsigned int i=0; i<res.rows(); i++)
    //	energy += log(1.f + square(k*res(i)));
    //printf("\n\nEnergy(0) = %f", energy);

    //Solve iterative reweighted least squares
    //===================================================================
    for (unsigned int i=1; i<=iter_irls; i++)
    {
        cont = 0;

        for (unsigned int u = 1; u < cols_i-1; u++)
            if (null(u) == 0)
            {
                const float res_weight = sqrt(1.f/(1.f + square(k*res(cont))));

                //Fill the matrix Aw
                Aw(cont,0) = res_weight*A(cont,0);
                Aw(cont,1) = res_weight*A(cont,1);
                Aw(cont,2) = res_weight*A(cont,2);
                Bw(cont) = res_weight*B(cont);
                cont++;
            }

        //Solve the linear system of equations using a minimum least squares method
        AtA.multiply_AtA(Aw);
        AtB.multiply_AtB(Aw,Bw);
        Var = AtA.ldlt().solve(AtB);
        res = A*Var - B;

        ////Compute the energy
        //energy = 0.f;
        //for (unsigned int j=0; j<res.rows(); j++)
        //	energy += log(1.f + square(k*res(j)));
        //printf("\nEnergy(%d) = %f", i, energy);
    }

    cov_odo = (1.f/float(num_valid_range-3))*AtA.inverse()*res.squaredNorm();
    kai_loc_level = Var;
    std::cout << endl << "COV_ODO: " << cov_odo  << endl;
}

void CLaserOdometry2D::Reset(CPose3D ini_pose, CObservation2DRangeScan scan)
{
	//Set the initial pose
	laser_pose = ini_pose;
	laser_oldpose = ini_pose;

    //readLaser(scan);
	createImagePyramid();
    //readLaser(scan);
	createImagePyramid();
}


void CLaserOdometry2D::performWarping()
{
	Matrix3f acu_trans; 
	acu_trans.setIdentity();
	for (unsigned int i=1; i<=level; i++)
		acu_trans = transformations[i-1]*acu_trans;

	MatrixXf wacu(1,cols_i);
	wacu.assign(0.f);
	range_warped[image_level].assign(0.f);

	const float cols_lim = float(cols_i-1);
	const float kdtita = cols_lim/fovh;

	for (unsigned int j = 0; j<cols_i; j++)
	{				
		if (range[image_level](j) > 0.f)
		{
			//Transform point to the warped reference frame
			const float x_w = acu_trans(0,0)*xx[image_level](j) + acu_trans(0,1)*yy[image_level](j) + acu_trans(0,2);
			const float y_w = acu_trans(1,0)*xx[image_level](j) + acu_trans(1,1)*yy[image_level](j) + acu_trans(1,2);
			const float tita_w = atan2(y_w, x_w);
			const float range_w = sqrt(x_w*x_w + y_w*y_w);

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
				if (abs(round(uwarp) - uwarp) < 0.05f)
				{
					range_warped[image_level](round(uwarp)) += range_w;
					wacu(round(uwarp)) += 1.f;
				}
				else
				{
					const float w_r = square(delta_l);
					range_warped[image_level](uwarp_r) += w_r*range_w;
					wacu(uwarp_r) += w_r;

					const float w_l = square(delta_r);
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
			xx_warped[image_level](u) = range_warped[image_level](u)*cos(tita);
			yy_warped[image_level](u) = range_warped[image_level](u)*sin(tita);
		}
		else
		{
			range_warped[image_level](u) = 0.f;
			xx_warped[image_level](u) = 0.f;
			yy_warped[image_level](u) = 0.f;
		}
	}
}





void CLaserOdometry2D::filterLevelSolution()
{
	//		Calculate Eigenvalues and Eigenvectors
	//----------------------------------------------------------
	SelfAdjointEigenSolver<MatrixXf> eigensolver(cov_odo);
	if (eigensolver.info() != Success) 
	{ 
		printf("Eigensolver couldn't find a solution. Pose is not updated");
		return;
	}
	
	//First, we have to describe both the new linear and angular speeds in the "eigenvector" basis
	//-------------------------------------------------------------------------------------------------
	Matrix<float,3,3> Bii;
	Matrix<float,3,1> kai_b;
	Bii = eigensolver.eigenvectors();

	kai_b = Bii.colPivHouseholderQr().solve(kai_loc_level);

	//Second, we have to describe both the old linear and angular speeds in the "eigenvector" basis too
	//-------------------------------------------------------------------------------------------------
	CMatrixFloat31 kai_loc_sub;

	//Important: we have to substract the solutions from previous levels
	Matrix3f acu_trans;
	acu_trans.setIdentity();
	for (unsigned int i=0; i<level; i++)
		acu_trans = transformations[i]*acu_trans;

	kai_loc_sub(0) = -fps*acu_trans(0,2);
	kai_loc_sub(1) = -fps*acu_trans(1,2);
	if (acu_trans(0,0) > 1.f)
		kai_loc_sub(2) = 0.f;
	else
		kai_loc_sub(2) = -fps*acos(acu_trans(0,0))*sign(acu_trans(1,0));
	kai_loc_sub += kai_loc_old;

	Matrix<float,3,1> kai_b_old;
	kai_b_old = Bii.colPivHouseholderQr().solve(kai_loc_sub);

	//Filter speed
	const float cf = 15e3f*expf(-int(level)), df = 0.05f*expf(-int(level));

	Matrix<float,3,1> kai_b_fil;
	for (unsigned int i=0; i<3; i++)
	{
			kai_b_fil(i,0) = (kai_b(i,0) + (cf*eigensolver.eigenvalues()(i,0) + df)*kai_b_old(i,0))/(1.f + cf*eigensolver.eigenvalues()(i,0) + df);
			//kai_b_fil_f(i,0) = (1.f*kai_b(i,0) + 0.f*kai_b_old_f(i,0))/(1.0f + 0.f);
	}

	//Transform filtered speed to local reference frame and compute transformation
	Matrix<float,3,1> kai_loc_fil = Bii.inverse().colPivHouseholderQr().solve(kai_b_fil);

	//transformation
	const float incrx = kai_loc_fil(0)/fps;
	const float incry = kai_loc_fil(1)/fps;
	const float rot = kai_loc_fil(2)/fps;
	transformations[level](0,0) = cos(rot);
	transformations[level](0,1) = -sin(rot);
	transformations[level](1,0) = sin(rot);
	transformations[level](1,1) = cos(rot);
	transformations[level](0,2) = incrx;
	transformations[level](1,2) = incry;
}


void CLaserOdometry2D::PoseUpdate()
{
	//First, compute the overall transformation
	//---------------------------------------------------
	Matrix3f acu_trans;
	acu_trans.setIdentity();
	for (unsigned int i=1; i<=ctf_levels; i++)
		acu_trans = transformations[i-1]*acu_trans;


	//				Compute kai_loc and kai_abs
	//--------------------------------------------------------
	kai_loc(0) = fps*acu_trans(0,2);
	kai_loc(1) = fps*acu_trans(1,2);
	if (acu_trans(0,0) > 1.f)
		kai_loc(2) = 0.f;
	else
		kai_loc(2) = fps*acos(acu_trans(0,0))*sign(acu_trans(1,0));

	//cout << endl << "Arc cos (incr tita): " << kai_loc(2);

	float phi = laser_pose.yaw();

	kai_abs(0) = kai_loc(0)*cos(phi) - kai_loc(1)*sin(phi);
	kai_abs(1) = kai_loc(0)*sin(phi) + kai_loc(1)*cos(phi);
	kai_abs(2) = kai_loc(2);


	//						Update poses
	//-------------------------------------------------------
	laser_oldpose = laser_pose;
	math::CMatrixDouble33 aux_acu = acu_trans;
	poses::CPose2D pose_aux_2D(acu_trans(0,2), acu_trans(1,2), kai_loc(2)/fps);
    laser_pose = laser_pose + pose_aux_2D;



	//				Compute kai_loc_old
	//-------------------------------------------------------
	phi = laser_pose.yaw();
	kai_loc_old(0) = kai_abs(0)*cos(phi) + kai_abs(1)*sin(phi);
	kai_loc_old(1) = -kai_abs(0)*sin(phi) + kai_abs(1)*cos(phi);
	kai_loc_old(2) = kai_abs(2);


    ROS_INFO("LASERodom = [%f %f %f]",laser_pose.x(),laser_pose.y(),laser_pose.yaw());


    // GET ROBOT POSE from LASER POSE
    //------------------------------  
    mrpt::poses::CPose3D LaserPoseOnTheRobot_inv;
    tf::StampedTransform transform;
    try
    {
        tf_listener.lookupTransform(last_scan.header.frame_id, "/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    //TF:transform -> mrpt::CPose3D (see mrpt-ros-bridge)
    const tf::Vector3 &t = transform.getOrigin();
    LaserPoseOnTheRobot_inv.x() = t[0];
    LaserPoseOnTheRobot_inv.y() = t[1];
    LaserPoseOnTheRobot_inv.z() = t[2];
    const tf::Matrix3x3 &basis = transform.getBasis();
    mrpt::math::CMatrixDouble33 R;
    for(int r = 0; r < 3; r++)
        for(int c = 0; c < 3; c++)
            R(r,c) = basis[r][c];
    LaserPoseOnTheRobot_inv.setRotationMatrix(R);

    //Compose Transformations
    robot_pose = laser_pose + LaserPoseOnTheRobot_inv;
    ROS_INFO("BASEodom = [%f %f %f]",robot_pose.x(),robot_pose.y(),robot_pose.yaw());


    // Estimate linear/angular speeds (mandatory for base_local_planner)
    // last_scan -> the last scan received
    // last_odom_time -> The time of the previous scan lasser used to estimate the pose
    //-------------------------------------------------------------------------------------
    double time_inc_sec = (last_scan.header.stamp - last_odom_time).toSec();
    last_odom_time = last_scan.header.stamp;
    double lin_speed = acu_trans(0,2) / time_inc_sec;
    //double lin_speed = sqrt( square(robot_oldpose.x()-robot_pose.x()) + square(robot_oldpose.y()-robot_pose.y()) )/time_inc_sec;
    double ang_inc = robot_pose.yaw() - robot_oldpose.yaw();
    if (ang_inc > 3.14159)
        ang_inc -= 2*3.14159;
    if (ang_inc < -3.14159)
        ang_inc += 2*3.14159;
    double ang_speed = ang_inc/time_inc_sec;
    robot_oldpose = robot_pose;

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

    //first, we'll publish the odometry over tf
    //---------------------------------------
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = odom_frame_id;
    odom_trans.child_frame_id = base_frame_id;
    odom_trans.transform.translation.x = robot_pose.x();
    odom_trans.transform.translation.y = robot_pose.y();
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(robot_pose.yaw());
    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    //-------------------------------------------------
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = odom_frame_id;
    //set the position
    odom.pose.pose.position.x = robot_pose.x();
    odom.pose.pose.position.y = robot_pose.y();
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(robot_pose.yaw());
    //set the velocity
    odom.child_frame_id = base_frame_id;
    odom.twist.twist.linear.x = lin_speed;    //linear speed
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = ang_speed;   //angular speed
    //publish the message
    odom_pub.publish(odom);
}



//-----------------------------------------------------------------------------------
//                                   CALLBACKS
//-----------------------------------------------------------------------------------

void CLaserOdometry2D::LaserCallBack(const sensor_msgs::LaserScan::ConstPtr& new_scan)
{
    //Keep in memory the last received laser_scan
    last_scan = *new_scan;

    //Initialize module on first scan
    if (first_laser_scan)
    {
        Init();
        first_laser_scan = false;
    }
    else
    {
        //copy laser scan to internal variable
        for (unsigned int i = 0; i<width; i++)
            range_wf(i) = new_scan->ranges[i];
        new_scan_available = true;
    }
}

//-----------------------------------------------------------------------------------
//                                   MAIN
//-----------------------------------------------------------------------------------
int main(int argc, char** argv)
{
    ros::init(argc, argv, "RF2O_LaserOdom");

    CLaserOdometry2D myLaserOdom;

    //Main Loop
    //----------
    ROS_INFO("initialization complete...Looping");
    ros::Rate loop_rate(myLaserOdom.freq);
    while (ros::ok())
    {
        ros::spinOnce();        //Check for new laser scans

        if( myLaserOdom.is_initialized() && myLaserOdom.scan_available() )
        {            
            //Process odometry estimation
            myLaserOdom.odometryCalculation();
        }
        else
        {
            ROS_WARN("Waiting for laser_scans....") ;
        }

        loop_rate.sleep();
    }
    return(0);
}
