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

#ifndef CLaserOdometry2D_H
#define CLaserOdometry2D_H

// std header
#include <iostream>
#include <fstream>
#include <numeric>

// ROS headers
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose.hpp>
// Eigen headers
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>

namespace rf2o {

template <typename T>
inline T sign(const T x) { return x<T(0) ? -1:1; }

template <typename Derived>
inline typename Eigen::MatrixBase<Derived>::Scalar
getYaw(const Eigen::MatrixBase<Derived>& r)
{
  return std::atan2( r(1, 0), r(0, 0) );
}

template<typename T>
inline Eigen::Matrix<T, 3, 3> matrixRollPitchYaw(const T roll,
                                                 const T pitch,
                                                 const T yaw)
{
  const Eigen::AngleAxis<T> ax = Eigen::AngleAxis<T>(roll,  Eigen::Matrix<T, 3, 1>::UnitX());
  const Eigen::AngleAxis<T> ay = Eigen::AngleAxis<T>(pitch, Eigen::Matrix<T, 3, 1>::UnitY());
  const Eigen::AngleAxis<T> az = Eigen::AngleAxis<T>(yaw,   Eigen::Matrix<T, 3, 1>::UnitZ());

  return (az * ay * ax).toRotationMatrix().matrix();
}

template<typename T>
inline Eigen::Matrix<T, 3, 3> matrixYaw(const T yaw)
{
  return matrixRollPitchYaw<T>(0, 0, yaw);
}

using Scalar = float;

using Pose2d = Eigen::Isometry2d;
using Pose3d = Eigen::Isometry3d;
using MatrixS31 = Eigen::Matrix<Scalar, 3, 1>;
using IncrementCov = Eigen::Matrix<Scalar, 3, 3>;

class CLaserOdometry2D: public rclcpp::Node
{
public:

  CLaserOdometry2D();

  void init(const sensor_msgs::msg::LaserScan& scan,
            const geometry_msgs::msg::Pose& initial_robot_pose);

  bool is_initialized();

  bool odometryCalculation(const sensor_msgs::msg::LaserScan& scan);

  void setLaserPose(const Pose3d& laser_pose);

  const Pose3d& getIncrement() const;

  const IncrementCov& getIncrementCovariance() const;

  Pose3d& getPose();
  const Pose3d& getPose() const;
  bool verbose, module_initialized, first_laser_scan;
  rclcpp::Time last_odom_time, current_scan_time;

  // Internal Data
  std::vector<Eigen::MatrixXf> range;
  std::vector<Eigen::MatrixXf> range_old;
  std::vector<Eigen::MatrixXf> range_inter;
  std::vector<Eigen::MatrixXf> range_warped;
  std::vector<Eigen::MatrixXf> xx;
  std::vector<Eigen::MatrixXf> xx_inter;
  std::vector<Eigen::MatrixXf> xx_old;
  std::vector<Eigen::MatrixXf> xx_warped;
  std::vector<Eigen::MatrixXf> yy;
  std::vector<Eigen::MatrixXf> yy_inter;
  std::vector<Eigen::MatrixXf> yy_old;
  std::vector<Eigen::MatrixXf> yy_warped;
  std::vector<Eigen::MatrixXf> transformations;

  Eigen::MatrixXf range_wf;
  Eigen::MatrixXf dtita;
  Eigen::MatrixXf dt;
  Eigen::MatrixXf rtita;
  Eigen::MatrixXf normx, normy, norm_ang;
  Eigen::MatrixXf weights;
  Eigen::MatrixXi null;

  Eigen::MatrixXf A,Aw;
  Eigen::MatrixXf B,Bw;

  MatrixS31 Var;	//3 unknowns: vx, vy, w
  IncrementCov cov_odo;

  //std::string LaserVarName;				//Name of the topic containing the scan lasers \laser_scan
  float fps;								//In Hz
  float fovh;								//Horizontal FOV
  unsigned int cols;
  unsigned int cols_i;
  unsigned int width;
  unsigned int ctf_levels;
  unsigned int image_level, level;
  unsigned int num_valid_range;
  unsigned int iter_irls;
  float g_mask[5];

  double lin_speed, ang_speed;

  //rclcpp::wall	m_runtime;

  MatrixS31 kai_abs_;
  MatrixS31 kai_loc_;
  MatrixS31 kai_loc_old_;
  MatrixS31 kai_loc_level_;

  Pose3d last_increment_;
  Pose3d laser_pose_on_robot_;
  Pose3d laser_pose_on_robot_inv_;
  Pose3d laser_pose_;
  Pose3d laser_oldpose_;
  Pose3d robot_pose_;
  Pose3d robot_oldpose_;

  bool test;
  std::vector<double> last_m_lin_speeds;
  std::vector<double> last_m_ang_speeds;

  // Methods
  void createImagePyramid();
  void calculateCoord();
  void performWarping();
  void calculaterangeDerivativesSurface();
  void computeNormals();
  void computeWeights();
  void findNullPoints();
  void solveSystemOneLevel();
  void solveSystemNonLinear();
  bool filterLevelSolution();
  void PoseUpdate();
  void Reset(const Pose3d& ini_pose/*, CObservation2DRangeScan scan*/);
};

} /* namespace rf2o */

#endif // CLaserOdometry2D_H
