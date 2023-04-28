#include "rf2o_laser_odometry/CLaserOdometry2D.hpp"

#include <tf2/convert.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/impl/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

namespace rf2o {

class CLaserOdometry2DNode : public rclcpp::Node
{
public:
  CLaserOdometry2DNode();
  void process();
  void publish();
  bool setLaserPoseFromTf();
  bool scan_available();

  // Params & vars
  CLaserOdometry2D    rf2o_ref;
  bool                publish_tf, new_scan_available;
  double              freq;
  std::string         laser_scan_topic;
  std::string         odom_topic;
  std::string         base_frame_id;
  std::string         odom_frame_id;
  std::string         init_pose_from_topic;

  sensor_msgs::msg::LaserScan                     last_scan;
  bool                                            GT_pose_initialized;
  std::shared_ptr<tf2_ros::Buffer>                buffer_;
  std::shared_ptr<tf2_ros::TransformListener>     tf_listener_;  
  std::unique_ptr<tf2_ros::TransformBroadcaster>  odom_broadcaster;
  nav_msgs::msg::Odometry                         initial_robot_pose;

  // Subscriptions & Publishers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr  laser_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      initPose_sub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr         odom_pub;

  // CallBacks
  void LaserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr new_scan);
  void initPoseCallBack(const nav_msgs::msg::Odometry::SharedPtr new_initPose);
};

