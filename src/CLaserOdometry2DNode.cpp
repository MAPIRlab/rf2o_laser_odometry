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

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class CLaserOdometry2DNode : CLaserOdometry2D
{
public:

  CLaserOdometry2DNode();
  ~CLaserOdometry2DNode();

  void process(const ros::TimerEvent &);
  void publish();

  bool setLaserPoseFromTf();

public:

  bool publish_tf,new_scan_available;

  double freq;

  std::string         laser_scan_topic;
  std::string         odom_topic;
  std::string         base_frame_id;
  std::string         odom_frame_id;
  std::string         init_pose_from_topic;

  ros::NodeHandle             n;
  sensor_msgs::LaserScan      last_scan;
  bool                        GT_pose_initialized;
  tf::TransformListener       tf_listener;          //Do not put inside the callback
  tf::TransformBroadcaster    odom_broadcaster;
  nav_msgs::Odometry          initial_robot_pose;

  //Subscriptions & Publishers
  ros::Subscriber laser_sub, initPose_sub;
  ros::Publisher odom_pub;

  bool scan_available();

  //CallBacks
  void LaserCallBack(const sensor_msgs::LaserScan::ConstPtr& new_scan);
  void initPoseCallBack(const nav_msgs::Odometry::ConstPtr& new_initPose);
};

CLaserOdometry2DNode::CLaserOdometry2DNode() :
  CLaserOdometry2D()
{
    ROS_INFO("Initializing RF2O node...");

    //Read Parameters
    //----------------
    ros::NodeHandle pn("~");
    pn.param<std::string>("laser_scan_topic",laser_scan_topic,"/laser_scan");
    pn.param<std::string>("odom_topic", odom_topic, "/odom_rf2o");
    pn.param<std::string>("base_frame_id", base_frame_id, "/base_link");
    pn.param<std::string>("odom_frame_id", odom_frame_id, "/odom");
    pn.param<bool>("publish_tf", publish_tf, true);
    pn.param<std::string>("init_pose_from_topic", init_pose_from_topic, "/base_pose_ground_truth");
    pn.param<double>("freq",freq,10.0);
    pn.param<bool>("verbose", verbose, true);

    //Publishers and Subscribers
    //--------------------------
    odom_pub  = pn.advertise<nav_msgs::Odometry>(odom_topic, 5);
    laser_sub = n.subscribe<sensor_msgs::LaserScan>(laser_scan_topic,1,&CLaserOdometry2DNode::LaserCallBack,this);

    //init pose??
    if (init_pose_from_topic != "")
    {
        initPose_sub = n.subscribe<nav_msgs::Odometry>(init_pose_from_topic,1,&CLaserOdometry2DNode::initPoseCallBack,this);
        GT_pose_initialized  = false;
    }
    else
    {
        GT_pose_initialized = true;
        initial_robot_pose.pose.pose.position.x = 0;
        initial_robot_pose.pose.pose.position.y = 0;
        initial_robot_pose.pose.pose.position.z = 0;
        initial_robot_pose.pose.pose.orientation.w = 0;
        initial_robot_pose.pose.pose.orientation.x = 0;
        initial_robot_pose.pose.pose.orientation.y = 0;
        initial_robot_pose.pose.pose.orientation.z = 0;
    }

    setLaserPoseFromTf();

    //Init variables
    module_initialized = false;
    first_laser_scan   = true;

  ROS_INFO_STREAM("Listening laser scan from topic: " << laser_sub.getTopic());
}

CLaserOdometry2DNode::~CLaserOdometry2DNode()
{
  //
}

bool CLaserOdometry2DNode::setLaserPoseFromTf()
{
  bool retrieved = false;

  // Set laser pose on the robot (through tF)
  // This allow estimation of the odometry with respect to the robot base reference system.
  tf::StampedTransform transform;
  transform.setIdentity();
  try
  {
      tf_listener.lookupTransform(base_frame_id, last_scan.header.frame_id, ros::Time(0), transform);
      retrieved = true;
  }
  catch (tf::TransformException &ex)
  {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      retrieved = false;
  }

  //TF:transform -> mrpt::CPose3D (see mrpt-ros-bridge)
  mrpt::poses::CPose3D laser_tf;
  const tf::Vector3 &t = transform.getOrigin();
  laser_tf.x() = t[0];
  laser_tf.y() = t[1];
  laser_tf.z() = t[2];
  const tf::Matrix3x3 &basis = transform.getBasis();
  mrpt::math::CMatrixDouble33 R;
  for(int r = 0; r < 3; r++)
      for(int c = 0; c < 3; c++)
          R(r,c) = basis[r][c];
  laser_tf.setRotationMatrix(R);

  setLaserPose(laser_tf);

  return retrieved;
}

bool CLaserOdometry2DNode::scan_available()
{
    return new_scan_available;
}

void CLaserOdometry2DNode::process(const ros::TimerEvent&)
{
  if( is_initialized() && scan_available() )
  {
      //Process odometry estimation
      odometryCalculation(last_scan);
      publish();
      new_scan_available = false; //avoids the possibility to run twice on the same laser scan
  }
  else
  {
      ROS_WARN("Waiting for laser_scans....") ;
  }
}

//-----------------------------------------------------------------------------------
//                                   CALLBACKS
//-----------------------------------------------------------------------------------

void CLaserOdometry2DNode::LaserCallBack(const sensor_msgs::LaserScan::ConstPtr& new_scan)
{
    if (GT_pose_initialized)
    {
        //Keep in memory the last received laser_scan
        last_scan = *new_scan;
        current_scan_time = last_scan.header.stamp;

        //Initialize module on first scan
        if (first_laser_scan)
        {
            Init(last_scan, initial_robot_pose.pose.pose);
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
}

void CLaserOdometry2DNode::initPoseCallBack(const nav_msgs::Odometry::ConstPtr& new_initPose)
{
    //Initialize module on first GT pose. Else do Nothing!
    if (!GT_pose_initialized)
    {
        initial_robot_pose = *new_initPose;
        GT_pose_initialized = true;
    }
}

void CLaserOdometry2DNode::publish()
{
  //first, we'll publish the odometry over tf
  //---------------------------------------
  if (publish_tf)
  {
      //ROS_INFO("[rf2o] Publishing TF: [base_link] to [odom]");
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
  }

  //next, we'll publish the odometry message over ROS
  //-------------------------------------------------
  //ROS_INFO("[rf2o] Publishing Odom Topic");
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
//                                   MAIN
//-----------------------------------------------------------------------------------
int main(int argc, char** argv)
{
    ros::init(argc, argv, "RF2O_LaserOdom");

    CLaserOdometry2DNode myLaserOdomNode;

    ros::TimerOptions timer_opt;
    timer_opt.oneshot   = false;
    timer_opt.autostart = true;
    timer_opt.callback_queue = ros::getGlobalCallbackQueue();
    timer_opt.tracked_object = ros::VoidConstPtr();

    timer_opt.callback = boost::bind(&CLaserOdometry2DNode::process, &myLaserOdomNode, _1);
    timer_opt.period   = ros::Rate(myLaserOdomNode.freq).expectedCycleTime();

    ros::Timer rf2o_timer = ros::NodeHandle("~").createTimer(timer_opt);

    ros::spin();

    return EXIT_SUCCESS;
}
