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

bool CLaserOdometry2DNode::CheckCovarianceArray(const nav_msgs::Odometry::_pose_type::_covariance_type &place, const std::vector<double>& matrix)
{
    if(place.size() < matrix.size())
    {
        ROS_ERROR("Can not write covariation matrix! Please check reserved array size [%lu vs %lu]!", place.size(), matrix.size());
        return false;
    }
    return true;
}

void CLaserOdometry2DNode::PreparePoseCovariance(nav_msgs::Odometry &message, const std::vector<double> &matrix)
{
    if(CheckCovarianceArray(message.pose.covariance, matrix))
    {
        unsigned int c = 0;
        for(auto i = matrix.begin(); i != matrix.end(); i++)
        {
            message.pose.covariance[c++] = *i;
        }
    }
}

void CLaserOdometry2DNode::PrepareTwistCovariance(nav_msgs::Odometry &message, const std::vector<double> &matrix)
{
    if(CheckCovarianceArray(message.twist.covariance, matrix))
    {
        unsigned int c = 0;
        for(auto i = matrix.begin(); i != matrix.end(); i++)
        {
            message.twist.covariance[c++] = *i;
        }
    }
}

CLaserOdometry2DNode::CLaserOdometry2DNode() :
    CLaserOdometry2D(),
    outdated(0)
{
    ROS_INFO("Initializing RF2O node...");

    //Read Parameters
    //----------------
    ros::NodeHandle pn("~");
    pn.param<std::string>("laser_scan_topic",laser_scan_topic,"/laser_scan");
    pn.param<std::string>("laser_frame_id", laser_frame_id, "/laser");
    pn.param<std::string>("odom_topic", odom_topic, "/odom_rf2o");
    pn.param<std::string>("base_frame_id", base_frame_id, "/base_link");
    pn.param<std::string>("odom_frame_id", odom_frame_id, "/odom");
    pn.param<bool>("publish_tf", publish_tf, true);
    pn.param<std::string>("init_pose_from_topic", init_pose_from_topic, "/base_pose_ground_truth");
    pn.param<double>("freq",freq,10.0);
    pn.param<bool>("verbose", verbose, true);
    pn.param<std::vector<double> >("pose_covariance_matrix", linear_covariance_matrix, std::vector<double>());
    pn.param<std::vector<double> >("twist_covariance_matrix", angular_covariance_matrix, std::vector<double>());
    pn.param<bool>("dynamic_covariance_boost/enable", dynamic_covariance_boost, false);
    pn.param<double>("dynamic_covariance_boost/initial_multiplier", dynamic_covariance_boost_initial_multiplier, 1.f);
    pn.param<bool>("dynamic_covariance_boost/progressive", dynamic_covariance_boost_progressive, false);
    pn.param<double>("dynamic_covariance_boost/progression_factor", dynamic_covariance_boost_progression_factor, 0.f);
    pn.param<std::string>("dynamic_covariance_boost/pose_fallback_topic", pose_fallback_topic, "");

    //Publishers and Subscribers
    //--------------------------
    odom_pub  = pn.advertise<nav_msgs::Odometry>(odom_topic, 5);
    laser_sub = n.subscribe<sensor_msgs::LaserScan>(laser_scan_topic,1,&CLaserOdometry2DNode::LaserCallBack,this);

    if(pose_fallback_topic != "")
    {
        pose_fallback = n.subscribe(pose_fallback_topic, 1, &CLaserOdometry2DNode::PoseFallbackCallback, this);
    }
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

bool CLaserOdometry2DNode::setLaserPoseFromTf()
{
    bool retrieved = false;

    // Set laser pose on the robot (through tF)
    // This allow estimation of the odometry with respect to the robot base reference system.
    tf::StampedTransform transform;
    transform.setIdentity();
    ros::Duration(2.0).sleep();
    try
    {
        tf_listener.lookupTransform(base_frame_id, laser_frame_id, ros::Time(0), transform);
        retrieved = true;
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        retrieved = false;
    }

    //ROS_INFO("YAW ANGLE: %f", tf::getYaw(transform.getRotation()));
    //TF:transform -> Eigen::Isometry3d

    const tf::Matrix3x3 &basis = transform.getBasis();
    Eigen::Matrix3d R;

    for(int r = 0; r < 3; r++)
        for(int c = 0; c < 3; c++)
            R(r,c) = basis[r][c];

    Pose3d laser_tf(R);

    const tf::Vector3 &t = transform.getOrigin();
    laser_tf.translation()(0) = t[0];
    laser_tf.translation()(1) = t[1];
    laser_tf.translation()(2) = t[2];

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
        if (!first_laser_scan)
        {
            //copy laser scan to internal variable
            for (unsigned int i = 0; i<width; i++)
                range_wf(i) = new_scan->ranges[i];
            new_scan_available = true;
        }
        else
        {
            init(last_scan, initial_robot_pose.pose.pose);
            first_laser_scan = false;
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

void CLaserOdometry2DNode::PoseFallbackCallback(const nav_msgs::Odometry::ConstPtr &fallback_pose_)
{
    fallback_pose = *fallback_pose_;
    fallback_time = ros::Time::now();
    if(!fallback_active) fallback_active = true;
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
        odom_trans.transform.translation.x = robot_pose_.translation()(0);
        odom_trans.transform.translation.y = robot_pose_.translation()(1);
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(rf2o::getYaw(robot_pose_.rotation()));
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
    odom.pose.pose.position.x = robot_pose_.translation()(0);
    odom.pose.pose.position.y = robot_pose_.translation()(1);
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(rf2o::getYaw(robot_pose_.rotation()));
    //set the velocity
    odom.child_frame_id = base_frame_id;
    double laser_yaw = rf2o::getYaw(  laser_pose_on_robot_.rotation() );
    if( (laser_yaw < (3.1415926 / 2)) || (laser_yaw > (-3.1415926 / 2)) )
    {
        odom.twist.twist.linear.x = linear_vx * std::cos(laser_yaw);
    }
    else
    {
        odom.twist.twist.linear.x = -linear_vx * std::cos(laser_yaw);
    }
    odom.twist.twist.linear.y = linear_vy;
    odom.twist.twist.angular.z = ang_speed;   //angular speed
    //Prepare covariance in case of dynamic boost
    std::vector<double> pub_linear_covariance = linear_covariance_matrix;
    std::vector<double> pub_angular_covariance = angular_covariance_matrix;
    if(dynamic_covariance_boost && unreliable)
    {
        BoostCovarianceMatrix(pub_linear_covariance);
        BoostCovarianceMatrix(pub_angular_covariance);
        outdated++;
    }
    if(!unreliable) outdated = 0;
    PreparePoseCovariance(odom, pub_linear_covariance);
    PrepareTwistCovariance(odom, pub_angular_covariance);
    odom_pub.publish(odom);
}

void CLaserOdometry2DNode::BoostCovarianceMatrix(std::vector<double>& covariance)
{
    for(auto e = covariance.begin(); e != covariance.end(); e++)
    {
        *e *= dynamic_covariance_boost_initial_multiplier;
        if(dynamic_covariance_boost_progressive)
            *e += dynamic_covariance_boost_progression_factor * outdated;
    }
    ROS_INFO_COND(verbose, "[COVARIANCE BOOST] Boosted by init %.3f with progression factor %.3f",
                  dynamic_covariance_boost_initial_multiplier,
                  dynamic_covariance_boost_progression_factor);
}

} /* namespace rf2o */

//-----------------------------------------------------------------------------------
//                                   MAIN
//-----------------------------------------------------------------------------------
int main(int argc, char** argv)
{
    ros::init(argc, argv, "RF2O_LaserOdom");

    rf2o::CLaserOdometry2DNode myLaserOdomNode;

    ros::TimerOptions timer_opt;
    timer_opt.oneshot   = false;
    timer_opt.autostart = true;
    timer_opt.callback_queue = ros::getGlobalCallbackQueue();
    timer_opt.tracked_object = ros::VoidConstPtr();

    timer_opt.callback = boost::bind(&rf2o::CLaserOdometry2DNode::process, &myLaserOdomNode, _1);
    timer_opt.period   = ros::Rate(myLaserOdomNode.freq).expectedCycleTime();

    ros::Timer rf2o_timer = ros::NodeHandle("~").createTimer(timer_opt);

    ros::spin();

    return EXIT_SUCCESS;
}
