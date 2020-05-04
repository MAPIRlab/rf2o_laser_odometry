# Laser Odometry

This is the patched fork project of [`rf2o_laser_odometry` by MAPIRlabs](https://github.com/MAPIRlab/rf2o_laser_odometry)

> Estimation of 2D odometry based on planar laser scans. Useful for mobile robots with innacurate base odometry. http://mapir.isa.uma.es/work/rf2o For full description of the algorithm, please refer to: **Planar Odometry from a Radial Laser Scanner. A Range Flow-based Approach. ICRA 2016**

The `rf2o_laser_odometry` module is [ROS node](https://www.ros.org/) which estimates path and displacements of the mobile robot using planar LiDAR scans. The module designed as standard ROS package with following dependencies:
- `catkin`
- `roscpp`
- `sensor_msgs`
- `std_msgs`
- `tf`
- `cmake_modules`
- `eigen`
- `nav_msgs`

To build the module within the existing workspace, standard `catkin_make` script could be used.

## Runnning

The module runs under control of `roslaunch` server with initialization script in corresponding `.launch`-file containing XML like following:

```xml
<launch>
    <node   pkg="rf2o_laser_odometry" 
            type="rf2o_laser_odometry_node" 
            name="rf2o_laser_odometry" 
            output="screen">
        <rosparam file="$(find rf2o_laser_odometry)/config/default.yml"/>
    </node>
</launch>
```

## Configuration

Using `<param>` XML tag the configuration parameters could be placed into the `.launch`-file directly but it is strongly recommended to create special dedicated YAML configuration file for `<rosparam>` tag (the example configuration file with default values for all parameters is shipped with the module, see [`default.yml`](./config/default.yml)). All parameters in this file could be omitted to remain with default values.

### General settings

|**Name**|**Default value**|**Type**|**Description**|
|-------|-------------------------|-------|------------|
|`verbose`|`false`|`bool`|Toggles debug messages on/off|
|`counter_clockwise`|`true`|`bool`|Set this flag to `true`, if your LiDAR sensor or mirror block rotates **counter clockwise**. For some robots, e.a. _Turtlebot 3_ the rotation direction of the mirror is **clockwise**, so this flag should be set `false` to avoid negative values of robot coordinate accretions|
|`laser_scan_topic`|`/laser_scan`|`std::string`|Topic name to subscribe for laser scans|
|`laser_frame_id`|`/laser`|`std::string`|Frame ID for laser scan message header. **CAUTION!** The coordinate transform should exist from `base_frame_id` to `laser_frame_id` to successfully run the module|
|`odom_topic`|`/odom_rf2o`|`std::string`|Topic name to publish output odometry messages|
|`publish_tf`|`true`|`bool`|Toggles publishing of coordinate transformation between `base_frame_id` and `odom_frame_id`|
|`base_frame_id`|`/base_link`|`std::string`|Frame ID for coordinate system/transformation snapped to the robot itself|
|`odom_frame_id`|`/odom`|`std::string`|Frame ID for output odometry message header|
|`init_pose_from_topic`|`/base_pose_ground_truth`|`std::string`|Topic name to subscribe for initial pose|
|`freq`|`10.0`|`double`|Odometry update frequency (Hz). This setting should be approximately equal to LiDAR sensor polling rate|

### Covariation settings

|**Name**|**Default value**|**Type**|**Description**|
|-------|-------------------------|-------|------------|
|`pose_covariance_matrix`|Zeros 6х6|`std::vector<double>`|Static pose covariation matrix|
|`twist_covariance_matrix`|Zeros 6х6|`std::vector<double>`|Static covariation matrix for kinematical parameters|

#### Covariation auto-adjustment settings

This patched version of the module can automatically boost the covariation when odometry gap (for example, because of long straight corridor) is detected. The corresponding settings are placed into `dynamic_covariance_boost` group in YAML file.

|**Name**|**Default value**|**Type**|**Description**|
|-------|-------------------------|-------|------------|
|`enable`|`false`|`bool`|Toggles automatic covariation boost on/off|
|`initial_multiplier`|`1.0`|`double`|Base multiplpier for covariation matrix|
|`progressive`|`false`|`bool`|Toggles additional boosting progression on/off when the odometry gap longs more that one polling period|
|`progression_factor`|`0`|`double`|Progression booster value|

#### Trusted ranges settings

These settings are dedicated to detection of odometry gaps. They allows user to set thresholds and fallback sources to replace the odometry if the obtained values become impossible or untrusted for the current robot. These settings are placed into the group `pose_fallback` in YAML file.

##### Trusted values settings

|**Name**|**Default value**|**Type**|**Description**|
|-------|-------------------------|-------|------------|
|`continuous_fallback_topic`|Empty string|`std::string`|Fallback topic to replace the pose data for the detected gap|
|`velocity_fallback_topic`|`continuous_fallback_topic` value|`std::string`|Fallback topic to replace the motion data for the detected gap|
|`enable_thresholds`|`false`|`bool`|Toggles trust thresholds on/off|
|`linear_velocity_threshold_x`|$`10^6`$|`double`|Lenghtwise velocity value trust threshold|
|`linear_velocity_threshold_y`|`linear_velocity_threshold_x` value|`double`|Transversal velocity value trust threshold|
|`angular_velocity_threshold`|$`10^6`$|`double`|Angular velocity value trust threshold. The angular velocity is considered to be define in `base_frame_id` coordinate system|


