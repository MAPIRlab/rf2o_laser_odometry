cmake_minimum_required(VERSION 3.5)
project(rf2o_laser_odometry)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2  REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)


## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
  include include/
)

include_directories(
  
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

set(DEPENDENCIES
  "rclcpp"           
  "std_msgs"         
  "geometry_msgs" 
  "tf2_geometry_msgs"
  "tf2"
  "tf2_ros"
  "sensor_msgs"
  "nav_msgs"
)
add_executable(rf2o_laser_odometry_node src/CLaserOdometry2DNode.cpp src/CLaserOdometry2D.cpp)
ament_target_dependencies(rf2o_laser_odometry_node ${DEPENDENCIES})

## Declare a cpp library
add_library(${PROJECT_NAME} src/CLaserOdometry2D.cpp)

## Declare a cpp executable
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

install(
  DIRECTORY launch  
  DESTINATION share/${PROJECT_NAME})

