cmake_minimum_required(VERSION 3.8)
project(multi_sensor_alignment)
#TODO:alter for c++17 support
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

#TODO: setup support for colcon build

find_package(ament_cmake REQUIRED) # used for ros2 build system
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
#find_package(image_transport REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED)
find_package(OpenMP)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

## Check OpenCV Version
#set(OpenCV_REQUIRED_VERSION "3.4")
find_package(OpenCV REQUIRED)
#if (${OpenCV_VERSION_MAJOR}.${OpenCV_VERSION_MINOR} VERSION_GREATER ${OpenCV_REQUIRED_VERSION} )
#  message(FATAL_ERROR "Incorrect OpenCV Version: ${OpenCV_VERSION}, Requires: ${OpenCV_REQUIRED_VERSION}")
#endif()


## Specify additional locations of header files
## Your package locations should be listed before other locations

add_executable(${PROJECT_NAME}_alignment_publisher_node
  src/alignment_publisher.cpp
)
#add_executable(${PROJECT_NAME}_icp_align_tool_node
#  src/icp_align_tool_node.cpp
#)

#include_directories(include
  #${PCL_INCLUDE_DIRS}
 # ${Boost_INCLUDE_DIRS}
 # ${OpenCV_INCLUDE_DIRS}
 #${Eigen3_INCLUDE_DIRS}
#)


#ament_target_dependencies(${PROJECT_NAME}_lib
# rclcpp
#  std_msgs
#  std_srvs
#  sensor_msgs
#  geometry_msgs
#  tf2_ros
#  tf2_eigen
#  pcl_conversions
#  pcl_ros
#)

#add_definitions(${PCL_DEFINITIONS})


ament_target_dependencies(
  ${PROJECT_NAME}_alignment_publisher_node
  rclcpp
  rcl_interfaces
  geometry_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
)

#set_target_properties(${PROJECT_NAME}_icp_align_tool_node PROPERTIES OUTPUT_NAME icp_align_tool PREFIX "")
set_target_properties(${PROJECT_NAME}_alignment_publisher_node PROPERTIES OUTPUT_NAME alignment_publisher PREFIX "")

#add_library(${PROJECT_NAME}_icp_align_tool_lib SHARED
#  src/icp_align_tool_class.cpp
#  include/icp_align_tool/icp_align_tool.h
#)

install(TARGETS
  ${PROJECT_NAME}_alignment_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY ./config
    DESTINATION share/${PROJECT_NAME})

ament_package()
