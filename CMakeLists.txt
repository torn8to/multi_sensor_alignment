cmake_minimum_required(VERSION 3.8)
project(multi_sensor_alignment)

# Use C++17 for better compatibility with modern PCL and ROS 2
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenMP)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

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

add_executable(${PROJECT_NAME}_icp_align_tool_node
  src/icp_align_tool_node.cpp
)

add_library(${PROJECT_NAME}_icp_align_tool_lib SHARED
  src/icp_align_tool_class.cpp
  include/icp_align_tool/icp_align_tool.h
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
 # ${OpenCV_INCLUDE_DIRS}
 #${Eigen3_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

target_link_libraries(${PROJECT_NAME}_icp_align_tool_lib
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_icp_align_tool_node
  ${PROJECT_NAME}_icp_align_tool_lib
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

ament_target_dependencies(
  ${PROJECT_NAME}_alignment_publisher_node
  rclcpp
  rcl_interfaces
  geometry_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
)

ament_target_dependencies(${PROJECT_NAME}_icp_align_tool_lib
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  tf2_sensor_msgs
  pcl_conversions
  pcl_ros
)


ament_target_dependencies(
  ${PROJECT_NAME}_icp_align_tool_node
  rclcpp
  rcl_interfaces
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_sensor_msgs
  pcl_conversions
  pcl_ros
)


#set_target_properties(${PROJECT_NAME}_icp_align_tool_node PROPERTIES OUTPUT_NAME icp_align_tool PREFIX "")
#set_target_properties(${PROJECT_NAME}_alignment_publisher_node PROPERTIES OUTPUT_NAME alignment_publisher PREFIX "")


install(TARGETS
  ${PROJECT_NAME}_alignment_publisher_node
  ${PROJECT_NAME}_icp_align_tool_lib
  ${PROJECT_NAME}_icp_align_tool_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY ./config
    DESTINATION share/${PROJECT_NAME})

ament_package()
