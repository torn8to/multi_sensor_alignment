cmake_minimum_required(VERSION 3.8)
project(multi_sensor_alignment)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(PKG_DEPS
  rclcpp
  nav_msgs 
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  tf2_sensor_msgs
  pcl_ros
  pcl_conversions
  cv_bridge
  image_transport
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(catkin REQUIRED COMPONENTS ${PKG_DEPS} cmake_modules) 
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
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
include_directories(include
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}                    
)

add_definitions(${PCL_DEFINITIONS})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DE<build_depend>pcl_conversions</build_depend>P_SET be the set of packages whose message types you use in
##   your messa<build_depend>pcl_conversions</build_depend>ges/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the fil<build_depend>pcl_conversions</build_depend>e package.xml:
##   * add a bu<build_depend>pcl_conversions</build_depend>ild_depend tag for "message_generation"
##   * add a bu<build_depend>pcl_conversions</build_depend>ild_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_D<build_depend>pcl_conversions</build_depend>EP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   nav_msgs
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder

include_directories(
  include
)

add_executable(${PROJECT_NAME}_icp_align_tool_node
  src/ic_align_tool_node.cpp
)

add_executable(${PROJECT_NAME}_alignment_publisher_node
  src/alignment_publisher.cpp
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
set_target_properties(${PROJECT_NAME}_icp_align_tool_node PROPERTIES OUTPUT_NAME icp_align_tool PREFIX "")
set_target_properties(${PROJECT_NAME}_alignment_publisher_node PROPERTIES OUTPUT_NAME alignment_publisher PREFIX "")

## Declare a C++ library
# add_library(${PROJECT_NAME} src/${PROJECT_NAME}/${PROJECT_NAME}.cpp )
add_library(${PROJECT_NAME}_icp_align_tool_lib SHARED
  src/icp_align_tool_class.cpp
  include/icp_align_tool/icp_align_tool.h
)

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

target_link_libraries(${PROJECT_NAME}_icp_align_tool_node
${PROJECT_NAME}_icp_align_tool_lib
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_alignment_publisher_node
  ${catkin_LIBRARIES}
  yaml-cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_alignment_publisher_node
   ${PROJECT_NAME}_gencfg
#   ${catkin_EXPORTED_TARGETS}
)

add_dependencies(${PROJECT_NAME}_icp_align_tool_lib
  ${catkin_EXPORTED_TARGETS}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_publisher_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_pointcloud2_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_${PROJECT_NAME}.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

