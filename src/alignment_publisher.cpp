/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include <rclcpp/rclcpp.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/PointCloud2.h>

#include <std_srvs/Empty.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

static std::string lidar_id_str_;
static std::string child_frame_;
static std::string parent_frame_;
static std::string alignment_file_;


boost::recursive_mutex server_mutex_;
class AlginmentPublisher:public rclcpp::Node
{
  public:
    AlignmentPublisher():Node("alignment_publisher")
    {
      RCL_CPP_INFO(this -> get_logger,"alignment_ publisher_online");
      this -> declare_parameter("alignment_file",rclcpp::PARAMETER_STRING);
      this -> declare_parameter("parent_frame",rclcpp::PARAMETER_STRING);
      this -> declare_parameter("child_frame",rclcpp::PARAMETER_STRING);
      this -> declare_parameter("x",rclcpp::PARAMETER_DOUBLE);
      this -> declare_parameter("y",rclcpp::PARAMETER_DOUBLE);
      this -> declare_parameter("z",rclcpp::PARAMETER_DOUBLE);
      this -> declare_parameter("roll",rclcpp::PARAMETER_DOUBLE);
      this -> declare_parameter("pitch",rclcpp::PARAMETER_DOUBLE);
      this -> declare_parameter("yaw",rclcpp::PARAMETER_DOUBLE);
      this -> get_parameter("parent_frame",parent_frame_initial);
      this -> get_parameter("child_frame",child_frame_initial);
      this -> get_parameter("x",x_initial);
      this -> get_parameter("y",y_initial);
      this -> get_parameter("z",z_initial);
      rclcpp::Time t = this -> now();
      tfRegistration(t);
    }
  private:
  void tfRegistration(const rclcpp::Time &timeStamp)
  {
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = timeStamp;
    transformStamped.header.frame_id = parent_frame_;
    transformStamped.child_frame_id = child_frame_;
    transformStamped.transform.translation.x = pose_.x;
    transformStamped.transform.translation.y = pose_.y;
    transformStamped.transform.translation.z = pose_.z;
    tf2::Quaternion q;
    q.setRPY(pose_.roll, pose_.pitch, pose_.yaw);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    
    RCLCPP_DEBUG(this -> get_logger(), "transformation: x=%f y=%f z=%f",  transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
    RCLCPP_DEBUG(this -> get_logger(), "rotation: x=%f y=%f z=%f w=%f",  transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
    tf_static_broadcaster_.broadcast(transformStamped);
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  string parent_frame_initial, child_frame_initial;
  float x_initial,y_initial,z_initial,roll_initial,yaw,pitch;
}



int main(int argc, char *argv[])
  {
  rclcpp::init(argc, argv);
  rclcpp::Node node  = AlignmentPublisher();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
