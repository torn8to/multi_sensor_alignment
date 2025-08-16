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
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
  
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <memory>
#include <chrono>
#include <set>
#include <map>




 

namespace Multi_Sensor_Alignment
{
  class ReconfigurableStaticTransformBroadcaster: public rclcpp::Node{
  public:
    ReconfigurableStaticTransformBroadcaster(): Node("simple tf broadcaster"){
      this->declare_parameter("auto_restart", true);
      this->declare_parameter("child_frame", "child_lidar");
      this->declare_parameter("parent_frame", "parent_lidar");
      this->declare_parameter("auto_restart", true);
      this->declare_parameter("broadcasting", true);
      this->declare_parameter(listener_x_param, 0.0);
      this->declare_parameter(listener_y_param, 0.0);
      this->declare_parameter(listener_z_param, 0.0);
      this->declare_parameter(listener_roll_param, 0.0);
      this->declare_parameter(listener_pitch_param, 1.0);
      this->declare_parameter(listener_yaw_param, 0.0);

      tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
      
      callback_handle_ = this->add_on_set_parameters_callback(std::bind(&ReconfigurableStaticTransformBroadcaster::parameterCallback,
                                                                   this,
                                                                   std::placeholders::_1));
    }

    ~ReconfigurableStaticTransformBroadcaster(){} // Empty deconstructor

    void tfRegistration()
    {
      geometry_msgs::msg::TransformStamped transformStamped;
      transformStamped.header.stamp = this->get_clock()->now();
      transformStamped.header.frame_id = parent_frame_;
      transformStamped.child_frame_id = child_frame_;
      transformStamped.transform.translation.x = x_transform_;
      transformStamped.transform.translation.y = y_transform_;
      transformStamped.transform.translation.z = z_transform_;
      tf2::Quaternion q;
      q.setRPY(roll_transform_, pitch_transform_, yaw_transform_);
      transformStamped.transform.rotation.x = q.x();
      transformStamped.transform.rotation.y = q.y();
      transformStamped.transform.rotation.z = q.z();
      transformStamped.transform.rotation.w = q.w();
      this->tf_static_broadcaster_->sendTransform(transformStamped);
    }

  rcl_interfaces::msg::SetParametersResult parameterCallback(const std::vector<rclcpp::Parameter> &parameters){
      // NOTE: this only supports static typing if you override with dynamic you will get run time errors for changing types
      rcl_interfaces::msg::SetParametersResult result;
      auto element = std::find_if(parameters.begin(), parameters.end(),[&] (const auto &param)
      {
        if(param.get_name() == listener_yaw_param)
        {
          if(param.as_double() > M_PI/2 || param.as_double() < (-M_PI)/2)
          {
            RCLCPP_ERROR(get_logger(),"new pitch value outside of range -pi to pi got %lf",
                      param.as_double());
            result.successful = false;
            result.reason = "invalid parameter type not double";
          }
          else
          {
            result.successful = true;
            result.reason = "invalid parameter type not double";
            yaw_transform_ = param.as_double();
          }
        }
        else if(param.get_name() == listener_pitch_param)
        {
          if(param.as_double() > M_PI || param.as_double() < -M_PI)
          {
            RCLCPP_ERROR(get_logger(),"new pitch value outside of range -pi to pi got %lf",
                      param.as_double());
            result.successful = false;
            result.reason = "invalid parameter type not double";
          }
          else
          {
            result.successful = true;
            result.reason = "invalid parameter type not double";
            pitch_transform_ = param.as_double();
          }
        }
        else if(param.get_name() == listener_roll_param)
        {
          if(param.as_double() > M_PI || param.as_double() < -M_PI)
          {
            RCLCPP_ERROR(get_logger(),"new roll value outside of range -pi to pi got %lf",
                      param.as_double());
            result.successful = false;
            result.reason = "parameter_not in roll range";
          }
          else
          {
            result.successful = true;
            result.reason = "success";
            roll_transform_ = param.as_double();
          }
        }
        else if(param.get_name() == listener_x_param)
        {
          result.successful = true;
          result.reason = "success";
          x_transform_ = param.as_double();
        }
        else if(param.get_name() == listener_y_param)
        {
          result.successful = true;
          result.reason = "success";
          y_transform_ = param.as_double();
        }
        else if(param.get_name() == listener_z_param)
        {
          result.successful = true;
          result.reason = "success";
          z_transform_ = param.as_double();
        }
        else
        {
          //NOTE: no extra checks for other params as those 
          result.successful = true;
          result.reason = "success";
        }
        return result.successful == false;
    });

      if(element == parameters.end()) tfRegistration();
      return result;
    }

  private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    const std::string listener_x_param = "transform_x";
    const std::string listener_y_param = "transform_y";
    const std::string listener_z_param = "transform_z";
    const std::string listener_roll_param = "transform_roll";
    const std::string listener_pitch_param = "transform_pitch";
    const std::string listener_yaw_param = "transform_yaw";
    const std::string child_frame_param_name = "child_frame";
    const std::string base_frame_param_name = "parent_frame";

    double x_transform_, y_transform_, z_transform_, roll_transform_, pitch_transform_, yaw_transform_;

    std::string lidar_id_str_;
    std::string child_frame_;
    std::string parent_frame_;

  };

  
} // namespace Multi_Sensor_Alignment

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Multi_Sensor_Alignment::ReconfigurableStaticTransformBroadcaster>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
