#include "icp_align_tool/icp_align_tool.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv){
  //TODO: make a 4 threaded executor and boot up with node options
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<Multi_Sensor_Alignment::Cloud_Alignment>(options, 10);
  executor.add_node(node);
  executor.spin(); // spin() will not return until the node has been shutdown
  rclcpp::shutdown();
  return 0;
}

