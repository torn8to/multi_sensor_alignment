#include "icp_align_tool/icp_align_tool.h"

int main(int argc, char **argv)
{
  double control_frequency = 10.0;
  int buffer_size = 1;


  rclcpp::init(argc,argv);
  rclcpp::executors::MultiThreadedExecutor executor; // runs on the amount of threads = to number of cores in this configuration
  
  Multi_Sensor_Alignment::Cloud_Alignment node(buffer_size);
  executor.add_node(node);
  executor.spin();
  return 0;
}

