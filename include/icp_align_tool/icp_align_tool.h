/* 

Copyright (c) 2017
*/

#ifndef ICP_ALIGN_TOOL_H
#define ICP_ALIGN_TOOL_H

#include <iostream>
#include <vector> 
#include <sstream>

#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>

#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/ndt.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>


using namespace boost::accumulators;

//convenient typedefs
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

/**
 * \brief Cross-registers pointcloud2 topics for use in alignment
 */

namespace Multi_Sensor_Alignment
{
  // reolace the dynamic reconfigure alignmnet tool with a struct
  

  struct Alignment_Tool_Config{
    int method_;

    int    norm_kSearch_;
    double norm_RadiusSearch_;

    double epsilon_;
    int    maxIterations_;
    double maxCorrespondenceDistance_;
   
    double ndt_StepSize_;
    double ndt_Resolution_;

    double voxelSize_;
    double filter_i_min_;
    double filter_i_max_;
    double filter_x_min_;
    double filter_x_max_;
    double filter_y_min_;
    double filter_y_max_;
    double filter_z_min_;
    double filter_z_max_;
  };


  struct Alignment_Tool_Pose{
        std::string parent_frame_id;
        std::string child_frame_id;

        double x;
        double y;
        double z;
        double roll;
        double pitch;
        double yaw;

        // flags to push updates
        bool parent_frame_flag;
        bool child_frame_flag;
        bool x_update_flag;
        bool y_update_flag;
        bool z_update_flag;
        bool roll_update_flag;
        bool pitch_update_flag;
        bool yaw_update_flag;

  };

  class Cloud_Alignment: public rclcpp::Node
  {
  typedef accumulator_set<double, stats<tag::rolling_mean > > window_acc;
  
  public:
    Cloud_Alignment(const rclcpp::NodeOptions &options, const int buffer_size);

    ~Cloud_Alignment();

    /** 
     * Main run loop
     */
    void onInit();

    //! Callback
    void input0_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void input1_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
    //publisher
    void publish_callback();

    std::string node_name{"cloud_alignment"};
  private:
    double PI = atan(1)*4;
    
    //Methods
    bool revert();
    bool reset();
    bool pushTransform();
    bool pushYaw();
    bool pushRollPitchCorrection();
    std::vector<rclcpp::Parameter> EvalAlignmentToolPoseToParams(const Alignment_Tool_Pose &pose_update);
    static bool AreQuaternionsClose(tf2::Quaternion q1, tf2::Quaternion q2);
    geometry_msgs::msg::Quaternion AverageQuaternion(const geometry_msgs::msg::Quaternion& newRotation);
    void DownsampleCloud(const pcl::PointCloud<PointT>::Ptr in_cloud, pcl::PointCloud<PointT> &out_cloud, double in_leaf_size);

    //service callbacks
    void freeze0_callback(const std_srvs::srv::Empty::Request::SharedPtr req,
            std_srvs::srv::Empty::Response::SharedPtr resp);
    void freeze1_callback(const std_srvs::srv::Empty::Request::SharedPtr req,
            std_srvs::srv::Empty::Response::SharedPtr resp);
    void unfreeze0_callback(const std_srvs::srv::Empty::Request::SharedPtr req,
            std_srvs::srv::Empty::Response::SharedPtr resp);
    void unfreeze1_callback(const std_srvs::srv::Empty::Request::SharedPtr req,
            std_srvs::srv::Empty::Response::SharedPtr resp);
    void revert_callback(const std_srvs::srv::Empty::Request::SharedPtr req,
            std_srvs::srv::Empty::Response::SharedPtr resp);
    void reset_callback(const std_srvs::srv::Empty::Request::SharedPtr req,
            std_srvs::srv::Empty::Response::SharedPtr resp);
    void pushtransform_callback(const std_srvs::srv::Empty::Request::SharedPtr req,
            std_srvs::srv::Empty::Response::SharedPtr resp);
    void pushYaw_callback(const std_srvs::srv::Empty::Request::SharedPtr req,
            std_srvs::srv::Empty::Response::SharedPtr resp);
    void pushRollPitchCorrection_callback(const std_srvs::srv::Empty::Request::SharedPtr req,
            std_srvs::srv::Empty::Response::SharedPtr resp);
    

    std::string parent_frame_id_, child_frame_id_;
    std::string output_trans_topic_;
    std::string output_cloud0_topic_, output_cloud1_topic_;
    std::string align_server_name_;
    //ros::Publisher output_trans_pub_,  output_cloud0_pub_, output_cloud1_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_cloud0_pub_, output_cloud1_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr output_trans_pub_;
    rclcpp::SyncParametersClient::SharedPtr  static_transform_publisher_param_client_;
    rclcpp::TimerBase::SharedPtr pub_timer_;

    std::shared_ptr<tf2_ros::TransformListener >tfListener_;
    std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
    float wait_for_tf_delay_;

    std::string input0_topic_, input1_topic_;
    sensor_msgs::msg::PointCloud2::SharedPtr cloud0_, cloud1_;
    //ros::Subscriber input_sub0_, input_sub1_;
    //ros::ServiceServer service0_, service1_, service2_, service3_, service4_, service5_, service6_, service7_, service8_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_sub0_, input_sub1_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service0_, service1_, service2_, service3_, service4_, service5_, service6_, service7_, service8_;

    bool freeze0_, freeze1_, is_output_filtered_;

    int buffer_size_;

    Eigen::Matrix4f current_guess_;
    geometry_msgs::msg::TransformStamped::Ptr output_;
    geometry_msgs::msg::Transform last_transform_;
    window_acc x_array_;
    window_acc y_array_;
    window_acc z_array_;
    window_acc qw_array_;
    window_acc qx_array_;
    window_acc qy_array_;
    window_acc qz_array_;
    double current_qw_, current_qx_, current_qy_, current_qz_;

    Alignment_Tool_Config icp_config_;

    std::recursive_mutex cloud0_mutex_;
    std::recursive_mutex cloud1_mutex_;
    double output_frequency_;

    bool parameter_client_connected_;
   
  }; // class Cloud_Alignment
} // namespace Multi_Sensor_Alignment

#endif  // ICP_ALIGN_TOOL_H

