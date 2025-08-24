#include "icp_align_tool/icp_align_tool.h"

namespace Multi_Sensor_Alignment
{
  Cloud_Alignment::Cloud_Alignment(const rclcpp::NodeOptions& options = rclcpp::NodeOptions(), 
                                   const int buffer_size=10):
  // Initialization list
  Node(node_name, options),
  freeze0_(0),
  freeze1_(0),
  current_guess_(Eigen::Matrix4f::Identity()),
  output_(new geometry_msgs::msg::TransformStamped),
  buffer_size_(buffer_size),
  x_array_(tag::rolling_window::window_size = buffer_size),
  y_array_(tag::rolling_window::window_size = buffer_size),
  z_array_(tag::rolling_window::window_size = buffer_size),
  qw_array_(tag::rolling_window::window_size = buffer_size),
  qx_array_(tag::rolling_window::window_size = buffer_size),
  qy_array_(tag::rolling_window::window_size = buffer_size),
  qz_array_(tag::rolling_window::window_size = buffer_size),
  current_qw_(0), current_qx_(0), current_qy_(0), current_qz_(0),
  wait_for_tf_delay_(0.1)
  {
    this->onInit();
  }


  Cloud_Alignment::~Cloud_Alignment()
  {
    // pass
  }

  void Cloud_Alignment::onInit()
  {
    // Node parameters
    this->declare_parameter("parent_frame","");
    this->declare_parameter("child_frame","");
    this->declare_parameter("output_frequency", 1.00);
    this->declare_parameter("voxelSize", 1.0);

    this->declare_parameter("input_cloud0", "input0");
    this->declare_parameter("input_cloud1", "input1");
    this->declare_parameter("output_cloud0", "output0");
    this->declare_parameter("output_cloud1", "output1");
    this->declare_parameter("output", "transform");
    this->declare_parameter("is_output_filtered", false);

    this->declare_parameter("filter/i_min", 0);
    this->declare_parameter("filter/i_max", 100);
    this->declare_parameter("filter/x_min", -50.0);
    this->declare_parameter("filter/x_max", 50.0);
    this->declare_parameter("filter/y_min", -50.0);
    this->declare_parameter("filter/y_max", 50.0);
    this->declare_parameter("filter/z_min", -1.0);
    this->declare_parameter("filter/z_max", 5.0);
    // 0 = ICP Nonlinear with scaling CorDist
    // 1 = ICP Nonlinear
    // 2 = Normal Distribution Transform
    // 3 = ICP with normals
    this->declare_parameter("method", 1);
    this->declare_parameter("epsilon", 0.001);
    this->declare_parameter("maxIterations", 1000);
    this->declare_parameter("maxCoorespondenceDistance", 0.001);
    
    this->declare_parameter("norm/KSearchs", 20);
    this->declare_parameter("norm/RadiusSearch", 0.5);

    this->declare_parameter("ndt/StepSize", 0.001);
    this->declare_parameter("ndt/Resolution", 1.0);

    // dynamic reconfigure parameters replacing the ros1 dynamic reconfigure server using ros2 parameter callback listening as it works internode
    this->declare_parameter("monitored_node_name", "static_transform_publisher");
    this->declare_parameter("x_param_listener", "transform_x_listener");
    this->declare_parameter("y_param_listener", "transform_y_listener");
    this->declare_parameter("z_param_listener", "transform_z_listener");
    this->declare_parameter("roll_param_listener", "transform_roll_listener"); 
    this->declare_parameter("pitch_param_listener", "transform_pitch_listener");
    this->declare_parameter("yaw_param_listener", "transform_yaw_listener");
    this->declare_parameter("reconfigure/x", 0.0);
    this->declare_parameter("reconfigure/y", 0.0);
    this->declare_parameter("reconfigure/z", 0.0);
    this->declare_parameter("reconfigure/roll", 0.0);
    this->declare_parameter("reconfigure/pitch", 0.0);
    this->declare_parameter("reconfigure/yaw", 0.0);

    // Get parameter values
    parent_frame_id_ = this->get_parameter("parent_frame").as_string();
    child_frame_id_ = this->get_parameter("child_frame").as_string();
    output_frequency_ = this->get_parameter("output_frequency").as_double();
    is_output_filtered_ = this->get_parameter("is_output_filtered").as_bool();
    
    input0_topic_ = this->get_parameter("input_cloud0").as_string();
    input1_topic_ = this->get_parameter("input_cloud1").as_string();
    output_cloud0_topic_ = this->get_parameter("output_cloud0").as_string();
    output_cloud1_topic_ = this->get_parameter("output_cloud1").as_string();
    output_trans_topic_ = this->get_parameter("output").as_string();

    tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

    // ROS publishers
    output_trans_pub_  = this->create_publisher<geometry_msgs::msg::TransformStamped>(output_trans_topic_,100);
    output_cloud0_pub_  = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud0_topic_,10);
    output_cloud1_pub_  = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud1_topic_,10);
    pub_timer_ = this->create_wall_timer(std::chrono::microseconds(static_cast<long>(1000000/output_frequency_)), 
                                       std::bind(&Cloud_Alignment::publish_callback, this));
  //TODO: figure out the out how to create time publisher

  // ROS subscribers
    input_sub0_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(input0_topic_, 100, std::bind(&Cloud_Alignment::input0_callback, this, std::placeholders::_1));
    input_sub1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(input1_topic_, 100, std::bind(&Cloud_Alignment::input1_callback, this, std::placeholders::_1));

    // ROS Services
    //
    service0_ = this->create_service<std_srvs::srv::Empty>("freeze_cloud0", std::bind(&Cloud_Alignment::freeze0_callback, this, std::placeholders::_1, std::placeholders::_2));
    service1_ = this->create_service<std_srvs::srv::Empty>("freeze_cloud1", std::bind(&Cloud_Alignment::freeze1_callback, this, std::placeholders::_1, std::placeholders::_2));
    service2_ = this->create_service<std_srvs::srv::Empty>("unfreeze_cloud0", std::bind(&Cloud_Alignment::unfreeze0_callback, this, std::placeholders::_1, std::placeholders::_2));
    service3_ = this->create_service<std_srvs::srv::Empty>("unfreeze_cloud1", std::bind(&Cloud_Alignment::unfreeze1_callback, this, std::placeholders::_1, std::placeholders::_2));
    service4_ = this->create_service<std_srvs::srv::Empty>("revert", std::bind(&Cloud_Alignment::revert_callback, this,std::placeholders::_1, std::placeholders::_2));

    service5_ = this->create_service<std_srvs::srv::Empty>("reset", std::bind(&Cloud_Alignment::reset_callback, this, std::placeholders::_1, std::placeholders::_2));
    service6_ = this->create_service<std_srvs::srv::Empty>("push_transform", std::bind(&Cloud_Alignment::pushtransform_callback, this, std::placeholders::_1, std::placeholders::_2));
    service7_ = this->create_service<std_srvs::srv::Empty>("push_yaw", std::bind(&Cloud_Alignment::pushYaw_callback, this,std::placeholders::_1, std::placeholders::_2));
    service8_ = this->create_service<std_srvs::srv::Empty>("push_roll_pitch_correction", std::bind(&Cloud_Alignment::pushRollPitchCorrection_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    static_transform_publisher_param_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, this->get_parameter("monitored_node_name").as_string());

    // sync parameter client
    rclcpp::sleep_for(std::chrono::seconds(1));
    if(static_transform_publisher_param_client_->service_is_ready()){
      RCLCPP_INFO(this->get_logger(),"synchronous param client initialized for %s", node_name.c_str());

    }

    bool has_x_transform_parameter = static_transform_publisher_param_client_->has_parameter(this->get_parameter("x_param_listener").as_string());
    bool has_y_transform_parameter = static_transform_publisher_param_client_->has_parameter(this->get_parameter("y_param_listener").as_string());
    bool has_z_transform_parameter = static_transform_publisher_param_client_->has_parameter(this->get_parameter("z_param_listener").as_string());
    bool has_roll_transform_parameter = static_transform_publisher_param_client_->has_parameter(this->get_parameter("roll_param_listener").as_string());
    bool has_pitch_transform_parameter = static_transform_publisher_param_client_->has_parameter(this->get_parameter("pitch_param_listener").as_string());
    bool has_yaw_transform_parameter = static_transform_publisher_param_client_->has_parameter(this->get_parameter("yaw_param_listener").as_string());

     
    if(!has_x_transform_parameter) RCLCPP_INFO(this->get_logger(),"does not have x transform listener expected %s", this->get_parameter("x_param_listener").as_string().c_str());
    if(!has_y_transform_parameter) RCLCPP_INFO(this->get_logger(),"does not have y transform listener expected %s", this->get_parameter("y_param_listener").as_string().c_str());
    if(!has_z_transform_parameter) RCLCPP_INFO(this->get_logger(),"does not have z transform listener expected %s", this->get_parameter("z_param_listener").as_string().c_str());
    if(!has_roll_transform_parameter) RCLCPP_INFO(this->get_logger(),"does not have roll transform listener expected %s", this->get_parameter("roll_param_listener").as_string().c_str());
    if(!has_pitch_transform_parameter) RCLCPP_INFO(this->get_logger(),"does not have pitch transform listener expected %s", this->get_parameter("pitch_param_listener").as_string().c_str());
    if(!has_yaw_transform_parameter) RCLCPP_INFO(this->get_logger(),"does not have yaw transform listener expected %s", this->get_parameter("yaw_param_listener").as_string().c_str());
    if(has_x_transform_parameter && has_y_transform_parameter && 
      has_z_transform_parameter && has_roll_transform_parameter && 
      has_pitch_transform_parameter && has_yaw_transform_parameter){
        RCLCPP_INFO(this->get_logger(), "sync parameter_client has access to all needed parameters");
      }
    if (has_x_transform_parameter &&
      has_y_transform_parameter &&
      has_z_transform_parameter &&
      has_roll_transform_parameter &&
      has_pitch_transform_parameter &&
      has_yaw_transform_parameter)
      {
      parameter_client_connected_ = true;
    }
    else
    {

    }
    
    // initialize icp configuration
    icp_config_.method_ = this->get_parameter("method").as_int();
    icp_config_.norm_kSearch_ = this->get_parameter("norm/KSearchs").as_int();
    icp_config_.norm_RadiusSearch_ = this->get_parameter("norm/RadiusSearch").as_double();
    icp_config_.epsilon_ = this->get_parameter("epsilon").as_double();
    icp_config_.maxIterations_ = this->get_parameter("maxIterations").as_int();
    icp_config_.maxCorrespondenceDistance_ = this->get_parameter("maxCoorespondenceDistance").as_double();
    icp_config_.ndt_StepSize_ = this->get_parameter("ndt/StepSize").as_double();
    icp_config_.ndt_Resolution_ = this->get_parameter("ndt/Resolution").as_double();
    icp_config_.voxelSize_ = this->get_parameter("voxelSize").as_double();
    icp_config_.filter_i_min_ = this->get_parameter("filter/i_min").as_double();
    icp_config_.filter_i_max_ = this->get_parameter("filter/i_max").as_double();
    icp_config_.filter_x_min_ = this->get_parameter("filter/x_min").as_double();
    icp_config_.filter_x_max_ = this->get_parameter("filter/x_max").as_double();
    icp_config_.filter_y_min_ = this->get_parameter("filter/y_min").as_double();
    icp_config_.filter_y_max_ = this->get_parameter("filter/y_max").as_double();
    icp_config_.filter_z_min_ = this->get_parameter("filter/z_min").as_double();


    Cloud_Alignment::revert();
    RCLCPP_INFO(this->get_logger(),"%s initialized", node_name.c_str());
  }

  std::vector<rclcpp::Parameter> Cloud_Alignment::EvalAlignmentToolPoseToParams(const Alignment_Tool_Pose &pose_update)
  {
      std::string x_transform_param  = this->get_parameter("x_param_listener").as_string();
      std::string y_transform_param  = this->get_parameter("y_param_listener").as_string();
      std::string z_transform_param  = this->get_parameter("z_param_listener").as_string();
      std::string roll_transform_param  = this->get_parameter("roll_param_listener").as_string();
      std::string pitch_transform_param  = this->get_parameter("pitch_param_listener").as_string();
      std::string yaw_transform_param  = this->get_parameter("yaw_param_listener").as_string();

      std::vector<rclcpp::Parameter> results;
      results.reserve(8);

      if(pose_update.parent_frame_flag) results.push_back(rclcpp::Parameter("child_frame", pose_update.parent_frame_id));
      if(pose_update.child_frame_flag) results.push_back(rclcpp::Parameter("parent_frame", pose_update.child_frame_id));
      if(pose_update.x_update_flag) results.push_back(rclcpp::Parameter(x_transform_param, pose_update.x));
      if(pose_update.y_update_flag) results.push_back(rclcpp::Parameter(y_transform_param, pose_update.y));
      if(pose_update.z_update_flag) results.push_back(rclcpp::Parameter(z_transform_param, pose_update.z));
      if(pose_update.roll_update_flag) results.push_back(rclcpp::Parameter(roll_transform_param, pose_update.roll));
      if(pose_update.pitch_update_flag) results.push_back(rclcpp::Parameter(pitch_transform_param, pose_update.pitch));
      if(pose_update.yaw_update_flag) results.push_back(rclcpp::Parameter(yaw_transform_param, pose_update.yaw));

      return results;
  }
    

  bool Cloud_Alignment::pushTransform()
  {
    if(!parameter_client_connected_)
    {
      RCLCPP_INFO(this->get_logger(), "Parameter client isn't connected.");
      return true;
    }

    tf2::Quaternion q;
    tf2::convert(output_->transform.rotation, q);
    tf2::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);

    struct Multi_Sensor_Alignment::Alignment_Tool_Pose pose_update{
      .parent_frame_id = "",
      .child_frame_id = "",
      .x = output_->transform.translation.x,
      .y = output_->transform.translation.y,
      .z = output_->transform.translation.z,
      .roll = roll,
      .pitch = pitch,
      .yaw = yaw,
      .parent_frame_flag = false,
      .child_frame_flag = false,
      .x_update_flag = true,
      .y_update_flag = true,
      .z_update_flag = true,
      .roll_update_flag = true,
      .pitch_update_flag = true,
      .yaw_update_flag = true,
    };

    std::vector<rclcpp::Parameter> params = Cloud_Alignment::EvalAlignmentToolPoseToParams(pose_update);
    rcl_interfaces::msg::SetParametersResult result = static_transform_publisher_param_client_->set_parameters_atomically(params);
    return result.successful; 
  }
  
  bool Cloud_Alignment::pushYaw()
  {
    if(!parameter_client_connected_)
    {
      RCLCPP_INFO(this->get_logger(), "Parameter client isn't connected.");
      return true;
    }
    tf2::Quaternion q;
    tf2::convert(output_->transform.rotation, q);
    tf2::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);

    struct Multi_Sensor_Alignment::Alignment_Tool_Pose pose_update{
      .parent_frame_id = "",
      .child_frame_id = "",
      .x = 0.0,
      .y = 0.0,
      .z = 0.0,
      .roll = 0.0,
      .pitch = 0.0,
      .yaw = yaw,
      .parent_frame_flag = false,
      .child_frame_flag = false,
      .x_update_flag = false,
      .y_update_flag = false,
      .z_update_flag = false,
      .roll_update_flag = false,
      .pitch_update_flag = false,
      .yaw_update_flag = true,
    };

    std::vector<rclcpp::Parameter> params = EvalAlignmentToolPoseToParams(pose_update);
    rcl_interfaces::msg::SetParametersResult result = static_transform_publisher_param_client_->set_parameters_atomically(params);
    return result.successful; 
  }
  
  bool Cloud_Alignment::pushRollPitchCorrection()
  {
    if(!parameter_client_connected_)
    {
      RCLCPP_INFO(this->get_logger(), "Parameter client isn't connected.");
      return true;
    }

    tf2::Quaternion q;
    tf2::convert(output_->transform.rotation, q);
    tf2::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);

    struct Multi_Sensor_Alignment::Alignment_Tool_Pose pose_update{
      .parent_frame_id = "",
      .child_frame_id = "",
      .x = 0.0,
      .y = 0.0,
      .z = 0.0,
      .roll = roll,
      .pitch = pitch,
      .yaw = 0.0,
      .parent_frame_flag = false,
      .child_frame_flag = false,
      .x_update_flag = false,
      .y_update_flag = false,
      .z_update_flag = false,
      .roll_update_flag = true,
      .pitch_update_flag = true,
      .yaw_update_flag = false,
    };

    std::vector<rclcpp::Parameter> params = EvalAlignmentToolPoseToParams(pose_update);
    rcl_interfaces::msg::SetParametersResult result = static_transform_publisher_param_client_->set_parameters_atomically(params);
    return result.successful; 
  }
  
  void Cloud_Alignment::publish_callback()
  {
    RCLCPP_INFO(this->get_logger(), "");

    if(cloud0_->data.size() <= 0 || cloud1_->data.size() <=0) return;

  // Convert from ROS msg to pointcloud2 object
    pcl::PointCloud<PointT>::Ptr cloud0(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud1(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*cloud0_, *cloud0);
    pcl::fromROSMsg(*cloud1_, *cloud1);

    std::string parent_frame = cloud0_->header.frame_id;
    std::string child_frame  = cloud1_->header.frame_id;

  //Downsample
    pcl::PointCloud<PointT>::Ptr filtered_cloud0(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr filtered_cloud1(new pcl::PointCloud<PointT>);
    DownsampleCloud(cloud0, *filtered_cloud0, icp_config_.voxelSize_);
    DownsampleCloud(cloud1, *filtered_cloud1, icp_config_.voxelSize_);
    
    RCLCPP_INFO(this->get_logger(), "\n");
    
  //Perform Registration
    Eigen::Matrix4f prev;
    pcl::PointCloud<PointT>::Ptr output_cloud0(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr output_cloud1(new pcl::PointCloud<PointT>);

    // ICP Nonlinear with scaling CorDist
    if(icp_config_.method_ == 0)
    {
      // Compute surface normals and curvature
      PointCloudWithNormals::Ptr points_with_normals0 (new PointCloudWithNormals);
      PointCloudWithNormals::Ptr points_with_normals1 (new PointCloudWithNormals);

      pcl::NormalEstimation<PointT, PointNormalT> norm_est;
      pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
      norm_est.setSearchMethod (tree);
      if(icp_config_.norm_kSearch_ > 0) norm_est.setKSearch(icp_config_.norm_kSearch_);
      else norm_est.setRadiusSearch(icp_config_.norm_RadiusSearch_);
      
      norm_est.setInputCloud (filtered_cloud0);
      norm_est.compute (*points_with_normals0);
      pcl::copyPointCloud (*filtered_cloud0, *points_with_normals0);

      norm_est.setInputCloud (filtered_cloud1);
      norm_est.compute (*points_with_normals1);
      pcl::copyPointCloud (*filtered_cloud1, *points_with_normals1);

      // Align
      pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
      reg.setTransformationEpsilon (icp_config_.epsilon_);
      // Set the maximum distance between two correspondences (cloud0<->cloud1) to user input
      // Note: adjust this based on the size of your datasets
      reg.setMaxCorrespondenceDistance (icp_config_.maxCorrespondenceDistance_);  
      // Set the first pointcloud as the target
      reg.setInputTarget (points_with_normals0);

      // Run the optimization in a loop and visualize the results
      PointCloudWithNormals::Ptr reg_result = points_with_normals1;
      reg.setMaximumIterations (2);
      for (int i = 0; i < icp_config_.maxIterations_; ++i)
      {
        RCLCPP_DEBUG(this->get_logger(),"Iteration Nr. %d maxCorrespondenceDistance=%f.\n", i, reg.getMaxCorrespondenceDistance());

        // save previous cloud
        points_with_normals1 = reg_result;

        // Estimate
        reg.setInputSource (points_with_normals1);
        reg.align (*reg_result, current_guess_);
      
        //accumulate transformation between each Iteration
        if(reg.hasConverged ())
        {
          current_guess_ = reg.getFinalTransformation ();

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
          if (std::abs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
                        reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () * 0.99);
        }
        
        prev = reg.getLastIncrementalTransformation();
      }

      RCLCPP_INFO(this->get_logger(), "ICP Nonlinear Transform converged: %s score: %f epsilon: %f", 
                  reg.hasConverged() ? "true" : "false", reg.getFitnessScore(), reg.getTransformationEpsilon());

    }
    // ICP Nonlinear
    else if(icp_config_.method_ == 1)
    {
      // Compute surface normals and curvature
      PointCloudWithNormals::Ptr points_with_normals0 (new PointCloudWithNormals);
      PointCloudWithNormals::Ptr points_with_normals1 (new PointCloudWithNormals);

      pcl::NormalEstimation<PointT, PointNormalT> norm_est;
      pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
      norm_est.setSearchMethod(tree);
      if(icp_config_.norm_kSearch_ > 0) norm_est.setKSearch(icp_config_.norm_kSearch_);
      else norm_est.setRadiusSearch(icp_config_.norm_RadiusSearch_);
      
      norm_est.setInputCloud (filtered_cloud0);
      norm_est.compute (*points_with_normals0);
      pcl::copyPointCloud (*filtered_cloud0, *points_with_normals0);

      norm_est.setInputCloud (filtered_cloud1);
      norm_est.compute (*points_with_normals1);
      pcl::copyPointCloud (*filtered_cloud1, *points_with_normals1);

      // Align
      pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
      reg.setTransformationEpsilon (icp_config_.epsilon_);
      // Set the maximum distance between two correspondences (cloud0<->cloud1) to user input
      // Note: adjust this based on the size of your datasets
      reg.setMaxCorrespondenceDistance (icp_config_.maxCorrespondenceDistance_);  
      reg.setMaximumIterations(icp_config_.maxIterations_);
      // Set the first pointcloud as the target
      reg.setInputTarget (points_with_normals0);
      reg.setInputSource (points_with_normals1);

      PointCloudWithNormals::Ptr reg_result = points_with_normals1;
      
      //Get Results
      reg.align (*reg_result, current_guess_);

      RCLCPP_INFO(this->get_logger(), "ICP Nonlinear Transform converged: %s score: %f epsilon: %f", 
                  reg.hasConverged() ? "true" : "false", reg.getFitnessScore(), reg.getTransformationEpsilon());
      
      if(reg.hasConverged() ) current_guess_ = reg.getFinalTransformation();

    }
    // Normal Distributions Transform
    else if(icp_config_.method_ == 2)
    {
      // Initializing Normal Distributions Transform (NDT).
      pcl::NormalDistributionsTransform<PointT, PointT> ndt;

      ndt.setTransformationEpsilon(icp_config_.epsilon_);
      ndt.setStepSize(icp_config_.ndt_StepSize_);
      ndt.setResolution(icp_config_.ndt_Resolution_);
      ndt.setMaximumIterations(icp_config_.maxIterations_);

      // Set the first pointcloud as the target
      ndt.setInputTarget(cloud0);
      ndt.setInputSource(filtered_cloud1);

      //Get Results
      ndt.align(*output_cloud1, current_guess_);
      
      RCLCPP_INFO(this->get_logger(), "Normal Distributions Transform converged: %s score: %f prob: %f", 
                  ndt.hasConverged() ? "true" : "false", ndt.getFitnessScore(), ndt.getTransformationProbability());

      if(ndt.hasConverged() ) 
      {
        current_guess_ = ndt.getFinalTransformation();
      
        Eigen::Matrix3f rotation_matrix = current_guess_.block(0,0,3,3);
        Eigen::Vector3f translation_vector = current_guess_.block(0,3,3,1);
        RCLCPP_INFO(this->get_logger(), "This transformation can be replicated using:");
        RCLCPP_INFO(this->get_logger(), "rosrun tf static_transform_publisher %f %f %f %f %f %f /%s /%s 10", 
                    translation_vector.transpose().x(), translation_vector.transpose().y(), translation_vector.transpose().z(),
                    rotation_matrix.eulerAngles(2,1,0).transpose().x(), rotation_matrix.eulerAngles(2,1,0).transpose().y(), rotation_matrix.eulerAngles(2,1,0).transpose().z(),
                    parent_frame.c_str(), child_frame.c_str());
      }

    }
    // ICP with Normals
    else if(icp_config_.method_ == 3)
    {
      #if (defined(PCL_VERSION) && PCL_VERSION_COMPARE(<, 1, 10, 1))
        throw std::runtime_error("PCL must be at or above version 1.10.1 for this method=3");
      #else

        // Compute surface normals and curvature
        PointCloudWithNormals::Ptr points_with_normals0 (new PointCloudWithNormals);
        PointCloudWithNormals::Ptr points_with_normals1 (new PointCloudWithNormals);

        pcl::NormalEstimation<PointT, PointNormalT> norm_est;
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
        norm_est.setSearchMethod (tree);
        
        if(icp_config_.norm_kSearch_ > 0) norm_est.setKSearch (icp_config_.norm_kSearch_);
        else norm_est.setRadiusSearch(icp_config_.norm_RadiusSearch_);
        
        norm_est.setInputCloud (filtered_cloud0);
        norm_est.compute (*points_with_normals0);
        pcl::copyPointCloud (*filtered_cloud0, *points_with_normals0);

        norm_est.setInputCloud (filtered_cloud1);
        norm_est.compute (*points_with_normals1);
        pcl::copyPointCloud (*filtered_cloud1, *points_with_normals1);

        // Align
        pcl::IterativeClosestPointWithNormals<PointNormalT, PointNormalT> reg;
        reg.setTransformationEpsilon (icp_config_.epsilon_);
        // Set the maximum distance between two correspondences (cloud0<->cloud1) to user input
        // Note: adjust this based on the size of your datasets
        reg.setMaxCorrespondenceDistance (icp_config_.maxCorrespondenceDistance_);  
        reg.setMaximumIterations(icp_config_.maxIterations_);
        reg.setUseSymmetricObjective(true);
        reg.setEnforceSameDirectionNormals(true);
        // Set the first pointcloud as the target
        reg.setInputTarget (points_with_normals0);
        reg.setInputSource (points_with_normals1);

        PointCloudWithNormals::Ptr reg_result = points_with_normals1;
        
        //Get Results
        reg.align (*reg_result, current_guess_);

        RCLCPP_INFO(this->get_logger(), "ICP with Normals converged: %s score: %f epsilon: %f", 
                    reg.hasConverged() ? "true" : "false", reg.getFitnessScore(), reg.getTransformationEpsilon());
        
        if(reg.hasConverged() ) current_guess_ = reg.getFinalTransformation();


      #endif
    }
    
  //convert transformation to ros usable form
    Eigen::Matrix4f mf = (current_guess_);
    Eigen::Matrix4d md(mf.cast<double>());
    Eigen::Affine3d affine(md);
    geometry_msgs::msg::TransformStamped transformStamped = tf2::eigenToTransform(affine);
    output_->transform = transformStamped.transform;
    output_->header = pcl_conversions::fromPCL(cloud1->header);
    output_->header.frame_id = parent_frame;
    output_->child_frame_id = child_frame;

    //Add new transform to accumulator and compute average
    if(buffer_size_ > 1)
    {
      x_array_(output_->transform.translation.x); double x_mean = rolling_mean(x_array_);
      y_array_(output_->transform.translation.y); double y_mean = rolling_mean(y_array_);
      z_array_(output_->transform.translation.z); double z_mean = rolling_mean(z_array_);
      geometry_msgs::msg::Quaternion q_mean = Cloud_Alignment::AverageQuaternion(output_->transform.rotation);

      //Update transformations with average
      output_->transform.translation.x = x_mean;
      output_->transform.translation.y = y_mean;
      output_->transform.translation.z = z_mean;
      output_->transform.rotation = q_mean;

      Eigen::Affine3d eigenTransform = tf2::transformToEigen(output_->transform);
      current_guess_ = eigenTransform.matrix().cast<float>();
    }

  // Transforming filtered or unfiltered, input cloud using found transform.
     if(is_output_filtered_)
    {
      pcl::copyPointCloud (*filtered_cloud0, *output_cloud0);
      pcl::copyPointCloud (*filtered_cloud1, *output_cloud1);
    }
    else
    {
      pcl::copyPointCloud (*cloud0, *output_cloud0);
      pcl::copyPointCloud (*cloud1, *output_cloud1);
    }
    
    // Calculate diff from last_transform
    tf2::Transform old_trans, new_trans;
    tf2::convert(last_transform_, old_trans);
    tf2::convert(output_->transform, new_trans);
    tf2::Transform diff = old_trans.inverseTimes(new_trans);
    
    geometry_msgs::msg::TransformStamped gm_diff;
    tf2::convert(diff, gm_diff.transform);
    gm_diff.header = pcl_conversions::fromPCL(cloud1->header);
    gm_diff.header.frame_id = child_frame;
    gm_diff.child_frame_id = child_frame;

    tf2::Vector3 diff_vector(diff.getOrigin());
    tf2::Matrix3x3 diff_matrix(diff.getRotation());
    double diff_x, diff_y, diff_z, diff_roll, diff_pitch, diff_yaw;
    diff_x = diff_vector.getX();
    diff_y = diff_vector.getY();
    diff_z = diff_vector.getZ();
    diff_matrix.getRPY(diff_roll, diff_pitch, diff_yaw);

    //writeout values
    if(freeze0_) 
    {
      RCLCPP_INFO(this->get_logger(), "input0 frozen");
    }
    if(freeze1_) 
    {
      RCLCPP_INFO(this->get_logger(), "input1 frozen");
    }

    RCLCPP_INFO(this->get_logger(), "X:     %f m, diff: %f m", output_->transform.translation.x, diff_x);
    RCLCPP_INFO(this->get_logger(), "Y:     %f m, diff: %f m", output_->transform.translation.y, diff_y);
    RCLCPP_INFO(this->get_logger(), "Z:     %f m, diff: %f m", output_->transform.translation.z, diff_z);

    tf2::Quaternion q(
          output_->transform.rotation.x,
          output_->transform.rotation.y,
          output_->transform.rotation.z,
          output_->transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);
    RCLCPP_INFO(this->get_logger(), "Roll:  %f rad, %f deg, diff: %f rad", roll, (roll/PI*180), diff_roll);
    RCLCPP_INFO(this->get_logger(), "pitch: %f rad, %f deg, diff: %f rad", pitch, (pitch/PI*180), diff_pitch);
    RCLCPP_INFO(this->get_logger(), "Yaw:   %f rad, %f deg, diff: %f rad", yaw, (yaw/PI*180), diff_yaw);

    // Create output msgs
    geometry_msgs::msg::TransformStamped::Ptr output(output_);
    sensor_msgs::msg::PointCloud2::Ptr output_msg0(new sensor_msgs::msg::PointCloud2);
    sensor_msgs::msg::PointCloud2::Ptr output_msg1(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*output_cloud0, *output_msg0);
    pcl::toROSMsg(*output_cloud1, *output_msg1);

    //first convert output_msg1 to parent_frame then apply output transform
    // geometry_msgs::msg::TransformStamped pTransform = tfBuffer_.lookupTransform(parent_frame, output_msg1->header.frame_id, output_msg1->header.stamp);
    // tf2::doTransform(*output_msg1, *output_msg1, pTransform);
    // tf2::doTransform(*output_msg1, *output_msg1, *output_);

    // correct error in cloud1's output msg 
    tf2::doTransform(*output_msg1, *output_msg1, gm_diff);

    output_trans_pub_->publish(*output);
    output_cloud0_pub_->publish(*output_msg0);
    output_cloud1_pub_->publish(*output_msg1);
  }

  void Cloud_Alignment::input0_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    
    if(freeze0_) 
    {
      return;
    }
    
    sensor_msgs::msg::PointCloud2 cloud_in(*msg), cloud_out;
    geometry_msgs::msg::TransformStamped transform;

    if(parent_frame_id_ != "")
    {
      try{
        transform = tfBuffer_->lookupTransform(parent_frame_id_, msg->header.frame_id, msg->header.stamp);
        tf2::doTransform(*msg, cloud_out, transform);

        cloud0_ = std::make_shared<sensor_msgs::msg::PointCloud2>(cloud_out);
        }
      catch (tf2::TransformException &ex){
        RCLCPP_WARN(this->get_logger(), "%s",ex.what());
        return;
      }
    }
    else
    {
      cloud0_ = msg;
    }

  }

  void Cloud_Alignment::input1_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if(freeze1_) 
    {
      return;
    }

    sensor_msgs::msg::PointCloud2 cloud_in(*msg), cloud_out;
    geometry_msgs::msg::TransformStamped transform;

     if(child_frame_id_ != "")
    {
      try{
        transform = tfBuffer_->lookupTransform(child_frame_id_, msg->header.frame_id, msg->header.stamp);
        
        tf2::doTransform(*msg, cloud_out, transform);

        cloud1_ = std::make_shared<sensor_msgs::msg::PointCloud2>(cloud_out);
      }
      catch (tf2::TransformException &ex){
        RCLCPP_WARN(this->get_logger(), "%s",ex.what());
        return;
      }
    }
    else
    {
      cloud1_ = msg;
    }

  }

  void Cloud_Alignment::freeze0_callback(const std_srvs::srv::Empty::Request::SharedPtr req,
            std_srvs::srv::Empty::Response::SharedPtr resp)
  {
    freeze0_ = true;
  }

  void Cloud_Alignment::freeze1_callback(const std_srvs::srv::Empty::Request::SharedPtr req,
            std_srvs::srv::Empty::Response::SharedPtr resp)
  {
    freeze1_ = true;	  
  }

  void Cloud_Alignment::unfreeze0_callback(const std_srvs::srv::Empty::Request::SharedPtr req,
            std_srvs::srv::Empty::Response::SharedPtr resp)
  {
    RCLCPP_INFO(this->get_logger(), "input0 unfrozen");
    freeze0_ = false;	  
  }

  void Cloud_Alignment::unfreeze1_callback(const std_srvs::srv::Empty::Request::SharedPtr req,
            std_srvs::srv::Empty::Response::SharedPtr resp)
  {
    RCLCPP_INFO(this->get_logger(), "input1 unfrozen");
    freeze1_ = false;	  
  }

  bool Cloud_Alignment::reset()
  {
    // Reinitialize icp_config_ from parameters
    icp_config_.method_ = this->get_parameter("method").as_int();
    icp_config_.norm_kSearch_ = this->get_parameter("norm/KSearchs").as_int();
    icp_config_.norm_RadiusSearch_ = this->get_parameter("norm/RadiusSearch").as_double();
    icp_config_.epsilon_ = this->get_parameter("epsilon").as_double();
    icp_config_.maxIterations_ = this->get_parameter("maxIterations").as_int();
    icp_config_.maxCorrespondenceDistance_ = this->get_parameter("maxCoorespondenceDistance").as_double();
    icp_config_.ndt_StepSize_ = this->get_parameter("ndt/StepSize").as_double();
    icp_config_.ndt_Resolution_ = this->get_parameter("ndt/Resolution").as_double();
    icp_config_.voxelSize_ = this->get_parameter("voxelSize").as_double();
    icp_config_.filter_i_min_ = this->get_parameter("filter/i_min").as_double();
    icp_config_.filter_i_max_ = this->get_parameter("filter/i_max").as_double();
    icp_config_.filter_x_min_ = this->get_parameter("filter/x_min").as_double();
    icp_config_.filter_x_max_ = this->get_parameter("filter/x_max").as_double();
    icp_config_.filter_y_min_ = this->get_parameter("filter/y_min").as_double();
    icp_config_.filter_y_max_ = this->get_parameter("filter/y_max").as_double();
    icp_config_.filter_z_min_ = this->get_parameter("filter/z_min").as_double();
    icp_config_.filter_z_max_ = this->get_parameter("filter/z_max").as_double();
    
    RCLCPP_INFO(this->get_logger(), "Configuration reset to parameter values");
    return true;
  }
  
  bool Cloud_Alignment::revert()
  {
    //revert Guess
    try {
      geometry_msgs::msg::TransformStamped transform = tfBuffer_->lookupTransform(parent_frame_id_, child_frame_id_, tf2::TimePointZero);
      last_transform_ = transform.transform;
      
      Eigen::Affine3d eigenTransform = tf2::transformToEigen(transform);
      current_guess_ = eigenTransform.matrix().cast<float>();
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not lookup transform: %s", ex.what());
      // Set to identity if transform not available
      current_guess_ = Eigen::Matrix4f::Identity();
    }
    
    //revert rolling window accumulators
    x_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    y_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    z_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    qx_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    qy_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    qz_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    qw_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    current_qx_ = 0; current_qy_ = 0; current_qz_ = 0; current_qw_ = 0; 
    
    RCLCPP_INFO(this->get_logger(), "Guess transform reverted");

    return true;
  }

  void Cloud_Alignment::revert_callback(const std_srvs::srv::Empty::Request::SharedPtr req,
            std_srvs::srv::Empty::Response::SharedPtr resp)
  {
    Cloud_Alignment::revert();
  }

  void Cloud_Alignment::reset_callback(const std_srvs::srv::Empty::Request::SharedPtr req,
            std_srvs::srv::Empty::Response::SharedPtr resp)
  {
    Cloud_Alignment::reset();
  }

  void Cloud_Alignment::pushtransform_callback(const std_srvs::srv::Empty::Request::SharedPtr req,
            std_srvs::srv::Empty::Response::SharedPtr resp)
  {
    Cloud_Alignment::pushTransform();
  }
  
  void Cloud_Alignment::pushYaw_callback(const std_srvs::srv::Empty::Request::SharedPtr req,
            std_srvs::srv::Empty::Response::SharedPtr resp)
  {
    Cloud_Alignment::pushYaw();
  }
  
  void Cloud_Alignment::pushRollPitchCorrection_callback(const std_srvs::srv::Empty::Request::SharedPtr req,
            std_srvs::srv::Empty::Response::SharedPtr resp)
  {

    Cloud_Alignment::pushRollPitchCorrection();
  }
  

  void Cloud_Alignment::DownsampleCloud(const pcl::PointCloud<PointT>::Ptr in_cloud,
                                                pcl::PointCloud<PointT> &out_cloud,
                                                double in_leaf_size)

  {
    pcl::PointCloud<PointT>::Ptr filtered_ptr(new pcl::PointCloud<PointT>);
  
    // build the condition
    pcl::ConditionAnd<PointT>::Ptr range_cond (new pcl::ConditionAnd<PointT> ());
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("intensity", pcl::ComparisonOps::GE, icp_config_.filter_i_min_)));
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("intensity", pcl::ComparisonOps::LE, icp_config_.filter_i_max_)));
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("x",         pcl::ComparisonOps::GE, icp_config_.filter_x_min_)));
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("x",         pcl::ComparisonOps::LE, icp_config_.filter_x_max_)));
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("y",         pcl::ComparisonOps::GE, icp_config_.filter_y_min_)));
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("y",         pcl::ComparisonOps::LE, icp_config_.filter_y_max_)));
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("z",         pcl::ComparisonOps::GE, icp_config_.filter_z_min_)));
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("z",         pcl::ComparisonOps::LE, icp_config_.filter_z_max_)));
    // build the filter
    pcl::ConditionalRemoval<PointT> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (in_cloud);
    condrem.setKeepOrganized(false);
    // apply filter
    condrem.filter (*filtered_ptr);

    if(in_leaf_size > 0.001)
    {
      pcl::VoxelGrid<PointT> voxelized;
      voxelized.setInputCloud(filtered_ptr);
      voxelized.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
      voxelized.filter(out_cloud);
    }
    else
    {
      pcl::copyPointCloud(*filtered_ptr, out_cloud);
    }
  }

  geometry_msgs::msg::Quaternion Cloud_Alignment::AverageQuaternion(const geometry_msgs::msg::Quaternion& newRotation)
  {
    //RCLCPP_INFO(this->get_logger(), "%f %f %f %f", current_qx_, current_qy_, current_qz_, current_qw_);
    tf2::Quaternion lastRotation(current_qx_, current_qy_, current_qz_, current_qw_);
    tf2::Quaternion currRotation; 
    tf2::convert(newRotation, currRotation);
    //On first pass lastRotation will be zero length
    if(abs(lastRotation.length()) < 0.1)
    {
      RCLCPP_INFO(this->get_logger(), "AverageQuaternion initialized");

      //Add new values to accumulators
      qx_array_(currRotation.x()); current_qx_ = currRotation.x();
      qy_array_(currRotation.y()); current_qy_ = currRotation.y();
      qz_array_(currRotation.z()); current_qz_ = currRotation.z();
      qw_array_(currRotation.w()); current_qw_ = currRotation.w();

      return newRotation;
    }

    //Before we add the new rotation to the average (mean), we have to check whether the quaternion has to be inverted. Because
    //q and -q are the same rotation, but cannot be averaged, we have to make sure they are all the same.
    // if(AreQuaternionsClose(currRotation, lastRotation))
    // {
    //     RCLCPP_INFO(this->get_logger(), "flip quaternion");
    //     RCLCPP_INFO(this->get_logger(), "%f %f %f %f", currRotation.x(), currRotation.y(), currRotation.z(), currRotation.w());
    //     RCLCPP_INFO(this->get_logger(), "%f %f %f %f", lastRotation.x(), lastRotation.y(), lastRotation.z(), lastRotation.w());
    //     currRotation = tf2::Quaternion(-currRotation.x(), -currRotation.y(), -currRotation.z(), -currRotation.w());
    // }
    current_qx_ = currRotation.x();
    current_qy_ = currRotation.y();
    current_qz_ = currRotation.z();
    current_qw_ = currRotation.w();
    
    //Add new values to accumulators
    qx_array_(currRotation.x());
    qy_array_(currRotation.y());
    qz_array_(currRotation.z());
    qw_array_(currRotation.w());
    float w = rolling_mean(qw_array_);
    float x = rolling_mean(qx_array_);
    float y = rolling_mean(qy_array_);
    float z = rolling_mean(qz_array_);

    //Convert back to quaternion
    tf2::Quaternion mean(x, y, z, w);

    geometry_msgs::msg::Quaternion result;
    tf2::convert(mean.normalize(), result);

    //note: if speed is an issue, you can skip the normalization step
    return result;
}

//Returns true if the two input quaternions are close to each other. This can
//be used to check whether or not one of two quaternions which are supposed to
//be very similar but has its component signs reversed (q has the same rotation as
//-q)
bool Cloud_Alignment::AreQuaternionsClose(tf2::Quaternion q1, tf2::Quaternion q2)
{

    float dot = q1.dot(q2);
    
    if(dot < 0.0f)
    {

        return false;                   
    }

    else
    {

        return true;
    }
}

  
}  // namespace Multi_Sensor_Alignment





