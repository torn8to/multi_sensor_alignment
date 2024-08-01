
#include "icp_align_tool/icp_align_tool.hpp"
/*
  * TODO port parameters over from ros2
  *         [ ] onInit
  *         [ ] onInit 
  * TODO remove dynamic reconfigure services
  * TODO add alignment publisher service
  * TODO replace all logging statemets with rclcpp logging
  *
*/
namespace Multi_Sensor_Alignment
{
  Cloud_Alignment::Cloud_Alignment(int buffer_size):node// Initialization list
  freeze0_(0),
  freeze1_(0),
  current_guess_(Eigen::Matrix4f::Identity()),
  output_(new geometry_msgs::TransformStamped),
  buffer_size_(buffer_size),
  x_array_(tag::rolling_window::window_size = buffer_size),
  y_array_(tag::rolling_window::window_size = buffer_size),
  z_array_(tag::rolling_window::window_size = buffer_size),
  qw_array_(tag::rolling_window::window_size = buffer_size),
  qx_array_(tag::rolling_window::window_size = buffer_size),
  qy_array_(tag::rolling_window::window_size = buffer_size),
  qz_array_(tag::rolling_window::window_size = buffer_size),
  current_qw_(0), current_qx_(0), current_qy_(0), current_qz_(0),
  tfListener_(tfBuffer_),
  wait_for_tf_delay_(0.1),
  received_alignPubConfig_(false),
  received_alignPubDesc_(false),
  received_alignToolConfig_(false)
  {
    this->onInit();
  }

  Cloud_Alignment::~Cloud_Alignment()
  {
    // pass
  }

  void Cloud_Alignment::onInit()
  {
  //Subscribe to Dynamic Reconfigure on the alignment publisher node, AlignPubConfig
    //ROS_INFO_STREAM_NAMED(node_name, "alignment_server set to " << align_server_name_);   
    RCL_CPP_INFO_STREAM(get_logger(), "alignment_server set to " << align_server_name_);   
    //alignClient_.reset(new dynamic_reconfigure::Client<multi_sensor_alignment::alignment_publisherConfig>(align_server_name_));
    //alignClient_->setConfigurationCallback(boost::bind(&Cloud_Alignment::align_pubconfig_callback, this, _1));
    //alignClient_->setDescriptionCallback(boost::bind(&Cloud_Alignment::align_pubdesc_callback, this, _1));
    //TODO get parameters 
    //Wait 60 seconds for the alignment publisher nodes dynamic param server to respond
    /*int count = 0, maxcount = 60;
    while((count < maxcount && (!received_alignPubConfig_ || !received_alignPubDesc_)) && align_server_name_ != "")
    {
      ros::Duration(1.0).sleep();
      //ROS_INFO_STREAM_NAMED(get_logger(), "Waiting on dynamic parameters from align_publisher. " << (maxcount-count) << " sec before giving up.");
      RCL_CPP_INFO_STREAM(get_logger(), "Waiting on dynamic parameters from align_publisher. " << (maxcount-count) << " sec before giving up.");
      ros::spinOnce();
      coun
    }
  */

  // ROS Parameters
    this->declare_parameter("parent_frame","");
    parent_frame_id_ = this->get_parameter(parent_frame).as_string();
    //pnh_.param<std::string>("parent_frame", parent_frame_id_, "");
      RCL_CPP_INFO_STREAM(get_logger(), "parent_frame set to " << parent_frame_id_);
    this->declare_parmeter("child_frame");
    child_frame_id_ = this->get_parameter("child_frame","").as_string();
    //pnh_.param<std::string>("child_frame", child_frame_id_, "");
      RCL_CPP_INFO_STREAM(get_logger(), "child_frame set to " << child_frame_id_);
      this-> declare_parameter("output_frequency", 0.1)
      output_frequency_ = this->get_parameter("child_frame").as_string();
      RCL_CPP_INFO_STREAM(get_logger(), "output_frequency set to " << output_frequency_);     
          

    if(!received_alignPubConfig_){
      RCL_CPP_INFO_STREAMN(get_logger(), "Proceeding without alignment publisher.");
    }

      RCL_CPP_INFO_STREAM(get_logger(), "x set to " << alignPubConfig_.x);
      RCL_CPP_INFO_STREAM(get_logger(), "y set to " << alignPubConfig_.y);
      RCL_CPP_INFO_STREAM(get_logger(), "z set to " << alignPubConfig_.z);
      RCL_CPP_INFO_STREAM(get_logger(), "roll set to " << alignPubConfig_.roll);
      RCL_CPP_INFO_STREAM(get_logger(), "pitch set to " << alignPubConfig_.pitch);
      RCL_CPP_INFO_STREAM(get_logger(), "yaw set to " << alignPubConfig_.yaw);
    

    this->declare_parameter("input_cloud0","input0");
    input_cloud0_topic_ = this->get_parameter("input_cloud0").as_string();
    //pnh_.param<std::string>("input_cloud0", input0_topic_, "input0");
    RCL_CPP_INFO_STREAM(get_logger(), "input_cloud0 topic set to " << input0_topic_);
    this->declare_parameter("input_cloud1","input1");
    input_cloud1_topic_ = this->get_parameter("input_cloud1").as_string();
    //pnh_.param<std::string>("input_cloud0", input0_topic_, "input0");
    //pnh_.param<std::string>("input_cloud1", input1_topic_, "input1");
    RCL_CPP_INFO_STREAM(get_logger(),"input_cloud1 topic set to " << input1_topic_);
    this->declare_parameter("output_cloud0","output0");
    output_cloud0_topic_ = this->get_parameter("output_cloud0").as_string();
    //pnh_.param<std::string>("output_cloud0", output_cloud0_topic_, "cloud0");
    RCL_CPP_INFO_STREAM(get_logger(), "output_cloud0 topic set to " << output_cloud0_topic_);
    this->declare_parameter("output_cloud1","cloud1");
    output_cloud1_topic_ = this->get_parameter("output_cloud1").as_string();
    RCL_CPP_INFO_STREAM(get_logger(), "output_cloud1 topic set to " << output_cloud1_topic_);
    this->declare_parameter("is_output_filtered",true);
    output_ = this->get_parameter("is_output_filtered").as_bool();
    //pnh_.param<std::string>("output", output_trans_topic_, "output");
    RCL_CPP_INFO_STREAM(get_logger(), "output topic set to " << output_trans_topic_);
    this->declare_parameter("output","output");
    output_cloud1_topic_ = this->get_parameter("output").as_string();
   //pnh_.param("is_output_filtered", is_output_filtered_, false);
    RCL_CPP_INFO_STREAM(get_logger(), "is_output_filtered set to " << is_output_filtered_);
    //TODO alter these to fit types  
    this->declare_parameter("voxelSize",0.05);
    voxelSize =  this->get_parameter("voxelSize").as_double();
    RCL_CPP_INFO_STREAM(get_logger(), "voxelSize set to " << voxelSize);
    this->declare_parameter("filter/i_min",0.05);
    filter_i_min_ =  this->get_parameter("filter/i_min").as_double();
    //pnh_.param("filter/i_min", alignToolConfig_.i_min, alignToolConfig_.i_min);
    RCL_CPP_INFO_STREAM(get_logger(), "filter min i set to " << fliter_i_min_);   
    this->declare_parameter("filter/i_max",0.0);
    filter_i_max_ =  this->get_parameter("filter/i_max").as_double();
    //pnh_.param("filter/i_max", alignToolConfig_.i_max, alignToolConfig_.i_max);
    RCL_CPP_INFO_STREAM(get_logger(), "filter max i set to " << alignToolConfig_.i_max);  
    this->declare_parameter("filter/x_min",0.0);
    filter_x_min_ =  this->get_parameter("filter/x_min").as_double();
    //pnh_.param("filter/x_min", alignToolConfig_.x_min, alignToolConfig_.x_min);
    RCL_CPP_INFO_STREAM(get_logger(), "filter min x set to " << filter_x_min_);   
    this->declare_parameter("filter/x_max",1000.0);
    filter_x_max_ =  this->get_parameter("filter/x_max").as_double();
    //pnh_.param("filter/x_max", alignToolConfig_.x_max, alignToolConfig_.x_max);
    RCL_CPP_INFO_STREAM(get_logger(), "filter max x set to " << filter_x_max_);   
    //pnh_.param("filter/y_min", alignToolConfig_.y_min, alignToolConfig_.y_min);
    this->declare_parameter("filter/y_min",1000.0);
    filter_x_max_ =  this->get_parameter("filter/y_min").as_double();
    RCL_CPP_INFO_STREAM(get_logger(), "filter min y set to " << filter_y_min_);   
    //pnh_.param("filter/y_max", alignToolConfig_.y_max, alignToolConfig_.y_max);
    this->declare_parameter("filter/y_max",1000.0);
    filter_x_max_ =  this->get_parameter("filter/y_max").as_double();
    RCL_CPP_INFO_STREAM(get_logger(), "filter max y set to " << filter_y_max_);   
    //pnh_.param("filter/z_min", alignToolConfig_.z_min, alignToolConfig_.z_min);
    this->declare_parameter("filter/z_min",1000.0);
    filter_z_min_ =  this->get_parameter("filter/z_min").as_double();
    RCL_CPP_INFO_STREAM(get_logger(), "filter min z set to " << alignToolConfig_.z_min);   
    //pnh_.param("filter/z_max", alignToolConfig_.z_max, alignToolConfig_.z_max);
    this->declare_parameter("filter/z_max",1000.0);
    filter_z_max_ =  this->get_parameter("filter/z_max").as_double();
    RCL_CPP_INFO_STREAM(get_logger(), "filter max z set to " << alignToolConfig_.z_max);
    this->declare_parameter("method",1)
    method_ =  this->get_parameter("method").as_int();
    //pnh_.param("method", alignToolConfig_.Method, 1);
      RCL_CPP_INFO_STREAM(get_logger(), "method set to " << method_);
    this->declare_parameter("epsilon",1)
    method_ =  this->get_parameter("epsilon").as_int();
    //pnh_.param("epsilon", alignToolConfig_.Epsilon, alignToolConfig_.Epsilon);
    this->declare_parameter("maxIterations",100)
    method_ =  this->get_parameter("maxIterations").as_int();
    //pnh_.param("maxIterations", alignToolConfig_.MaxIterations, alignToolConfig_.MaxIterations);
      RCL_CPP_INFO_STREAM(get_logger(), "maxIterations set to " << alignToolConfig_.MaxIterations);
    this->declare_parameter("maxCorrespondenceDistance",0.1);
    maxCorrespondenceDistance_ = this->get_parameter("maxCorrespondenceDistance").as_double();
    //pnh_.param("maxCorrespondenceDistance", alignToolConfig_.MaxCorrespondenceDistance, alignToolConfig_.MaxCorrespondenceDistance);
      RCL_CPP_INFO_STREAM(get_logger(), "maxCorrespondenceDistance set to " << alignToolConfig_.MaxCorrespondenceDistance);
    this->declare_parameter("norm/KSearch",30);
    norm_kSearch_ = this->get_parameter("norm/KSearch").as_int();
   // pnh_.param("norm/KSearch", alignToolConfig_.KSearch, alignToolConfig_.KSearch);
    RCL_CPP_INFO_STREAM(get_logger(), "KSearch set to " << norm_kSearch_);
    this->declare_parameter("norm/RadiusSearch",30);
    norm_RadiusSearch_ = this->get_parameter("norm/RadiusSearch").as_int();
    //pnh_.param("norm/RadiusSearch", alignToolConfig_.RadiusSearch, alignToolConfig_.RadiusSearch);
    RCL_CPP_INFO_STREAM(get_logger(), "RadiusSearch set to " << norm_RadiusSearch_);      
    this->declare_parameter("ndt/StepSize",0.01);
    ndt_StepSize_ = this->get_parameter("ndt/StepSize").as_double(). 
    //pnh_.param("ndt/StepSize", alignToolConfig_.StepSize, alignToolConfig_.StepSize);
    RCL_CPP_INFO_STREAM(get_logger(), "StepSize set to " << ndt_StepSize);

    this->declare_parameter("ndt/Resolution",1.0);
    ndt_Resoultion_ = this->get_parameter("ndt/Resolution").as_double(). 
    //pnh_.param("ndt/Resolution", alignToolConfig_.Resolution, alignToolConfig_.Resolution);
    RCL_CPP_INFO_STREAM(get_logger(), "Resolution set to " << ndt_Resolution_);   
    

  // ROS publishers
  // TODO fill in data on publishers
    output_trans_pub_ = this->createPublisher<geometry_msgs::msg::TransformStamped>(output_trans_topic_,10);
    output_cloud0_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud0_topic_,10);
    output_cloud1_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud1_topic_,10);
    pub_timer_ = nh_.create_wall_timer(1, boost::bind(& publish_callback, this));
    /*
    output_trans_pub_  = nh_.advertise<geometry_msgs::TransformStamped>(output_trans_topic_,100);
    output_cloud0_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_cloud0_topic_,10);
    output_cloud1_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_cloud1_topic_,10);
    pub_timer_ = nh_.createTimer(ros::Duration(1.0/output_frequency_), boost::bind(& Cloud_Alignment::publish_callback, this, _1));
    */
  // ROS subscribers
  input_sub0_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(input0_topic_,&Cloud_Alignment::input0_callback);
  input_sub1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(input1_topic_,&cloud_Alignment::input1_callback);

    /*
    input_sub0_ = nh_.subscribe(input0_topic_, 100, &Cloud_Alignment::input0_callback, this);
    input_sub1_ = nh_.subscribe(input1_topic_, 100, &Cloud_Alignment::input1_callback, this);
    */
  // ROS Service
    /*
    service0_ = pnh_.advertiseService("freeze_cloud0", &Cloud_Alignment::freeze0_callback, this);
    service1_ = pnh_.advertiseService("freeze_cloud1", &Cloud_Alignment::freeze1_callback, this);
    service2_ = pnh_.advertiseService("unfreeze_cloud0", &Cloud_Alignment::unfreeze0_callback, this);
    service3_ = pnh_.advertiseService("unfreeze_cloud1", &Cloud_Alignment::unfreeze1_callback, this);
    */
    this->createService("freeze_cloud0",&Cloud_Alignment::freeze0_callback);
    this->createService("freeze_cloud1",&Cloud_Alignment::freeze1_callback);
    this->createService("unfreeze_cloud0",&Cloud_Alignment::freeze0_callback);
    this->createService("unfreeze_cloud1",&Cloud_Alignment::freeze1_callback);

    if(received_alignPubConfig_)
    {
      service4_ = pnh_.advertiseService("revert", &Cloud_Alignment::revert_callback, this);
      service5_ = pnh_.advertiseService("reset", &Cloud_Alignment::reset_callback, this);
      service6_ = pnh_.advertiseService("push_transform", &Cloud_Alignment::pushtransform_callback, this);
      service7_ = pnh_.advertiseService("push_yaw", &Cloud_Alignment::pushYaw_callback, this);
      service8_ = pnh_.advertiseService("push_roll_pitch_correction", &Cloud_Alignment::pushRollPitchCorrection_callback, this);
    }

  // Reset the guess transform for good measure
    Cloud_Alignment::revert();
  }


  void Cloud_Alignment::align_pubconfig_callback(const multi_sensor_alignment::alignment_publisherConfig& config) 
  {
    RCL_CPP_INFO("Received configuration from alignment publisher");
    RCL_CPP_INFO("%f %f %f %f %f %f", 
            config.x, config.y, config.z, config.roll, config.pitch, config.yaw);

    if (!received_alignPubConfig_) 
      initialAlignPubConfig_ = config;

    alignPubConfig_ = config;
    received_alignPubConfig_ = true;

    Cloud_Alignment::revert();
  }
/*
TODO change this to a
*/
  void Cloud_Alignment::align_pubdesc_callback(const dynamic_reconfigure::ConfigDescription& description) 
  {
    RCL_CPP_INFO("Received description from alignment publisher");

    alignPubDesc_ = description;
    received_alignPubDesc_ = true;
  }

  bool Cloud_Alignment::pushTransform()
  {
    if(!received_alignPubConfig_)
    {
      // ROS_WARN_STREAM_NAMED(node_name, "Alignment Server isn't connected.");
      RCL_CPP_INFO_STREAM(node_name, "Alignment Server isn't connected.");
      return true;
    }

    alignPubConfig_.x = output_->transform.translation.x;
    alignPubConfig_.y = output_->transform.translation.y;
    alignPubConfig_.z = output_->transform.translation.z;

    tf2::Quaternion q;
    convert(output_->transform.rotation, q);
    tf2::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);

    alignPubConfig_.roll = roll;
    alignPubConfig_.pitch = pitch;
    alignPubConfig_.yaw = yaw;

    return alignClient_->setConfiguration(alignPubConfig_);
  }
  
  bool Cloud_Alignment::pushYaw()
  {
    if(!received_alignPubConfig_)
    {
      // ROS_WARN_STREAM_NAMED(node_name, "Alignment Server isn't connected.");
      RCL_CPP_INFO_STREAM(node_name, "Alignment Server isn't connected.");
      return true;
    }

    tf2::Quaternion q;
    convert(output_->transform.rotation, q);
    tf2::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);

    alignPubConfig_.yaw = yaw;

    return alignClient_->setConfiguration(alignPubConfig_);
  }
  
  bool Cloud_Alignment::pushRollPitchCorrection()
  {
    if(!received_alignPubConfig_)
    {
      // ROS_WARN_STREAM_NAMED(node_name, "Alignment Server isn't connected.");
      ROS_INFO_STREAM_NAMkD(node_name, "Alignment Server isn't connected.");
      return true;
    }

    tf2::Quaternion q;
    convert(output_->transform.rotation, q);
    tf2::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);

    alignPubConfig_.roll -= (roll / 2);
    alignPubConfig_.pitch -= (pitch / 2);

    return alignClient_->setConfiguration(alignPubConfig_);
  }
  
  void Cloud_Alignment::publish_callback(const ros::TimerEvent& event)
  {
    ROS_INFO_STREAM_NAMED(node_name, "---");

    if(cloud0_.data.size() <= 0 || cloud1_.data.size() <=0) return;

  // Convert from ROS msg to pointcloud2 object
    pcl::PointCloud<PointT>::Ptr cloud0(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud1(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(cloud0_, *cloud0);
    pcl::fromROSMsg(cloud1_, *cloud1);

    std::string parent_frame = cloud0_.header.frame_id;
    std::string child_frame  = cloud1_.header.frame_id;

  //Downsample
    pcl::PointCloud<PointT>::Ptr filtered_cloud0(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr filtered_cloud1(new pcl::PointCloud<PointT>);
    DownsampleCloud(cloud0, *filtered_cloud0, alignToolConfig_.VoxelSize);
    DownsampleCloud(cloud1, *filtered_cloud1, alignToolConfig_.VoxelSize);
    
    ROS_INFO_STREAM_NAMED(node_name, "\n");
    
  //Perform Registration
    Eigen::Matrix4f prev;
    pcl::PointCloud<PointT>::Ptr output_cloud0(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr output_cloud1(new pcl::PointCloud<PointT>);

    // ICP Nonlinear with scaling CorDist
    if(alignToolConfig_.Method == 0)
    {
      // Compute surface normals and curvature
      PointCloudWithNormals::Ptr points_with_normals0 (new PointCloudWithNormals);
      PointCloudWithNormals::Ptr points_with_normals1 (new PointCloudWithNormals);

      pcl::NormalEstimation<PointT, PointNormalT> norm_est;
      pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
      norm_est.setSearchMethod (tree);
      if(alignToolConfig_.KSearch > 0) norm_est.setKSearch(alignToolConfig_.KSearch);
      else norm_est.setRadiusSearch(alignToolConfig_.RadiusSearch);
      
      norm_est.setInputCloud (filtered_cloud0);
      norm_est.compute (*points_with_normals0);
      pcl::copyPointCloud (*filtered_cloud0, *points_with_normals0);

      norm_est.setInputCloud (filtered_cloud1);
      norm_est.compute (*points_with_normals1);
      pcl::copyPointCloud (*filtered_cloud1, *points_with_normals1);

      // Align
      pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
      reg.setTransformationEpsilon (alignToolConfig_.Epsilon);
      // Set the maximum distance between two correspondences (cloud0<->cloud1) to user input
      // Note: adjust this based on the size of your datasets
      reg.setMaxCorrespondenceDistance (alignToolConfig_.MaxCorrespondenceDistance);  
      // Set the first pointcloud as the target
      reg.setInputTarget (points_with_normals0);

      // Run the optimization in a loop and visualize the results
      PointCloudWithNormals::Ptr reg_result = points_with_normals1;
      reg.setMaximumIterations (2);
      for (int i = 0; i < alignToolConfig_.MaxIterations; ++i)
      {
        ROS_DEBUG_STREAM_NAMED(node_name,"Iteration Nr. " << i << " maxCorrespondenceDistance=" << reg.getMaxCorrespondenceDistance () << ".\n");

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

      ROS_INFO_STREAM_NAMED(node_name, "ICP Nonlinear Transform converged:" << reg.hasConverged ()
              << " score: " << reg.getFitnessScore () << " epsilon:" << reg.getTransformationEpsilon());

    }
    // ICP Nonlinear
    else if(alignToolConfig_.Method == 1)
    {
      // Compute surface normals and curvature
      PointCloudWithNormals::Ptr points_with_normals0 (new PointCloudWithNormals);
      PointCloudWithNormals::Ptr points_with_normals1 (new PointCloudWithNormals);

      pcl::NormalEstimation<PointT, PointNormalT> norm_est;
      pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
      norm_est.setSearchMethod (tree);
      if(alignToolConfig_.KSearch > 0) norm_est.setKSearch(alignToolConfig_.KSearch);
      else norm_est.setRadiusSearch(alignToolConfig_.RadiusSearch);
      
      norm_est.setInputCloud (filtered_cloud0);
      norm_est.compute (*points_with_normals0);
      pcl::copyPointCloud (*filtered_cloud0, *points_with_normals0);

      norm_est.setInputCloud (filtered_cloud1);
      norm_est.compute (*points_with_normals1);
      pcl::copyPointCloud (*filtered_cloud1, *points_with_normals1);

      // Align
      pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
      reg.setTransformationEpsilon (alignToolConfig_.Epsilon);
      // Set the maximum distance between two correspondences (cloud0<->cloud1) to user input
      // Note: adjust this based on the size of your datasets
      reg.setMaxCorrespondenceDistance (alignToolConfig_.MaxCorrespondenceDistance);  
      reg.setMaximumIterations(alignToolConfig_.MaxIterations);
      // Set the first pointcloud as the target
      reg.setInputTarget (points_with_normals0);
      reg.setInputSource (points_with_normals1);

      PointCloudWithNormals::Ptr reg_result = points_with_normals1;
      
      //Get Results
      reg.align (*reg_result, current_guess_);

      ROS_INFO_STREAM_NAMED(node_name, "ICP Nonlinear Transform converged:" << reg.hasConverged ()
              << " score: " << reg.getFitnessScore () << " epsilon:" << reg.getTransformationEpsilon());
      
      if(reg.hasConverged() ) current_guess_ = reg.getFinalTransformation();

    }
    // Normal Distributions Transform
    else if(alignToolConfig_.Method == 2)
    {
      // Initializing Normal Distributions Transform (NDT).
      pcl::NormalDistributionsTransform<PointT, PointT> ndt;

      ndt.setTransformationEpsilon(alignToolConfig_.Epsilon);
      ndt.setStepSize(alignToolConfig_.StepSize);
      ndt.setResolution(alignToolConfig_.Resolution);
      ndt.setMaximumIterations(alignToolConfig_.MaxIterations);

      // Set the first pointcloud as the target
      ndt.setInputTarget(cloud0);
      ndt.setInputSource(filtered_cloud1);

      //Get Results
      ndt.align(*output_cloud1, current_guess_);
      
      ROS_INFO_STREAM_NAMED(node_name, "Normal Distributions Transform converged:" << ndt.hasConverged ()
                << " score: " << ndt.getFitnessScore () << " prob:" << ndt.getTransformationProbability());

      if(ndt.hasConverged() ) 
      {
        current_guess_ = ndt.getFinalTransformation();
      
        Eigen::Matrix3f rotation_matrix = current_guess_.block(0,0,3,3);
        Eigen::Vector3f translation_vector = current_guess_.block(0,3,3,1);
        ROS_INFO_STREAM_NAMED(node_name, "This transformation can be replicated using:");
        ROS_INFO_STREAM_NAMED(node_name, "rosrun tf static_transform_publisher " << translation_vector.transpose()
                << " " << rotation_matrix.eulerAngles(2,1,0).transpose() << " /" << parent_frame
                << " /" << child_frame << " 10");
      }

    }
    // ICP with Normals
    else if(alignToolConfig_.Method == 3)
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
        
        if(alignToolConfig_.KSearch > 0) norm_est.setKSearch (alignToolConfig_.KSearch);
        else norm_est.setRadiusSearch(alignToolConfig_.RadiusSearch);
        
        norm_est.setInputCloud (filtered_cloud0);
        norm_est.compute (*points_with_normals0);
        pcl::copyPointCloud (*filtered_cloud0, *points_with_normals0);

        norm_est.setInputCloud (filtered_cloud1);
        norm_est.compute (*points_with_normals1);
        pcl::copyPointCloud (*filtered_cloud1, *points_with_normals1);

        // Align
        pcl::IterativeClosestPointWithNormals<PointNormalT, PointNormalT> reg;
        reg.setTransformationEpsilon (alignToolConfig_.Epsilon);
        // Set the maximum distance between two correspondences (cloud0<->cloud1) to user input
        // Note: adjust this based on the size of your datasets
        reg.setMaxCorrespondenceDistance (alignToolConfig_.MaxCorrespondenceDistance);  
        reg.setMaximumIterations(alignToolConfig_.MaxIterations);
        reg.setUseSymmetricObjective(true);
        reg.setEnforceSameDirectionNormals(true);
        // Set the first pointcloud as the target
        reg.setInputTarget (points_with_normals0);
        reg.setInputSource (points_with_normals1);

        PointCloudWithNormals::Ptr reg_result = points_with_normals1;
        
        //Get Results
        reg.align (*reg_result, current_guess_);

        RCL_CPP_INFO_STREAM(get_logger(), "ICP with Normals converged:" << reg.hasConverged ()
                << " score: " << reg.getFitnessScore () << " epsilon:" << reg.getTransformationEpsilon());
        
        if(reg.hasConverged() ) current_guess_ = reg.getFinalTransformation();


      #endif
    }
    
  //convert transformation to ros usable form
    Eigen::Matrix4f mf = (current_guess_);
    Eigen::Matrix4d md(mf.cast<double>());
    Eigen::Affine3d affine(md);
    geometry_msgs::TransformStamped transformStamped = tf2::eigenToTransform(affine);
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
      geometry_msgs::Quaternion q_mean = Cloud_Alignment::AverageQuaternion(output_->transform.rotation);

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
    
    geometry_msgs::TransformStamped gm_diff;
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
      RCL_CPP_INFO_STREAM(get_logger(), "input0 frozen");
    }
    if(freeze1_) 
    {
      RCL_CPP_INFO_STREAM(get_logger(), "input1 frozen");
    }

    RCL_CPP_INFO_STREAM(get_logger(), "X:     " << output_->transform.translation.x << " m" << ", diff: " << diff_x << " m");
    RCL_CPP_INFO_STREAM(get_logger(), "Y:     " << output_->transform.translation.y << " m" << ", diff: " << diff_y << " m");
    RCL_CPP_INFO_STREAM(get_logger(), "Z:     " << output_->transform.translation.z << " m" << ", diff: " << diff_z << " m");

    tf2::Quaternion q(
          output_->transform.rotation.x,
          output_->transform.rotation.y,
          output_->transform.rotation.z,
          output_->transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);
    RCL_CPP_INFO_STREAM(get_logger(), "Roll:  " << roll <<  " rad, " << (roll/PI*180)  << " deg" << ", diff: " << diff_roll << " rad");
    RCL_CPP_INFO_STREAM(get_logger(), "pitch: " << pitch << " rad, " << (pitch/PI*180) << " deg" << ", diff: " << diff_pitch << " rad");
    RCL_CPP_INFO_STREAM(get_logger(), "Yaw:   " << yaw <<   " rad, " << (yaw/PI*180)   << " deg" << ", diff: " << diff_yaw << " rad");

    // Create output msgs
    geometry_msgs::TransformStamped::Ptr output(output_);
    sensor_msgs::PointCloud2::Ptr output_msg0(new sensor_msgs::PointCloud2);
    sensor_msgs::PointCloud2::Ptr output_msg1(new sensor_msgs::PointCloud2);
    sensor_msgs::PointCloud2 p_msg1; pcl::toROSMsg(*output_cloud0, *output_msg0);
    pcl::toROSMsg(*output_cloud1, *output_msg1);
    //first convert output_msg1 to parent_frame then apply output transform
    // geometry_msgs::TransformStamped pTransform = tfBuffer_.lookupTransform(parent_frame, output_msg1->header.frame_id, output_msg1->header.stamp);
    // tf2::doTransform(*output_msg1, *output_msg1, pTransform);
    // tf2::doTransform(*output_msg1, *output_msg1, *output_);

    // correct error in cloud1's output msg 
    tf2::doTransform(*output_msg1, *output_msg1, gm_diff);

    output_trans_pub_.publish(output);
    output_cloud0_pub_.publish(output_msg0);
    output_cloud1_pub_.publish(output_msg1);
  }

  void Cloud_Alignment::input0_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    
    if(freeze0_) 
    {
      return;
    }
    
    sensor_msgs::PointCloud2 cloud_in(*msg), cloud_out;
    geometry_msgs::TransformStamped transform;

    if(parent_frame_id_ != "")
    {
      try{
        transform = tfBuffer_.lookupTransform(parent_frame_id_, msg->header.frame_id, msg->header.stamp);
        
        tf2::doTransform(*msg, cloud_out, transform);

        cloud0_ = cloud_out;
        }
      catch (tf2::TransformException ex){
        RCL_CPP_WARN("%s",ex.what());
        return;
      }
    }

  }

  void Cloud_Alignment::input1_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    if(freeze1_) 
    {
      return;
    }

    sensor_msgs::PointCloud2 cloud_in(*msg), cloud_out;
    geometry_msgs::TransformStamped transform;

     if(child_frame_id_ != "")
    {
      try{
        transform = tfBuffer_.lookupTransform(child_frame_id_, msg->header.frame_id, msg->header.stamp);
        
        tf2::doTransform(*msg, cloud_out, transform);

        cloud1_ = cloud_out;
      }
      catch (tf2::TransformException ex){
        RCLCPP_WARN("%s",ex.what());
        return;
      }
    }

  }

  bool Cloud_Alignment::freeze0_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp)
  {
    freeze0_ = true;

    return true;	  
  }

  bool Cloud_Alignment::freeze1_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp)
  {
    freeze1_ = true;	  
    
    return true;
  }

  bool Cloud_Alignment::unfreeze0_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp)
  {
    RCL_CPP_INFO_STREAM(get_logger(), "input0 unfrozen");
    freeze0_ = false;	  

    return true;
  }

  bool Cloud_Alignment::unfreeze1_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp)
  {
    RCL_CPP_INFO_STREAM(get_logger(), "input1 unfrozen");
    freeze1_ = false;	  
    
    return true;
  }

  bool Cloud_Alignment::reset()
  {
    alignPubConfig_ = initialAlignPubConfig_;
    return alignClient_->setConfiguration(alignPubConfig_);
  }
  
  bool Cloud_Alignment::revert()
  {
    //revert Guess
    last_transform_ = tfBuffer_.lookupTransform(parent_frame_id_, child_frame_id_, ros::Time::now()).transform;
    
    Eigen::Affine3d eigenTransform = tf2::transformToEigen(last_transform_);
    current_guess_ = eigenTransform.matrix().cast<float>();
    
    //revert rolling window accumulators
    x_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    y_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    z_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    qx_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    qy_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    qz_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    qw_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    current_qx_ = 0; current_qy_ = 0; current_qz_ = 0; current_qw_ = 0; 
    
    RCL_CPP_INFO_STREAM(node_name, "Guess transform reverted");

    return true;
  }

  bool Cloud_Alignment::revert_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp)
  {
    return Cloud_Alignment::revert();
  }

  bool Cloud_Alignment::reset_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp)
  {
    return Cloud_Alignment::reset();
  }

  bool Cloud_Alignment::pushtransform_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp)
  {
    return Cloud_Alignment::pushTransform();
  }
  
  bool Cloud_Alignment::pushYaw_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp)
  {
    return Cloud_Alignment::pushYaw();
  }
  
  bool Cloud_Alignment::pushRollPitchCorrection_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp)
  {
    return Cloud_Alignment::pushRollPitchCorrection();
  }
  

  void Cloud_Alignment::DownsampleCloud(const pcl::PointCloud<PointT>::Ptr in_cloud,
                                                pcl::PointCloud<PointT> &out_cloud,
                                                double in_leaf_size)

  {
    pcl::PointCloud<PointT>::Ptr filtered_ptr(new pcl::PointCloud<PointT>);
  
    // build the condition
    pcl::ConditionAnd<PointT>::Ptr range_cond (new pcl::ConditionAnd<PointT> ());
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("intensity", pcl::ComparisonOps::GE, alignToolConfig_.i_min)));
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("intensity", pcl::ComparisonOps::LE, alignToolConfig_.i_max)));
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("x",         pcl::ComparisonOps::GE, alignToolConfig_.x_min)));
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("x",         pcl::ComparisonOps::LE, alignToolConfig_.x_max)));
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("y",         pcl::ComparisonOps::GE, alignToolConfig_.y_min)));
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("y",         pcl::ComparisonOps::LE, alignToolConfig_.y_max)));
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("z",         pcl::ComparisonOps::GE, alignToolConfig_.z_min)));
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("z",         pcl::ComparisonOps::LE, alignToolConfig_.z_max)));
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

  geometry_msgs::Quaternion Cloud_Alignment::AverageQuaternion(const geometry_msgs::Quaternion& newRotation)
  {
    //ROS_INFO_STREAM_NAMED(node_name, current_qx_ << " " << current_qy_ << " " << current_qz_ << " " << current_qw_);
    tf2::Quaternion lastRotation(current_qx_, current_qy_, current_qz_, current_qw_);
    tf2::Quaternion currRotation; 
    tf2::convert(newRotation, currRotation);
    //On first pass lastRotation will be zero length
    if(abs(lastRotation.length()) < 0.1)
    {
      ROS_INFO_STREAM_NAMED(node_name, "AverageQuaternion initialized");

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
    //     ROS_INFO_STREAM_NAMED(node_name, "flip quaternion");
    //     ROS_INFO_STREAM_NAMED(node_name, currRotation.x() << " " << currRotation.y() << " " << currRotation.z() << " " << currRotation.w());
    //     ROS_INFO_STREAM_NAMED(node_name, lastRotation.x() << " " << lastRotation.y() << " " << lastRotation.z() << " " << lastRotation.w());
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

    geometry_msgs::Quaternion result;
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





