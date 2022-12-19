
#include "cb_align_tool/cb_align_tool.h"

namespace Multi_Sensor_Alignment
{
  Chessboard_Alignment::Chessboard_Alignment(const ros::NodeHandle &node_handle, 
                        const ros::NodeHandle &private_node_handle, int buffer_size) 
  // Initialization list
  :nh_(node_handle),
  pnh_(private_node_handle),
  current_guess_(Eigen::Matrix4f::Identity()),
  output_transform_(new geometry_msgs::TransformStamped),
  optical_transform_(new geometry_msgs::TransformStamped),
  sensor_transform_(new geometry_msgs::TransformStamped),
  cloud_transform_(new geometry_msgs::TransformStamped),
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
  Chessboard_Alignment::~Chessboard_Alignment()
  {
    // delete sourceCloud_;
    // delete sourceCamera_;
  }

  void Chessboard_Alignment::onInit()
  {

    const std::string complete_ns = pnh_.getNamespace();
    std::size_t id = complete_ns.find_last_of("/");
    node_name = complete_ns.substr(id + 1);

  //Setup Dynamic Reconfigure Server for this node
    dynamic_reconfigure::Server<multi_sensor_alignment::cb_align_toolConfig>::CallbackType
        drServerCallback_ = boost::bind(&Chessboard_Alignment::reconfigure_server_callback, this, _1, _2);
    drServer_.reset(new dynamic_reconfigure::Server<multi_sensor_alignment::cb_align_toolConfig>(drServer_mutex_, pnh_));
    drServer_->setCallback(drServerCallback_);

    //Wait on this nodes dynamic param server to intialize values
    while(!received_alignToolConfig_)
    {
      ros::Duration(1.0).sleep();
      ROS_INFO_STREAM_NAMED(node_name, "Waiting on dynamic parameters.");
      ros::spinOnce();
    }

  //Subscribe to Dynamic Reconfigure on the alignment publisher node, AlignPubConfig
    pnh_.param<std::string>("alignment_server", align_server_name_, "");
      ROS_INFO_STREAM_NAMED(node_name, "alignment_server set to " << align_server_name_);   
    alignClient_.reset(new dynamic_reconfigure::Client<multi_sensor_alignment::alignment_publisherConfig>(align_server_name_));
    alignClient_->setConfigurationCallback(boost::bind(&Chessboard_Alignment::align_pubconfig_callback, this, _1));
    alignClient_->setDescriptionCallback(boost::bind(&Chessboard_Alignment::align_pubdesc_callback, this, _1));
    
    //Wait up to 60 seconds for the alignment publisher nodes dynamic param server to respond
    int count = 0, maxcount = 60;
    while((count < maxcount && (!received_alignPubConfig_ || !received_alignPubDesc_)) && align_server_name_ != "")
    {
      ros::Duration(1.0).sleep();
      ROS_INFO_STREAM_NAMED(node_name, "Waiting on dynamic parameters from align_publisher. " << (maxcount-count) << " sec before giving up.");
      ros::spinOnce();
      count++;
    }

  // ROS Parameters
    pnh_.param<std::string>("parent_frame", parent_frame_id_, "");
      ROS_INFO_STREAM_NAMED(node_name, "parent_frame set to " << parent_frame_id_);
    pnh_.param<std::string>("child_frame", child_frame_id_, "");
      ROS_INFO_STREAM_NAMED(node_name, "child_frame set to " << child_frame_id_);
    pnh_.param("wait_for_tf_delay", wait_for_tf_delay_, wait_for_tf_delay_);
      ROS_INFO_STREAM_NAMED(node_name, "wait_for_tf_delay set to " << wait_for_tf_delay_);

    pnh_.param("grid_cols", grid_cols_, 9);
      ROS_INFO_STREAM_NAMED(node_name, "grid_cols set to " << grid_cols_);
    pnh_.param("grid_rows", grid_rows_, 6);
      ROS_INFO_STREAM_NAMED(node_name, "grid_rows set to " << grid_rows_);  
    pnh_.param("square_size_mm", square_size_, 60.0);
      ROS_INFO_STREAM_NAMED(node_name, "square_size set to " << square_size_);
    pnh_.param("board_height_mm", board_height_, 600.0);
      ROS_INFO_STREAM_NAMED(node_name, "board_height set to " << board_height_);
    pnh_.param("board_width_mm", board_width_, 420.0);
      ROS_INFO_STREAM_NAMED(node_name, "board_width set to " << board_width_);
    pnh_.param("height_offset_mm", height_offset_, 0.0);
      ROS_INFO_STREAM_NAMED(node_name, "height_offset set to " << height_offset_);      
    pnh_.param("width_offset_mm", width_offset_, 0.0);
      ROS_INFO_STREAM_NAMED(node_name, "width_offset set to " << width_offset_);      

    if(!received_alignPubConfig_)
    {
      ROS_INFO_STREAM_NAMED(node_name, "Proceeding without alignment publisher.");
    }

    ROS_INFO_STREAM_NAMED(node_name, "x set to " << alignPubConfig_.x);
    ROS_INFO_STREAM_NAMED(node_name, "y set to " << alignPubConfig_.y);
    ROS_INFO_STREAM_NAMED(node_name, "z set to " << alignPubConfig_.z);
    ROS_INFO_STREAM_NAMED(node_name, "roll set to " << alignPubConfig_.roll);
    ROS_INFO_STREAM_NAMED(node_name, "pitch set to " << alignPubConfig_.pitch);
    ROS_INFO_STREAM_NAMED(node_name, "yaw set to " << alignPubConfig_.yaw);

    pnh_.param<std::string>("input_cloud_topic", input_cloud_topic_, "input_cloud");
      ROS_INFO_STREAM_NAMED(node_name, "input_cloud_topic set to " << input_cloud_topic_);
    pnh_.param<std::string>("input_image_topic", input_image_topic_, "input_image");
      ROS_INFO_STREAM_NAMED(node_name, "input_image_topic set to " << input_image_topic_);
    pnh_.param<std::string>("input_info_topic", input_info_topic_, "input_info");
      ROS_INFO_STREAM_NAMED(node_name, "input_info_topic set to " << input_info_topic_);
    pnh_.param<std::string>("image_cloud_topic", image_cloud_topic_, "image_cloud");
      ROS_INFO_STREAM_NAMED(node_name, "image_cloud_topic set to " << image_cloud_topic_);
    // pnh_.param<std::string>("filter_cloud_topic", filter_cloud_topic_, "filter_cloud");
    //   ROS_INFO_STREAM_NAMED(node_name, "filter_cloud_topic set to " << filter_cloud_topic_);
    pnh_.param<std::string>("output_cloud0_topic", output_cloud0_topic_, "output_cloud0");
      ROS_INFO_STREAM_NAMED(node_name, "output_cloud0_topic set to " << output_cloud0_topic_);
    pnh_.param<std::string>("output_cloud1_topic", output_cloud1_topic_, "output_cloud1");
      ROS_INFO_STREAM_NAMED(node_name, "output_cloud1_topic set to " << output_cloud1_topic_);
    pnh_.param<std::string>("output_camera_topic", output_camera_topic_, "output_camera");
      ROS_INFO_STREAM_NAMED(node_name, "output_camera_topic set to " << output_camera_topic_);
      pnh_.param<std::string>("output_info_topic", output_info_topic_, "camera_info");
      ROS_INFO_STREAM_NAMED(node_name, "output_info_topic set to " << output_info_topic_);
    pnh_.param<std::string>("output_marker_topic", output_marker_topic_, "output_marker");
      ROS_INFO_STREAM_NAMED(node_name, "output_marker_topic set to " << output_marker_topic_);
    pnh_.param<std::string>("output_tranform", output_trans_topic_, "output_trans");
      ROS_INFO_STREAM_NAMED(node_name, "output_trans_topic set to " << output_trans_topic_);
    pnh_.param("output_frequency", output_frequency_, 10.0);
      ROS_INFO_STREAM_NAMED(node_name, "output_frequency set to " << output_frequency_);     
    pnh_.param("is_rectified", is_rectified_, false);
      ROS_INFO_STREAM_NAMED(node_name, "is_rectified set to " << is_rectified_);     
      
    pnh_.param("filter/i_min", alignToolConfig_.i_min, alignToolConfig_.i_min);
      ROS_INFO_STREAM_NAMED(node_name, "filter min i set to " << alignToolConfig_.i_min);   
    pnh_.param("filter/i_max", alignToolConfig_.i_max, alignToolConfig_.i_max);
      ROS_INFO_STREAM_NAMED(node_name, "filter max i set to " << alignToolConfig_.i_max);  
    pnh_.param("filter/x_min", alignToolConfig_.x_min, alignToolConfig_.x_min);
      ROS_INFO_STREAM_NAMED(node_name, "filter min x set to " << alignToolConfig_.x_min);   
    pnh_.param("filter/x_max", alignToolConfig_.x_max, alignToolConfig_.x_max);
      ROS_INFO_STREAM_NAMED(node_name, "filter max x set to " << alignToolConfig_.x_max);   
    pnh_.param("filter/y_min", alignToolConfig_.y_min, alignToolConfig_.y_min);
      ROS_INFO_STREAM_NAMED(node_name, "filter min y set to " << alignToolConfig_.y_min);   
    pnh_.param("filter/y_max", alignToolConfig_.y_max, alignToolConfig_.y_max);
      ROS_INFO_STREAM_NAMED(node_name, "filter max y set to " << alignToolConfig_.y_max);   
    pnh_.param("filter/z_min", alignToolConfig_.z_min, alignToolConfig_.z_min);
      ROS_INFO_STREAM_NAMED(node_name, "filter min z set to " << alignToolConfig_.z_min);   
    pnh_.param("filter/z_max", alignToolConfig_.z_max, alignToolConfig_.z_max);
      ROS_INFO_STREAM_NAMED(node_name, "filter max z set to " << alignToolConfig_.z_max);

    pnh_.param("plane_tol", alignToolConfig_.plane_tol, 0.05);
      ROS_INFO_STREAM_NAMED(node_name, "plane_tol set to " << alignToolConfig_.plane_tol);

  //initialized source classes
    sourceCloud_ = new ERDC::SourceCloud(nh_, input_cloud_topic_);
    pcl::PCLPointCloud2 cloud; tf2::Transform transform;
    std_msgs::Header header;
    sourceCloud_->cloud_buffer.push_back(cloud);
    sourceCloud_->transform_buffer.push_back(transform);
    sourceCloud_->header_buffer.push_back(header);
    sourceCamera_ = new ERDC::SourceCamera(nh_, input_image_topic_, input_info_topic_, is_rectified_, "field_label", 0, output_camera_topic_, output_info_topic_, "", 0, 0, false);


    drServer_->updateConfig(alignToolConfig_);

  // ROS publishers
    output_trans_pub_  = nh_.advertise<geometry_msgs::TransformStamped>(output_trans_topic_,100);
    image_cloud_pub_   = nh_.advertise<sensor_msgs::PointCloud2>(image_cloud_topic_,10);
    // filter_cloud_pub_  = nh_.advertise<sensor_msgs::PointCloud2>(filter_cloud_topic_,10);
    output_cloud0_pub_  = nh_.advertise<sensor_msgs::PointCloud2>(output_cloud0_topic_,10);
    output_cloud1_pub_  = nh_.advertise<sensor_msgs::PointCloud2>(output_cloud1_topic_,10);
    // output_info_pub_ = nh_.advertise<sensor_msgs::Image>(output_info_topic_,10);
    // output_camera_pub_ = nh_.advertise<sensor_msgs::Image>(output_camera_topic_,10);
    // output_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(output_marker_topic_, 10);
    pub_timer_ = nh_.createTimer(ros::Duration(1.0/output_frequency_), boost::bind(& Chessboard_Alignment::publish_callback, this, _1));

  // ROS subscribers
    sourceCloud_->cloud_sub = nh_.subscribe<sensor_msgs::PointCloud2>(sourceCloud_->cloud_in_topic, 1, boost::bind(&Chessboard_Alignment::input_cloud_callback, this, _1));
    sourceCamera_->cameraSync->registerCallback(boost::bind(&Chessboard_Alignment::input_camera_callback, this, _1, _2));

  // ROS Services
    if(received_alignPubConfig_)
    {
      service0_ = pnh_.advertiseService("reset", &Chessboard_Alignment::reset_callback, this);
      service1_ = pnh_.advertiseService("push_transform", &Chessboard_Alignment::pushtransform_callback, this);
    } 
    else 
    {
      ROS_WARN_STREAM_NAMED(node_name, "CB_ALIGN_TOOL did not find the alignment publisher.");
    }

  // Reset the guess transform for good measure
    Chessboard_Alignment::reset();

    ROS_INFO_STREAM_NAMED(node_name, node_name.c_str() << " initialized!");
  }

  void Chessboard_Alignment::reconfigure_server_callback(multi_sensor_alignment::cb_align_toolConfig &config, uint32_t level) 
  {
    
    ROS_INFO("Reconfigure Request: %f %f %f %f %f %f %f %f ", 
            config.plane_tol,
            config.i_min, config.i_max, 
            config.x_min, config.x_max, 
            config.y_min, config.y_max, 
            config.z_min, config.z_max);

    alignToolConfig_ = config;
    received_alignToolConfig_ = true;

    Chessboard_Alignment::reset();
  }

  void Chessboard_Alignment::align_pubconfig_callback(const multi_sensor_alignment::alignment_publisherConfig& config) 
  {
    ROS_INFO("cb_align_tool received new configuration from alignment publisher");
    ROS_INFO("%f %f %f %f %f %f", 
            config.x, config.y, config.z, config.roll, config.pitch, config.yaw);

    if (!received_alignPubConfig_) 
      initialAlignPubConfig_ = config;

    alignPubConfig_ = config;
    received_alignPubConfig_ = true;

    Chessboard_Alignment::reset();
  }

  void Chessboard_Alignment::align_pubdesc_callback(const dynamic_reconfigure::ConfigDescription& description) 
  {
    ROS_INFO("Received description from alignment publisher");

    alignPubDesc_ = description;
    received_alignPubDesc_ = true;
  }

  bool Chessboard_Alignment::pushTransform()
  {
    alignPubConfig_.x = output_transform_->transform.translation.x;
    alignPubConfig_.y = output_transform_->transform.translation.y;
    alignPubConfig_.z = output_transform_->transform.translation.z;

    tf2::Quaternion q;
    convert(output_transform_->transform.rotation, q);
    tf2::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);

    alignPubConfig_.roll = roll;
    alignPubConfig_.pitch = pitch;
    alignPubConfig_.yaw = yaw;

    return alignClient_->setConfiguration(alignPubConfig_);
  }

  void Chessboard_Alignment::publish_callback(const ros::TimerEvent& event)
  {
    // ROS_WARN_STREAM("Publish Callback");
  // Gather Input
    // Image
    const std::lock_guard<std::mutex> lock_camera(camera_mutex_);
    if(sourceCamera_->image.image.empty()) 
    { 
      ROS_WARN_STREAM("No image found");
      return;
    }
    cv::Mat gray;
    cv::cvtColor(sourceCamera_->image.image, gray, CV_BGR2GRAY);

    // Pointcloud
    const std::lock_guard<std::mutex> lock_cloud(cloud_mutex_);
    pcl::PointCloud<PointT>::Ptr input_cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr lidar_cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr image_cloud(new pcl::PointCloud<PointT>);
    if(sourceCloud_->cloud_buffer.back().width < 1 || sourceCloud_->cloud_buffer.back().height < 1)
    {
       ROS_WARN_STREAM("No cloud found");
       return;
    }
    pcl::fromPCLPointCloud2(sourceCloud_->cloud_buffer.back(), *input_cloud);
    
  //Check image for Chessboard Pattern
    cv::Size2i patternNum(grid_rows_,grid_cols_);
    std::vector<cv::Point2f> chessCorners;
    bool patternfound = cv::findChessboardCorners(gray, patternNum, chessCorners,
                                                  cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);

  //Proceed with transforms if pattern found
    if(patternfound)
    {
       ROS_INFO_STREAM("Chessboard Found");
       
    // Find location of chessboard in image (image_transform) and create pointcloud (image_cloud) representing board
       ImageProcessing(gray, chessCorners, image_cloud); 

    //Filter lidar_cloud to area in vicinity of chessboard in the image   
       CloudProcessing(input_cloud, lidar_cloud);

    //Perform Registration
      Multi_Sensor_Alignment::PerformRegistration(lidar_cloud, image_cloud,
         current_guess_, alignToolConfig_.Method, alignToolConfig_.MaxIterations, alignToolConfig_.Epsilon,  
         alignToolConfig_.KSearch, alignToolConfig_.RadiusSearch, alignToolConfig_.MaxCorrespondenceDistance, 
         alignToolConfig_.StepSize, alignToolConfig_.Resolution);
    
    //convert transformation to ros usable form
      Eigen::Matrix4f mf = (current_guess_);
      Eigen::Matrix4d md(mf.cast<double>());
      Eigen::Affine3d affine(md);
      geometry_msgs::TransformStamped transformStamped = tf2::eigenToTransform(affine);
      output_transform_->transform = transformStamped.transform;
      output_transform_->header = pcl_conversions::fromPCL(image_cloud->header);
      output_transform_->header.frame_id = parent_frame_id_;
      output_transform_->child_frame_id = child_frame_id_;
    
    //Add new transform to accumulator and compute average
      if(buffer_size_ > 1)
      {
        x_array_(output_transform_->transform.translation.x); double x_mean = rolling_mean(x_array_);
        y_array_(output_transform_->transform.translation.y); double y_mean = rolling_mean(y_array_);
        z_array_(output_transform_->transform.translation.z); double z_mean = rolling_mean(z_array_);
        geometry_msgs::Quaternion q_mean = Multi_Sensor_Alignment::AverageQuaternion(output_transform_->transform.rotation, 
            qx_array_, qy_array_, qz_array_, qw_array_, current_qx_, current_qy_, current_qz_, current_qw_);

        //Update transformations with average
        output_transform_->transform.translation.x = x_mean;
        output_transform_->transform.translation.y = y_mean;
        output_transform_->transform.translation.z = z_mean;
        output_transform_->transform.rotation = q_mean;

        Eigen::Affine3d eigenTransform = tf2::transformToEigen(output_transform_->transform);
        current_guess_ = eigenTransform.matrix().cast<float>();
      }

    // Calculate diff from last_transform
      tf2::Transform old_trans, new_trans;
      tf2::convert(last_transform_, old_trans);
      tf2::convert(output_transform_->transform, new_trans);
      tf2::Transform diff = old_trans.inverseTimes(new_trans);
      
      geometry_msgs::TransformStamped gm_diff;
      tf2::convert(diff, gm_diff.transform);
      gm_diff.header = pcl_conversions::fromPCL(image_cloud->header);
      gm_diff.header.frame_id = child_frame_id_;
      gm_diff.child_frame_id = child_frame_id_;

      tf2::Vector3 diff_vector(diff.getOrigin());
      tf2::Matrix3x3 diff_matrix(diff.getRotation());
      double diff_x, diff_y, diff_z, diff_roll, diff_pitch, diff_yaw;
      diff_x = diff_vector.getX();
      diff_y = diff_vector.getY();
      diff_z = diff_vector.getZ();
      diff_matrix.getRPY(diff_roll, diff_pitch, diff_yaw);

      double roll,pitch,yaw;
      tf2::Quaternion q(
            output_transform_->transform.rotation.x,
            output_transform_->transform.rotation.y,
            output_transform_->transform.rotation.z,
            output_transform_->transform.rotation.w);
      tf2::Matrix3x3 m(q);
      m.getRPY(roll,pitch,yaw);

      ROS_INFO_STREAM_NAMED(node_name, "ICP Translation X: " << output_transform_->transform.translation.x << " m" << ", diff: " << diff_x << " m");
      ROS_INFO_STREAM_NAMED(node_name, "ICP Translation Y: " << output_transform_->transform.translation.y << " m" << ", diff: " << diff_y << " m");
      ROS_INFO_STREAM_NAMED(node_name, "ICP Translation Z: " << output_transform_->transform.translation.z << " m" << ", diff: " << diff_z << " m");

      ROS_INFO_STREAM_NAMED(node_name, "Roll:  " << roll <<  " rad, " << (roll/PI*180)  << " deg" << ", diff: " << diff_roll << " rad");
      ROS_INFO_STREAM_NAMED(node_name, "pitch: " << pitch << " rad, " << (pitch/PI*180) << " deg" << ", diff: " << diff_pitch << " rad");
      ROS_INFO_STREAM_NAMED(node_name, "Yaw:   " << yaw <<   " rad, " << (yaw/PI*180)   << " deg" << ", diff: " << diff_yaw << " rad");

      
    // Create output msgs
      pcl::PointCloud<PointT>::Ptr output_cloud0(new pcl::PointCloud<PointT>);
      pcl::PointCloud<PointT>::Ptr output_cloud1(new pcl::PointCloud<PointT>);

      pcl::copyPointCloud (*lidar_cloud, *output_cloud0);
      pcl::copyPointCloud (*image_cloud, *output_cloud1);

      geometry_msgs::TransformStamped::Ptr output(output_transform_);
      sensor_msgs::PointCloud2::Ptr output_msg0(new sensor_msgs::PointCloud2);
      sensor_msgs::PointCloud2::Ptr output_msg1(new sensor_msgs::PointCloud2);
      sensor_msgs::PointCloud2 p_msg1;
      pcl::toROSMsg(*output_cloud0, *output_msg0);
      pcl::toROSMsg(*output_cloud1, *output_msg1);

      //first convert output_msg1 to parent_frame then apply output transform
      // geometry_msgs::TransformStamped pTransform = tfBuffer_.lookupTransform(parent_frame, output_msg1->header.frame_id, output_msg1->header.stamp);
      // tf2::doTransform(*output_msg1, *output_msg1, pTransform);
      // tf2::doTransform(*output_msg1, *output_msg1, *output_transform_);

      // correct error in cloud1's output msg 
      tf2::doTransform(*output_msg1, *output_msg1, gm_diff);

      output_trans_pub_.publish(output);
      output_cloud0_pub_.publish(output_msg0);
      output_cloud1_pub_.publish(output_msg1);

    }
    else
    {
      ROS_INFO_STREAM("Chessboard Not Found");
    }

  }

  void Chessboard_Alignment::ImageProcessing(cv::Mat &gray, std::vector<cv::Point2f> &chessCorners, pcl::PointCloud<PointT>::Ptr &cloud_out)
  {
  // Find chessboard features  
    std::vector<cv::Point3f> gridPoints;
    // Find intersecting corner points with sub-pixel accuracy
    cv::cornerSubPix(gray, chessCorners, cv::Size(11,11), cv::Size(-1,-1),
                  cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    cv::Size imgsize;
    imgsize.height = gray.rows;
    imgsize.width = gray.cols;
    cv::Size2i patternNum(grid_rows_,grid_cols_);
    cv::Size2d patternSize(square_size_,square_size_);
    double tx, ty; // Translation values
    // Location of board frame origin from the bottom left inner corner of the chessboard
    tx = (patternNum.height - 1) * patternSize.height/2;
    ty = (patternNum.width - 1) * patternSize.width/2;
    // Board corners w.r.t board frame
    for(int i = 0; i < patternNum.height; i++)
    {
      for(int j = 0; j < patternNum.width; j++)
      {
        cv::Point3f tmp_gridPoint;
        // Translating origin from bottom left corner to the centre of the chessboard
        tmp_gridPoint.x = i*patternSize.height - tx;
        tmp_gridPoint.y = j*patternSize.width - ty;
        tmp_gridPoint.z = 0;
        gridPoints.push_back(tmp_gridPoint);
      }
    }

  // Expected chessboard point coordinates in 3D frame based on origin at the centre of the chess pattern
    // Board corners
    std::vector< cv::Point3f > boardcorners;
    boardcorners.push_back(cv::Point3f( (board_width_ - width_offset_)/2,
                                        (board_height_ - height_offset_)/2, 0.0));
    boardcorners.push_back(cv::Point3f(-(board_width_ + width_offset_)/2,
                                        (board_height_ - height_offset_)/2, 0.0));
    boardcorners.push_back(cv::Point3f(-(board_width_ + width_offset_)/2,
                                        -(board_height_ + height_offset_)/2, 0.0));
    boardcorners.push_back(cv::Point3f( (board_width_ - width_offset_)/2,
                                        -(board_height_ + height_offset_)/2, 0.0));
    // Board center
    boardcorners.push_back(cv::Point3f(-width_offset_/2,
                                        -height_offset_/2, 0.0));

    // // Board centre chessboard square corner coordinates wrt the centre of the board (origin)
    // std::vector< cv::Point3f > square_edge;
    // square_edge.push_back(cv::Point3f(-square_size_/2, -square_size_/2, 0.0));
    // square_edge.push_back(cv::Point3f( square_size_/2,  square_size_/2, 0.0));

    // // chessboard corners, middle square corners, board corners and centre
    // std::vector<cv::Point2f> imagePoints0, imagePoints1, imagePoints2;

  // Find transform using 3D-2D point correspondences.
    cv::Mat rvec(3,3,cv::DataType<double>::type); // Initialization for pinhole and fisheye cameras
    cv::Mat tvec(3,1,cv::DataType<double>::type);
    if (sourceCamera_->distortion_model == "plumb_bob")
    {
      //Finds an object pose from 3D-2D point correspondences. 
      //This function returns the rotation and the translation vectors that transform a 3D point expressed in the object coordinate frame to the camera coordinate frame
      cv::solvePnP(gridPoints, chessCorners, sourceCamera_->intrinsic_matrix, sourceCamera_->distortion_coefficients, rvec, tvec);
      // // Convert all to image coordinates 
      // cv::projectPoints(gridPoints, rvec, tvec, sourceCamera_->intrinsic_matrix, sourceCamera_->distortion_coefficients, imagePoints0);
      // cv::projectPoints(square_edge, rvec, tvec, sourceCamera_->intrinsic_matrix, sourceCamera_->distortion_coefficients, imagePoints1);
      // cv::projectPoints(boardcorners, rvec, tvec, sourceCamera_->intrinsic_matrix, sourceCamera_->distortion_coefficients, imagePoints2);
    }
    else
    {
      ROS_WARN_STREAM(sourceCamera_->distortion_model << " distortion Model not implemented");
    }
    
  //Convert OpenCv Transform into geometry_msgs::Transform
    Eigen::Affine3d cb_pose;  cb_pose.setIdentity();
    cv::Mat tmprmat = cv::Mat(3,3,CV_64F); // rotation matrix
    cv::Rodrigues(rvec,tmprmat); // rvec to rotation matrix

    for(int j = 0; j < 3; j++)
    {
      for(int k = 0; k < 3; k++)
      {
        cb_pose(j,k) = tmprmat.at<double>(j,k);
      }
      cb_pose(j,3) = tvec.at<double>(j);
    }
    geometry_msgs::Transform transform = tf2::eigenToTransform(cb_pose).transform;
    optical_transform_->header.frame_id = sourceCamera_->frameID;
    optical_transform_->header.stamp = sourceCamera_->transform.header.stamp;
    optical_transform_->transform.translation.x = transform.translation.x/1000.0;
    optical_transform_->transform.translation.y = transform.translation.y/1000.0;
    optical_transform_->transform.translation.z = transform.translation.z/1000.0;
    optical_transform_->transform.rotation = transform.rotation;

  //Visualize Points
    // take every point in boardcorners set
    for (int k = 0; k < boardcorners.size(); k++)
    {
      cv::Point3f pt(boardcorners[k]);
      cv::Mat image_points = cv::Mat::eye(3,5,CV_64F);

      // Transform to obtain the coordinates in optical frame
      for (int i = 0; i < 3; i++)
      {
        image_points.at<double>(i,k) = cb_pose(i,0)*pt.x +
            cb_pose(i,1)*pt.y + cb_pose(i,3);
      }

      // convert 3D coordinates to image coordinates
      double * img_coord = Chessboard_Alignment::convert_to_imgpts(image_points.at<double>(0,k),
                                                                   image_points.at<double>(1,k),
                                                                   image_points.at<double>(2,k));
      // Mark the corners and the board centre
      if (k==0)
        cv::circle(sourceCamera_->image.image, cv::Point(img_coord[0],img_coord[1]),
            12, CV_RGB(0,255,0),-1); //green
      else if (k==1)
        cv::circle(sourceCamera_->image.image, cv::Point(img_coord[0],img_coord[1]),
            12, CV_RGB(255,255,0),-1); //yellow
      else if (k==2)
        cv::circle(sourceCamera_->image.image, cv::Point(img_coord[0],img_coord[1]),
            12, CV_RGB(0,0,255),-1); //blue
      else if (k==3)
        cv::circle(sourceCamera_->image.image, cv::Point(img_coord[0],img_coord[1]),
            12, CV_RGB(255,0,0),-1); //red
      else
        cv::circle(sourceCamera_->image.image, cv::Point(img_coord[0],img_coord[1]),
            12, CV_RGB(255,255,255),-1); //white for centre

      delete[] img_coord;
    }

  // Republish the image with all the features marked on it
    sensor_msgs::CameraInfo::ConstPtr info;
    info = PublishCameraInfo(sourceCamera_);
    PublishCameraImage(sourceCamera_, info);

  //transform location of chessboard from optical frame to child frame
    tf2::doTransform(*optical_transform_, *sensor_transform_, sourceCamera_->transform);

  //Create pointcloud from chessboard location in child frame
    pcl::PointCloud<PointT>::Ptr image_corners(new pcl::PointCloud<PointT>);
    //corners
    for (int k = 0; k < boardcorners.size(); k++)
    {
      PointT temp_point;
      temp_point.x = boardcorners[k].x/1000;
      temp_point.y = boardcorners[k].y/1000;
      temp_point.z = boardcorners[k].z/1000;
      image_corners->points.push_back(temp_point);
    }
    //center
    PointT temp_point;
    temp_point.x = 0.0;
    temp_point.y = 0.0;
    temp_point.z = 0.0;
    image_corners->points.push_back(temp_point);
    //fill
    int n = 20;
    int m = 20;
    for(int i = 0; i < n+1; i++)
    {
      for(int j = 0; j < m+1; j++)
      {
        double weight0 = (1.0-i/(double)n) * ((1.0-j/(double)m));
        double weight1 =     (i/(double)n) * ((1.0-j/(double)m));
        double weight2 =     (i/(double)n) *     ((j/(double)m));
        double weight3 = (1.0-i/(double)n) *     ((j/(double)m));
        PointT temp_point;
        temp_point.x = (weight0*boardcorners[0].x + weight1*boardcorners[1].x + weight2*boardcorners[2].x + weight3*boardcorners[3].x)/1000;
        temp_point.y = (weight0*boardcorners[0].y + weight1*boardcorners[1].y + weight2*boardcorners[2].y + weight3*boardcorners[3].y)/1000;
        temp_point.z = (weight0*boardcorners[0].z + weight1*boardcorners[1].z + weight2*boardcorners[2].z + weight3*boardcorners[3].z)/1000;
        // ROS_INFO_STREAM(temp_point.x << ":" << temp_point.y << ":" << temp_point.z);
        cloud_out->points.push_back(temp_point);
      }
    }

    sensor_msgs::PointCloud2 cloud_optical; pcl::toROSMsg(*cloud_out, cloud_optical);
    tf2::doTransform(cloud_optical, cloud_optical, *optical_transform_);

    // Publish the cloud in the images frame
    image_cloud_pub_.publish(cloud_optical);

    // store the cloud in the child frame
    sensor_msgs::PointCloud2 cloud_sensor; pcl::toROSMsg(*cloud_out, cloud_sensor);
    tf2::doTransform(cloud_sensor, cloud_sensor, *sensor_transform_);
    pcl::fromROSMsg(cloud_sensor, *cloud_out);
  }

  void Chessboard_Alignment::CloudProcessing(const pcl::PointCloud<PointT>::Ptr &in_cloud, pcl::PointCloud<PointT>::Ptr &out_cloud)
  {
    ROS_INFO_STREAM("Filter Cloud");
  //Filter the cloud using user defined filtering parameters
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    Multi_Sensor_Alignment::DownsampleCloud(in_cloud, cloud_filtered, 0.0,
      alignToolConfig_.i_min, alignToolConfig_.i_max, 
      alignToolConfig_.x_min, alignToolConfig_.x_max,
      alignToolConfig_.y_min, alignToolConfig_.y_max,
      alignToolConfig_.z_min, alignToolConfig_.z_max);

  //Further filter the cloud by drawing a cube around the center point found in the image transform
    pcl::CropBox<PointT> boxFilter;
    geometry_msgs::TransformStamped cloud_transform, total_transform;
    cloud_transform = tfBuffer_.lookupTransform(cloud_filtered->header.frame_id, sensor_transform_->header.frame_id,  pcl_conversions::fromPCL(cloud_filtered->header.stamp));
    //transform location of chessboard from optical frame to child frame
    tf2::doTransform(*sensor_transform_, total_transform, cloud_transform);

    float maxDim = std::max(board_width_, board_height_)/1000;
    float minX = total_transform.transform.translation.x - 0.55*maxDim;
    float maxX = total_transform.transform.translation.x + 0.55*maxDim;
    float minY = total_transform.transform.translation.y - 0.55*maxDim;
    float maxY = total_transform.transform.translation.y + 0.55*maxDim;
    float minZ = total_transform.transform.translation.z - 0.55*maxDim;
    float maxZ = total_transform.transform.translation.z + 0.55*maxDim;
    // ROS_INFO_STREAM("  min:" << minX << ":" << minY << ":" << minZ);
    // ROS_INFO_STREAM("  max:" << maxX << ":" << maxY << ":" << maxZ);
    boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    boxFilter.setInputCloud(cloud_filtered);
    boxFilter.filter(*out_cloud);

  // //Find Plane of Board
  //   int count = 0;
  //   pcl::PointCloud<PointT>::Ptr cloud_outliers(new pcl::PointCloud<PointT>(*cloud_filtered2));
  //   pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>);
  //   pcl::PointCloud<PointT>::Ptr cloud_hull(new pcl::PointCloud<PointT>);
  //   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_corner(new pcl::PointCloud<pcl::PointXYZ>);
  //   pcl::PointCloud<PointT>::Ptr cloud_combined(new pcl::PointCloud<PointT>);
  //   cloud_plane->header = cloud_hull->header = cloud_corner->header = cloud_combined->header = cloud_outliers->header;
  //   pcl::PointXYZ point_normal;
    
  //   while(cloud_outliers->size() > 30)
  //   {
  //     count = count + 1;
  //   //Fit plane
  //     ROS_INFO_STREAM("Fit Plane");
  //     //Use image plane as initial guess
  //     pcl::ModelCoefficients::Ptr coefficients_image(new pcl::ModelCoefficients);
  //     pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  //     coefficients_image->values.emplace_back(0.0);
  //     coefficients_image->values.emplace_back(0.0);
  //     coefficients_image->values.emplace_back(1.0);
  //     coefficients_image->values.emplace_back(0.0);
  //     ROS_INFO_STREAM("  image plane:" << coefficients_image->values[0] << ":" << coefficients_image->values[1] << ":" << coefficients_image->values[2] << ":" << coefficients_image->values[3]);
  //     coefficients = coefficients_image;

  //     Eigen::Affine3d eigenTransform = tf2::transformToEigen(sensor_transform_->transform);

  //     // Eigen::Matrix3d m = eigenTransform.rotation();
  //     // Eigen::Vector3d v = eigenTransform.translation();
  //     // Eigen::Quaterniond q = (Eigen::Quaterniond)eigenTransform.linear();
  //     // double x, y, z, qx, qy, qz, qw;
  //     //  x = v.x();
  //     //  y = v.y();
  //     //  z = v.z();
  //     //  qx = q.x();
  //     //  qy = q.y();
  //     //  qz = q.z();
  //     //  qw = q.w();
  //     //    //ROS_INFO_STREAM("image transform");    
  //     // ROS_INFO_STREAM("image_transform->translation\n" << x << "\n" << y << "\n" << z);
  //     // ROS_INFO_STREAM("image_transform->quaterion\n"  << qx << "\n" << qy << "\n" << qz << "\n" << qw);

  //     ROS_INFO_STREAM("  Transform");
  //     pcl::transformPlane(coefficients_image, coefficients, eigenTransform);
  //     ROS_INFO_STREAM("    guess plane:" << coefficients->values[0] << ":" << coefficients->values[1] << ":" << coefficients->values[2] << ":" << coefficients->values[3]);

  //     pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  //     pcl::SACSegmentation<PointT> seg;
  //     seg.setOptimizeCoefficients(true);
  //     seg.setModelType(pcl::SACMODEL_PLANE);
  //     seg.setMethodType(pcl::SAC_RANSAC);
  //     seg.setDistanceThreshold(alignToolConfig_.plane_tol);
  //     seg.setMaxIterations(100);
  //     seg.setInputCloud(cloud_outliers);
  //     seg.segment(*inliers, *coefficients);

  //     if(inliers->indices.size() < 30)
  //     {
  //       ROS_WARN_STREAM("Unable to find plane in cloud segment " << count);
  //       break;
  //     }
  //     else
  //     {
  //        ROS_INFO_STREAM("  fit points:" << inliers->indices.size() << "/" << cloud_outliers->size());
  //        ROS_INFO_STREAM("  fit plane:" << coefficients->values[0] << ":" << coefficients->values[1] << ":" << coefficients->values[2] << ":" << coefficients->values[3]);
  //     }

  //   // Project the inliers on the plane
  //     pcl::ProjectInliers<PointT> proj;
  //     proj.setModelType(pcl::SACMODEL_PLANE);
  //     proj.setInputCloud(cloud_outliers);
  //     proj.setModelCoefficients(coefficients);
  //     proj.filter(*cloud_plane);
  //     float mag = sqrt(pow(coefficients->values[0], 2) + pow(coefficients->values[1], 2)
  //       + pow(coefficients->values[2], 2));
  //     float sign = 1;
  //     // if(coefficients->values[0] > 0) sign = -1;  //Vector should always point back toward camera 
  //     point_normal.x = sign*coefficients->values[0]/mag;
  //     point_normal.y = sign*coefficients->values[1]/mag;
  //     point_normal.z = sign*coefficients->values[2]/mag;
  //     ROS_INFO_STREAM("Plane Points: " << cloud_plane->size());

  //     // Publish the cloud
  //     sensor_msgs::PointCloud2 cloud_final;
  //     pcl::toROSMsg(*cloud_plane, cloud_final);
  //     output_cloud_pub_.publish(cloud_final);


  //     // create a visualization object
  //     // pcl::PointCloud<pcl::PointXYZ>::Ptr view_points (new pcl::PointCloud<pcl::PointXYZ>);
  //     // pcl::copyPointCloud<PointT>(*cloud_plane, *view_points);

  //     // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  //     //  viewer = Multi_Sensor_Alignment::simpleVis(view_points);
  //     //  viewer->spin();


  //     //Extract the outliers for next pass (i.e. current plane does not work out to be the board)
  //     ROS_DEBUG_STREAM("Extract outliers");
  //     pcl::ExtractIndices<PointT> extract;
  //     extract.setInputCloud(cloud_outliers);
  //     extract.setIndices(inliers);
  //     extract.setNegative (true);
  //     extract.filter(*cloud_outliers);

  //   //Use Convex hull to find edges points of plane
  //     ROS_DEBUG_STREAM("Hull");
  //     pcl::ConcaveHull<PointT> chull;
  //     chull.setAlpha(board_height_/1000.0/1.0);
  //     chull.setInputCloud(cloud_plane);
  //     chull.reconstruct (*cloud_hull);
  //     if(cloud_hull->points.size() < 15)
  //     {
  //       ROS_WARN_STREAM("Unable to find sufficient edge points on plane " << count);
  //       continue;
  //     }
  //     ROS_DEBUG_STREAM("Hull Points: " << cloud_hull->size());

  //     //Find key points (those with greatest y/z values). These should be the closest points to the corners. 
  //     // ToDo there is an underlying assumption that the working frame is X-forward and Z-up, really need a more robust solution
  //     //Start and bottom and go clockwise to organize the points
  //     ROS_DEBUG_STREAM("Key Points");
  //     pcl::PointCloud<PointT> key_points;
  //     for(int i = 0; i < 4; i++) key_points.push_back(cloud_hull->points[0]);
  //     for(int i = 0; i < cloud_hull->size(); i++)
  //     {
  //       if(cloud_hull->points[i].z < key_points.points[0].z) key_points.points[0] = cloud_hull->points[i];
  //       if(cloud_hull->points[i].y < key_points.points[1].y) key_points.points[1] = cloud_hull->points[i];
  //       if(cloud_hull->points[i].z > key_points.points[2].z) key_points.points[2] = cloud_hull->points[i];
  //       if(cloud_hull->points[i].y > key_points.points[3].y) key_points.points[3] = cloud_hull->points[i];
  //     }
  //     ROS_DEBUG_STREAM("Key Points: " << key_points.points[0] << key_points.points[1] << key_points.points[2] << key_points.points[3]);

  //     //Sort into edges using key points as boundaries
  //     ROS_DEBUG_STREAM("Group Edges");
  //     pcl::PointCloud<PointT>::Ptr downright(new pcl::PointCloud<PointT>);
  //     pcl::PointCloud<PointT>::Ptr upright(new pcl::PointCloud<PointT>);
  //     pcl::PointCloud<PointT>::Ptr upleft(new pcl::PointCloud<PointT>);
  //     pcl::PointCloud<PointT>::Ptr downleft(new pcl::PointCloud<PointT>);
  //     for(int i = 0; i < cloud_hull->size(); i++)
  //     {
  //       double plane_tol = alignToolConfig_.plane_tol;
  //       if( (cloud_hull->points[i].y < (key_points.points[0].y))            && (cloud_hull->points[i].z < (key_points.points[1].z)) 
  //        && (cloud_hull->points[i].y > (key_points.points[1].y+ plane_tol)) && (cloud_hull->points[i].z > (key_points.points[0].z + plane_tol))) { downright->push_back(cloud_hull->points[i]); cloud_combined->push_back(cloud_hull->points[i]); }
  //       if( (cloud_hull->points[i].y < (key_points.points[2].y))            && (cloud_hull->points[i].z > (key_points.points[1].z)) 
  //        && (cloud_hull->points[i].y > (key_points.points[1].y+ plane_tol)) && (cloud_hull->points[i].z < (key_points.points[2].z - plane_tol))) { upright->push_back(cloud_hull->points[i]); cloud_combined->push_back(cloud_hull->points[i]); }
  //       if( (cloud_hull->points[i].y > (key_points.points[2].y))            && (cloud_hull->points[i].z > (key_points.points[3].z)) 
  //        && (cloud_hull->points[i].y < (key_points.points[3].y- plane_tol)) && (cloud_hull->points[i].z < (key_points.points[2].z - plane_tol))) { upleft->push_back(cloud_hull->points[i]); cloud_combined->push_back(cloud_hull->points[i]); }
  //       if( (cloud_hull->points[i].y > (key_points.points[0].y))            && (cloud_hull->points[i].z < (key_points.points[3].z)) 
  //        && (cloud_hull->points[i].y < (key_points.points[3].y- plane_tol)) && (cloud_hull->points[i].z > (key_points.points[0].z + plane_tol))) { downleft->push_back(cloud_hull->points[i]); cloud_combined->push_back(cloud_hull->points[i]); }
  //     }
  //     ROS_INFO_STREAM("Edge Points: " << downright->points.size() << "," << upright->points.size() << "," << upleft->points.size() << "," << downleft->points.size());
  //     if((downright->points.size() < 3) || (upright->points.size() < 3) || (upleft->points.size() < 3) || (downleft->points.size() < 3))
  //     {
  //       ROS_WARN_STREAM("Insufficient points along one edge of plane " << count);
  //       continue;
  //     }

  //     //Fit lines through the the edge points
  //     ROS_DEBUG_STREAM("Fit Lines");
  //     std::vector<pcl::ModelCoefficients::Ptr> l_coefficients;
  //     std::vector<pcl::PointIndices::Ptr> l_inliers;
  //     pcl::PointCloud<PointT>::Ptr hull_outliers(new pcl::PointCloud<PointT>(*cloud_hull));

  //     seg.setModelType(pcl::SACMODEL_LINE);
  //     seg.setMethodType(pcl::SAC_RANSAC);
  //     seg.setDistanceThreshold(alignToolConfig_.plane_tol*2);
  //     seg.setOptimizeCoefficients(true);
      
  //     pcl::ModelCoefficients::Ptr a_coefficients (new pcl::ModelCoefficients);
  //     pcl::PointIndices::Ptr a_inliers (new pcl::PointIndices);
  //     seg.setInputCloud(downright);
  //     seg.segment (*a_inliers, *a_coefficients);
  //     l_coefficients.push_back(a_coefficients);
  //     //ROS_DEBUG_STREAM("downright: " << a_coefficients->values.size());

      
  //     pcl::ModelCoefficients::Ptr b_coefficients (new pcl::ModelCoefficients);
  //     pcl::PointIndices::Ptr b_inliers (new pcl::PointIndices);
  //     seg.setInputCloud(upright);
  //     seg.segment (*b_inliers, *b_coefficients);
  //     l_coefficients.push_back(b_coefficients);
  //     //ROS_DEBUG_STREAM("upright: " << b_coefficients->values.size());

  //     pcl::ModelCoefficients::Ptr c_coefficients (new pcl::ModelCoefficients);
  //     pcl::PointIndices::Ptr c_inliers (new pcl::PointIndices);
  //     seg.setInputCloud(upleft);
  //     seg.segment (*c_inliers, *c_coefficients);
  //     l_coefficients.push_back(c_coefficients);
  //     //ROS_DEBUG_STREAM("upleft: " << c_coefficients->values.size());

  //     pcl::ModelCoefficients::Ptr d_coefficients (new pcl::ModelCoefficients);
  //     pcl::PointIndices::Ptr d_inliers (new pcl::PointIndices);
  //     seg.setInputCloud(downleft);  // might need to clear inliers and coefficients
  //     seg.segment (*d_inliers, *d_coefficients);
  //     l_coefficients.push_back(d_coefficients);
  //     //ROS_DEBUG_STREAM("downleft: " << d_coefficients->values.size());

  //   //Find board corners as line intersections
  //     ROS_DEBUG_STREAM("Find Intersections");
  //     Eigen::Vector4f Point_l;
  //     pcl::PointXYZ basic_point; // intersection points stored here
  //     double plane_tol = alignToolConfig_.plane_tol;
  //     if(pcl::lineWithLineIntersection(*l_coefficients[3], *l_coefficients[0], Point_l, plane_tol*plane_tol/4.))
  //     {
  //       basic_point.x = Point_l[0];
  //       basic_point.y = Point_l[1];
  //       basic_point.z = Point_l[2];
  //       cloud_corner->points.push_back(basic_point);
  //       ROS_DEBUG_STREAM("point 1=" << basic_point);
  //     }
  //     else
  //     {
  //       ROS_WARN_STREAM("Unable to find corner point 1 in plane " << count);
  //       continue;
  //     }
      
  //     if(pcl::lineWithLineIntersection(*l_coefficients[0], *l_coefficients[1], Point_l, plane_tol*plane_tol/4.))
  //     {
  //       basic_point.x = Point_l[0];
  //       basic_point.y = Point_l[1];
  //       basic_point.z = Point_l[2];
  //       cloud_corner->points.push_back(basic_point);
  //       ROS_DEBUG_STREAM("point 2=" << basic_point);
  //     }
  //           else
  //     {
  //       ROS_WARN_STREAM("Unable to find corner point 2 in plane " << count);
  //       continue;
  //     }

  //     if(pcl::lineWithLineIntersection(*l_coefficients[1], *l_coefficients[2], Point_l, plane_tol*plane_tol/4.))
  //     {
  //       basic_point.x = Point_l[0];
  //       basic_point.y = Point_l[1];
  //       basic_point.z = Point_l[2];
  //       cloud_corner->points.push_back(basic_point);
  //       ROS_DEBUG_STREAM("point 3=" << basic_point);
  //     }
  //     else
  //     {
  //       ROS_WARN_STREAM("Unable to find corner point 3 in plane " << count);
  //       continue;
  //     }
  //     if(pcl::lineWithLineIntersection(*l_coefficients[2], *l_coefficients[3], Point_l, plane_tol*plane_tol/4.))
  //     {
  //       basic_point.x = Point_l[0];
  //       basic_point.y = Point_l[1];
  //       basic_point.z = Point_l[2];
  //       cloud_corner->points.push_back(basic_point);
  //       ROS_DEBUG_STREAM("point 4=" << basic_point);
  //     }
  //     else
  //     {
  //       ROS_WARN_STREAM("Unable to find corner point 4 in plane " << count);
  //       continue;
  //     }

  //     // Check length of diagonals
  //     float diagonal1x = cloud_corner->points[2].x-cloud_corner->points[0].x;
  //     float diagonal1y = cloud_corner->points[2].y-cloud_corner->points[0].y;
  //     float diagonal1z = cloud_corner->points[2].z-cloud_corner->points[0].z;
  //     float diagonal2x = cloud_corner->points[3].x-cloud_corner->points[1].x;
  //     float diagonal2y = cloud_corner->points[3].y-cloud_corner->points[1].y;
  //     float diagonal2z = cloud_corner->points[3].z-cloud_corner->points[1].z;
  //     float actual_length1 = diagonal1x*diagonal1x + diagonal1y*diagonal1y + diagonal1z*diagonal1z;
  //     float actual_length2 = diagonal2x*diagonal2x + diagonal2y*diagonal2y + diagonal2z*diagonal2z;
  //     float expected_length = board_width_*board_width_ + board_height_*board_height_;
  //     if(actual_length1 < 0.9*expected_length || actual_length1 > 1.1*expected_length  ||
  //        actual_length2 < 0.9*expected_length || actual_length2 > 1.1*expected_length)
  //     {
  //       ROS_WARN_STREAM("Board dimensions do not work in plane " << count);
  //       continue;
  //     } 
      
  //     //Find center from corner diagonals
  //     pcl::PointXYZ center1;
  //     center1.x = (cloud_corner->points[2].x+cloud_corner->points[0].x)/2.0;
  //     center1.y = (cloud_corner->points[2].y+cloud_corner->points[0].y)/2.0;
  //     center1.z = (cloud_corner->points[2].z+cloud_corner->points[0].z)/2.0;
  //     pcl::PointXYZ center2;
  //     center2.x = (cloud_corner->points[3].x+cloud_corner->points[1].x)/2.0;
  //     center2.y = (cloud_corner->points[3].y+cloud_corner->points[1].y)/2.0;
  //     center2.z = (cloud_corner->points[3].z+cloud_corner->points[1].z)/2.0;
  //     if(((center1.x - center2.x)*(center1.x - center2.x) +
  //         (center1.y - center2.y)*(center1.y - center2.y) + 
  //         (center1.z - center2.z)*(center1.z - center2.z)) > plane_tol*plane_tol);
  //     {
  //       ROS_WARN_STREAM("Board diagonals do not cross on plane " << count);
  //       continue;
  //     } 

  //     basic_point.x = (center1.x + center2.x)/2.0;
  //     basic_point.y = (center1.y + center2.y)/2.0;
  //     basic_point.z = (center1.z + center2.z)/2.0;
  //     cloud_corner->points.push_back(basic_point);
  //     ROS_DEBUG_STREAM("point 5=" << basic_point);
      
  //     break;
  //   }
  
  // // Publish the cloud
  //   sensor_msgs::PointCloud2 cloud_final;
  //   if(cloud_corner->points.size() == 5) pcl::toROSMsg(*cloud_combined, cloud_final);
  //   else pcl::toROSMsg(*cloud_hull, cloud_final);
  //   ROS_DEBUG_STREAM("Publishing Cloud");
  //   output_cloud_pub_.publish(cloud_final);

  //   if(cloud_corner->points.size() != 5) return;

  // //Turn results into a pose
  //   Eigen::Vector3d xy_normal_vector, board_normal_vector, plane_rotation_vector;
  //   xy_normal_vector[0] = 0.0;
  //   xy_normal_vector[1] = 0.0;
  //   xy_normal_vector[2] = 1.0;
  //   board_normal_vector[0] = point_normal.x;
  //   board_normal_vector[1] = point_normal.y;
  //   board_normal_vector[2] = point_normal.z;
  //   board_normal_vector.normalize();
  //   plane_rotation_vector = xy_normal_vector.cross(board_normal_vector);
  //   double plane_rotation_angle = atan2(plane_rotation_vector.norm(), xy_normal_vector.dot(board_normal_vector));
  //   plane_rotation_vector.normalize();
 
  //   Eigen::Affine3d cb_pose = Eigen::Affine3d::Identity();
  //   cb_pose = Eigen::AngleAxisd(plane_rotation_angle, plane_rotation_vector);
  //   cb_pose(0,3) = cloud_corner->points[4].x;
  //   cb_pose(1,3) = cloud_corner->points[4].y;
  //   cb_pose(2,3) = cloud_corner->points[4].z;
  //   //ROS_INFO_STREAM("corners camera frame" << "\n" << cloud_corner->points[0] << "\n" << cloud_corner->points[1] << "\n" << cloud_corner->points[2] << "\n" << cloud_corner->points[3] << "\n" << cloud_corner->points[4]);

  // // The transform, cb_pose, as it stands will transform into the plane of the board.  
  //   // However, we are still lacking the final orientation about the normal of this plane.
  //   // We need another rotation transform to align the board edges with the xy axis of the board coordinate system.
  //   geometry_msgs::TransformStamped inverse = tf2::eigenToTransform(cb_pose);
  //   tf2::Transform transform_tf; tf2::convert(inverse.transform, transform_tf);
  //   tf2::Transform inverse_tf = transform_tf.inverse();
  //   tf2::convert(inverse_tf, inverse.transform);
  //   sensor_msgs::PointCloud2 corners_xy; pcl::toROSMsg(*cloud_corner, corners_xy);
  //   tf2::doTransform(corners_xy, corners_xy, inverse);
  //   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_corner_xy(new pcl::PointCloud<pcl::PointXYZ>);
  //   pcl::fromROSMsg(corners_xy, *cloud_corner_xy);
    
  //   //ROS_INFO_STREAM("corners board unoriented" << "\n" << cloud_corner_xy->points[0] << "\n" << cloud_corner_xy->points[1] << "\n" << cloud_corner_xy->points[2] << "\n" << cloud_corner_xy->points[3] << "\n" << cloud_corner_xy->points[4]);
    
  //   // Additional rotation vector
  //   Eigen::Vector3d x_vector, orient_rotation;
  //   x_vector[0] = 1.0;
  //   x_vector[1] = 0.0;
  //   x_vector[2] = 0.0;
  //   Eigen::Vector4d rotation_angles(0.0,0.0,0.0,0.0);
  //   //ROS_INFO_STREAM("corner angles");
  //   // calculate rotations for all edges and take average
  //   cloud_corner_xy->points[4] = cloud_corner_xy->points[0]; //copy first value to end of list for easy loopback.
  //   for(int i = 0; i < 4; i++)
  //   {
  //     Eigen::Vector3d edge_vector, rotation_vector;
  //     edge_vector[0] = std::abs(cloud_corner_xy->points[i+1].x - cloud_corner_xy->points[i].x);
  //     edge_vector[1] = std::abs(cloud_corner_xy->points[i+1].y - cloud_corner_xy->points[i].y);
  //     edge_vector[2] = std::abs(cloud_corner_xy->points[i+1].z - cloud_corner_xy->points[i].z);
  //     rotation_vector = x_vector.cross(edge_vector);
  //     rotation_angles[i] = atan2(rotation_vector.norm(), x_vector.dot(edge_vector));
  //     if(rotation_angles[i] > +PI/4) rotation_angles[i] = +PI/2 - rotation_angles[i];
  //     if(rotation_angles[i] < -PI/4) rotation_angles[i] = -PI/2 - rotation_angles[i];
  //     // ROS_INFO_STREAM(rotation_angles[i]*180.0/PI);
  //   }
  //    //Update Transform
  //   cb_pose = Eigen::AngleAxisd(PI/2-rotation_angles.mean(), board_normal_vector)
  //            *Eigen::AngleAxisd(plane_rotation_angle, plane_rotation_vector);
  //   cb_pose(0,3) = cloud_corner->points[4].x;
  //   cb_pose(1,3) = cloud_corner->points[4].y;
  //   cb_pose(2,3) = cloud_corner->points[4].z;
    
  // // Check transform by applying the inverse transform to the points.  
  //   // Should end up with all points in x,y plane and symmetric about 0,0.
  //   geometry_msgs::TransformStamped inverse2 = tf2::eigenToTransform(cb_pose);
  //   tf2::Transform transform_tf2; tf2::convert(inverse2.transform, transform_tf2);
  //   tf2::Transform inverse_tf2 = transform_tf2.inverse();
  //   tf2::convert(inverse_tf2, inverse2.transform);
  //   pcl::toROSMsg(*cloud_corner, corners_xy);
  //   tf2::doTransform(corners_xy, corners_xy, inverse2);
  //   pcl::fromROSMsg(corners_xy, *cloud_corner_xy);
  //   //ROS_INFO_STREAM("corners board frame" << "\n" << cloud_corner_xy->points[0] << "\n" << cloud_corner_xy->points[1] << "\n" << cloud_corner_xy->points[2] << "\n" << cloud_corner_xy->points[3] << "\n" << cloud_corner_xy->points[4]);
    
  //   geometry_msgs::TransformStamped transform = tf2::eigenToTransform(cb_pose);
  //   cloud_transform_->transform.translation.x = transform.transform.translation.x;
  //   cloud_transform_->transform.translation.y = transform.transform.translation.y;
  //   cloud_transform_->transform.translation.z = transform.transform.translation.z;
  //   cloud_transform_->transform.rotation = transform.transform.rotation;    

  //   //Visualize the corner points of velodyne board, 4 corners and center
  //   visualization_msgs::Marker corners_board;
  //   corners_board.header.frame_id = cloud->header.frame_id;
  //   corners_board.header.stamp = pcl_conversions::fromPCL(cloud->header.stamp);
  //   corners_board.ns = "my_sphere";
  //   corners_board.type = visualization_msgs::Marker::SPHERE;
  //   corners_board.action = visualization_msgs::Marker::ADD;
  //   corners_board.pose.orientation.w = 1.0;
  //   corners_board.scale.x = 0.1; corners_board.scale.y = 0.1; corners_board.scale.z = 0.1;
  //   corners_board.color.a = 1.0;
  //   for (int i = 0; i < cloud_corner->points.size(); i++)
  //   {
  //     corners_board.pose.position.x = cloud_corner->points[i].x;
  //     corners_board.pose.position.y = cloud_corner->points[i].y;
  //     corners_board.pose.position.z = cloud_corner->points[i].z;
      
  //     corners_board.id = i;
  //     if (corners_board.id == 0) {
  //       corners_board.color.r = 1.0; corners_board.color.g = 1.0; corners_board.color.b = 0.0; }
  //     else if (corners_board.id == 1) {
  //       corners_board.color.r = 0.0; corners_board.color.g = 0.0; corners_board.color.b = 1.0; }
  //     else if (corners_board.id == 2) {
  //       corners_board.color.r = 1.0; corners_board.color.b = 0.0; corners_board.color.g = 0.0; }
  //     else if (corners_board.id == 3) {
  //       corners_board.color.r = 0.0; corners_board.color.g = 1.0; corners_board.color.b = 0.0;  }
  //     else if (corners_board.id == 4) {
  //       corners_board.color.r = 1.0; corners_board.color.g = 1.0; corners_board.color.b = 1.0;  }
  //     output_marker_pub_.publish(corners_board);
  //   }

  //   // Visualize board normal vector
  //   visualization_msgs::Marker normal;
  //   normal.header.frame_id = cloud->header.frame_id;
  //   normal.header.stamp = pcl_conversions::fromPCL(cloud->header.stamp);
  //   normal.ns = "my_normal";
  //   normal.id = 12;
  //   normal.type = visualization_msgs::Marker::ARROW;
  //   normal.action = visualization_msgs::Marker::ADD;
  //   normal.scale.x = 0.04; normal.scale.y = 0.06; normal.scale.z = 0.1;
  //   normal.color.a = 1.0; normal.color.r = 0.0; normal.color.g = 0.0; normal.color.b = 1.0;
  //   geometry_msgs::Point start, end;
  //   start.x = cloud_corner->points[4].x;
  //   start.y = cloud_corner->points[4].y;
  //   start.z = cloud_corner->points[4].z;
  //   end.x = start.x + point_normal.x/2;
  //   end.y = start.y + point_normal.y/2;
  //   end.z = start.z + point_normal.z/2;
  //   normal.points.resize(2);
  //   normal.points[0].x = start.x;
  //   normal.points[0].y = start.y;
  //   normal.points[0].z = start.z;
  //   normal.points[1].x = end.x;
  //   normal.points[1].y = end.y;
  //   normal.points[1].z = end.z;
  //   output_marker_pub_.publish(normal);

  }
  // /// Sorts lines by how horizontal they are in the xz plane.  Used to match near horizontal lines to near vertical lines for finding corners by line intersection.
  // bool Chessboard_Alignment::compareLineCoeff(const pcl::ModelCoefficients::Ptr &c1, const pcl::ModelCoefficients::Ptr &c2)
  // { 
  //   double x1 = c1->values[3], x2 = c2->values[3];
  //   double y1 = c1->values[5], y2 = c2->values[5];
  //   if( y1 < 0.0) x1 *= -1;
  //   if( y2 < 0.0) x2 *= -1;
  //   return (x1 > x2);
  // }
  // /// Sorts points by z value.
  // bool Chessboard_Alignment::comparePoints(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2)
  // { 
  //   return (p1.z < p2.z);
  // } 
  
  void Chessboard_Alignment::input_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    const std::lock_guard<std::mutex> lock(cloud_mutex_);
    //Transform to storage coordinates
    sensor_msgs::PointCloud2 cloud_in(*msg), cloud_transformed;
    geometry_msgs::TransformStamped transform;
    
    if(parent_frame_id_ != "")
    {
      try
      {
        transform = tfBuffer_.lookupTransform(parent_frame_id_, msg->header.frame_id, msg->header.stamp);
        tf2::doTransform(*msg, cloud_transformed, transform);
      }
      catch (tf2::TransformException ex)
      {
        ROS_WARN_STREAM_NAMED(node_name, "Cloud Callback: " << ex.what());
        return;
      }
    }

    pcl::PCLPointCloud2 pcl_out;
    pcl_conversions::toPCL(cloud_transformed, pcl_out);
    
    sourceCloud_->cloud_buffer.back() = pcl_out;
    tf2::Transform currentPose;
    tf2::convert(transform.transform, currentPose);
    sourceCloud_->transform_buffer.back() = currentPose;
    sourceCloud_->header_buffer.back() = msg->header;

    return;
  }

  void Chessboard_Alignment::camera_info_callback(const sensor_msgs::CameraInfo::ConstPtr& msg)
  {
    // Make local copy
    sensor_msgs::CameraInfo info_in(*msg);

    //Lock mutex
    std::lock_guard<std::mutex> myLock(*sourceCamera_->mutex);

    if (sourceCamera_->outputWidth < 1 || sourceCamera_->outputWidth > msg->width) sourceCamera_->outputWidth = msg->width;
    if (sourceCamera_->outputHeight < 1 || sourceCamera_->outputHeight > msg->height) sourceCamera_->outputHeight = msg->height;
    sourceCamera_->distortion_model = msg->distortion_model;

    //Calculate scaling values
    double xScale = ((double)(sourceCamera_->outputWidth))/((double)(msg->width));
    double yScale = ((double)(sourceCamera_->outputHeight))/((double)(msg->height)); 

    //Extract pinhole camera variables
    sourceCamera_->fx = static_cast<float>(info_in.K[0]) * xScale;
    sourceCamera_->fy = static_cast<float>(info_in.K[4]) * yScale;
    sourceCamera_->cx = static_cast<float>(info_in.K[2]) * xScale;
    sourceCamera_->cy = static_cast<float>(info_in.K[5]) * yScale;

    //Extract intrinsics matrix
    sourceCamera_->intrinsic_matrix = cv::Mat(3, 3, CV_64F);
    for (int row = 0; row < 3; row++)
    {
      for (int col = 0; col < 3; col++)
      {
        if(sourceCamera_->cameraRectified) sourceCamera_->intrinsic_matrix.at<double>(row, col) = info_in.P[row * 4 + col];
        else sourceCamera_->intrinsic_matrix.at<double>(row, col) = info_in.K[row * 3 + col];

        if(row == 0) sourceCamera_->intrinsic_matrix.at<double>(row, col) *= xScale;
        if(row == 1) sourceCamera_->intrinsic_matrix.at<double>(row, col) *= yScale;
      }
    }

    //Extract distortion coefficients
    sourceCamera_->distortion_coefficients = cv::Mat(1, info_in.D.size(), CV_64F);
   for (int col = 0; col < sourceCamera_->distortion_coefficients.total(); col++)
    {
      if(sourceCamera_->cameraRectified) sourceCamera_->distortion_coefficients.at<double>(col) = 0.0;
      else sourceCamera_->distortion_coefficients.at<double>(col) = info_in.D[col];
    }

    //Extract rectification matrix
    sourceCamera_->rectification_matrix = cv::Mat(3, 3, CV_64F);
    for (int row = 0; row < 3; row++)
    {
      for (int col = 0; col < 3; col++)
      {
        sourceCamera_->rectification_matrix.at<double>(row, col) = info_in.R[row * 3 + col];
      }
    }

    //Extract projection matrix
    sourceCamera_->projection_matrix = cv::Mat(3, 4, CV_64F);
    for (int row = 0; row < 3; row++)
    {
      for (int col = 0; col < 4; col++)
      {
        sourceCamera_->projection_matrix.at<double>(row, col) = info_in.P[row * 4 + col];

        if(row == 0) sourceCamera_->projection_matrix.at<double>(row, col) *= xScale;
        if(row == 1) sourceCamera_->projection_matrix.at<double>(row, col) *= yScale;
      }
    }

    sourceCamera_->camera_info_received = true;
  }

  void Chessboard_Alignment::input_camera_callback(const sensor_msgs::Image::ConstPtr& image_msg, const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
    camera_info_callback(info_msg);

    const std::lock_guard<std::mutex> myLock(*sourceCamera_->mutex);

    //Obtain info for transform
    sourceCamera_->frameID = image_msg->header.frame_id;

    geometry_msgs::TransformStamped transform;

    if(child_frame_id_ != "")
    {
      try
      {
        transform = tfBuffer_.lookupTransform(child_frame_id_, image_msg->header.frame_id, image_msg->header.stamp);
        sourceCamera_->transform = transform;
      }
      catch (tf2::TransformException ex)
      {
        ROS_WARN_STREAM_NAMED(node_name, "Camera Callback: " << ex.what());
        return;
      }
    }

    //Extract image
    try
    {
      cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_msg, image_msg->encoding);
      sourceCamera_->image = *cv_image;
            
      int count = sourceCamera_->image.image.rows * sourceCamera_->image.image.cols;
      ROS_DEBUG_STREAM_NAMED(node_name, "Image encoding " << image_msg->encoding);
      ROS_DEBUG_STREAM_NAMED(node_name, sourceCamera_->cameraImage_topic  << " has " << count << " pixels.");
    }
    catch (cv_bridge::Exception& e) 
    {
      ROS_ERROR("cv_bridge exception: %s", e.what()); 
    }
  }

  bool Chessboard_Alignment::reset()
  {
    //revert Guess
    last_transform_ = tfBuffer_.lookupTransform(parent_frame_id_, child_frame_id_, ros::Time::now()).transform;
     
    Eigen::Affine3d eigenTransform = tf2::transformToEigen(last_transform_);
    //current_guess_ = eigenTransform.matrix().cast<float>();
    
    //Reset rolling window accumulators
    x_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    y_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    z_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    qx_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    qy_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    qz_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    qw_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    current_qx_ = 0; current_qy_ = 0; current_qz_ = 0; current_qw_ = 0; 
    
    ROS_DEBUG_STREAM_NAMED(node_name, "Guess transform reset");

    return true;
  }

  bool Chessboard_Alignment::reset_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp)
  {
    return Chessboard_Alignment::reset();
  }

  bool Chessboard_Alignment::pushtransform_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp)
  {
    return Chessboard_Alignment::pushTransform();
  }

  // Convert 3D points w.r.t camera frame to 2D pixel points in image frame
  double * Chessboard_Alignment::convert_to_imgpts(double x, double y, double z)
  {  
    double tmpxC = x/z;
    double tmpyC = y/z;
    cv::Point2d planepointsC;
    planepointsC.x = tmpxC;
    planepointsC.y = tmpyC;
    double r2 = tmpxC*tmpxC + tmpyC*tmpyC;

    if (sourceCamera_->distortion_model == "plumb_bob")
    {
      double tmpdist = 1 + sourceCamera_->distortion_coefficients.at<double>(0)*r2 + sourceCamera_->distortion_coefficients.at<double>(1)*r2*r2 +
          sourceCamera_->distortion_coefficients.at<double>(4)*r2*r2*r2;
      planepointsC.x = tmpxC*tmpdist + 2*sourceCamera_->distortion_coefficients.at<double>(2)*tmpxC*tmpyC +
          sourceCamera_->distortion_coefficients.at<double>(3)*(r2+2*tmpxC*tmpxC);
      planepointsC.y = tmpyC*tmpdist + sourceCamera_->distortion_coefficients.at<double>(2)*(r2+2*tmpyC*tmpyC) +
          2*sourceCamera_->distortion_coefficients.at<double>(3)*tmpxC*tmpyC;
      planepointsC.x = sourceCamera_->intrinsic_matrix.at<double>(0,0)*planepointsC.x + sourceCamera_->intrinsic_matrix.at<double>(0,2);
      planepointsC.y = sourceCamera_->intrinsic_matrix.at<double>(1,1)*planepointsC.y + sourceCamera_->intrinsic_matrix.at<double>(1,2);
    }

    double * img_coord = new double[2];
    *(img_coord) = planepointsC.x;
    *(img_coord+1) = planepointsC.y;

    return img_coord;
  }  

  void Chessboard_Alignment::PublishCameraImage(const ERDC::SourceCamera* image_entry, sensor_msgs::CameraInfo::ConstPtr info)
  {
    cv_bridge::CvImage::Ptr cv_image(new cv_bridge::CvImage);

    //new intrinsic matrix
    cv::Mat new_intrinsic_matrix = cv::Mat(3, 3, CV_64F);
    for (int row = 0; row < 3; row++)
    {
      for (int col = 0; col < 3; col++)
      {
        new_intrinsic_matrix.at<double>(row, col) = info->K[row * 3 + col];
      }
    }

    //Rectify image if needed
    if(image_entry->outputRectify && !image_entry->cameraRectified) 
    {
      cv_image->header = image_entry->image.header;
      cv_image->encoding = image_entry->image.encoding;
      cv::undistort(image_entry->image.image, cv_image->image, image_entry->intrinsic_matrix, image_entry->distortion_coefficients, new_intrinsic_matrix);
    }
    else 
    {
      cv_image->header = image_entry->image.header;
      cv_image->encoding = image_entry->image.encoding;
      cv_image->image = image_entry->image.image;
    }

    //publish image
    image_entry->outputImage_pub.publish(cv_image->toImageMsg(), info); 
  }

  sensor_msgs::CameraInfo::ConstPtr Chessboard_Alignment::PublishCameraInfo(const ERDC::SourceCamera* image_entry)
  {
    //Create camera info topic
    sensor_msgs::CameraInfo::Ptr info(new sensor_msgs::CameraInfo);
    info->header = image_entry->image.header;
    info->width = image_entry->outputWidth;
    info->height = image_entry->outputHeight;
    info->distortion_model = image_entry->distortion_model;

    //Distortion coefficients
    for (int col = 0; col < image_entry->distortion_coefficients.total(); col++)
    {
      if(image_entry->outputRectify) info->D.push_back(0.0);
      else info->D.push_back(image_entry->distortion_coefficients.at<double>(col));
    }

    //Intrinsic matrix
    for (int row = 0; row < 3; row++)
    {
      for (int col = 0; col < 3; col++)
      {
        if(image_entry->outputRectify) info->K[row * 3 + col] = image_entry->projection_matrix.at<double>(row, col);
        else info->K[row * 3 + col] = image_entry->intrinsic_matrix.at<double>(row, col);
      }
    }
  
    //Rectification matrix
    for (int row = 0; row < 3; row++)
    {
      for (int col = 0; col < 3; col++)
      {
        info->R[row * 3 + col] = image_entry->rectification_matrix.at<double>(row, col);
      }
    }

    //Projection matrix
    for (int row = 0; row < 3; row++)
    {
      for (int col = 0; col < 4; col++)
      {
        info->P[row * 4 + col] = image_entry->projection_matrix.at<double>(row, col);
      }
    }

    //publish camera info
    return info;
  }

}  // namespace Multi_Sensor_Alignment





