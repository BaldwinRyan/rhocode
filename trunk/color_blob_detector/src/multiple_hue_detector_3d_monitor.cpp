#include "color_blob_detector/multiple_hue_detector_3d_monitor.h"

namespace enc = sensor_msgs::image_encodings;
using namespace cv;

MultipleHueDetector3DMonitor::MultipleHueDetector3DMonitor(std::string node_name, ros::NodeHandle nh){

  nh_ = nh;

  nh_.param<std::string>("feature_type", feature_type_, DEFAULT_FEATURE_TYPE);
  nh_.param<std::string>("feature_name", feature_name_, DEFAULT_FEATURE_NAME);
  nh_.param<int>("display_hue", display_hue_, DEFAULT_DISPLAY_HUE);
  nh_.param<int>("blur_kernel", blur_kernel_, DEFAULT_BLUR_KERNEL);
  nh_.param<int>("max_cluster_size", max_cluster_size_, DEFAULT_MAX_CLUSTER_SIZE);
  nh_.param<int>("min_cluster_size", min_cluster_size_, DEFAULT_MIN_CLUSTER_SIZE);
  nh_.param<double>("within_cluster_distance", within_cluster_distance_, DEFAULT_WITHIN_CLUSTER_DISTANCE);
  nh_.param<bool>("display_image", display_image_, DEFAULT_DISPLAY_IMAGE);
  nh_.param<bool>("publish_markers", publish_markers_, DEFAULT_PUBLISH_MARKERS);
  nh_.param<int>("max_num_features", max_num_features_, DEFAULT_MAX_NUM_FEATURES);
  nh_.param<int>("update_rate", update_rate_, DEFAULT_UPDATE_RATE);
  nh_.param<std::string>("source_identifier", source_identifier_, DEFAULT_SOURCE_IDENTIFIER);
  nh_.param<std::string>("output_identifier", output_identifier_, DEFAULT_OUTPUT_IDENTIFIER);
  nh_.param<std::string>("marker_identifier", marker_identifier_, DEFAULT_MARKER_IDENTIFIER);
  nh_.param<std::string>("base_frame", base_frame_, DEFAULT_BASE_FRAME);

  //get attributes
  //first get attribute names
  XmlRpc::XmlRpcValue attribute_list;
  std::vector <std::string> att_names;
  if(nh_.getParam("attribute_names", attribute_list)){
    ROS_ASSERT(attribute_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
      
    for (int i = 0; i < (int)attribute_list.size(); i++) 
      {
	ROS_ASSERT(attribute_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
	std::string temp_att_name = static_cast<std::string>(attribute_list[i]);
	att_names.push_back(temp_att_name);
      }
  }


  // get attribute mins
  XmlRpc::XmlRpcValue attribute_min_dict;
  if(nh_.getParam("attribute_mins", attribute_min_dict)){
    ROS_ASSERT(attribute_min_dict.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      
    for (std::vector<std::string>::const_iterator att_name = att_names.begin(); att_name != att_names.end(); att_name++){
      
      if (attribute_min_dict.hasMember(*att_name)){
	XmlRpc::XmlRpcValue att_value_list = attribute_min_dict[*att_name];
	ROS_ASSERT(att_value_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
	std::vector <double> att_vals;
	for (int i = 0; i < (int)att_value_list.size(); i++) 
	  {
	    ROS_ASSERT(att_value_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	    double temp_val = static_cast<double>(att_value_list[i]);
	    ROS_INFO("attribute name: %s, with min: %4.3f", att_name->c_str(), temp_val);
	    att_vals.push_back(temp_val);
	  }
	attribute_mins_.insert(make_pair(*att_name, att_vals));
      }
      else
	ROS_ERROR("attribute_mins parameter not specified correctly."); 
    }
  }

  // get attribute maxes
  XmlRpc::XmlRpcValue attribute_max_dict;
  if(nh_.getParam("attribute_maxes", attribute_max_dict)){
    ROS_ASSERT(attribute_max_dict.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      
    for (std::vector<std::string>::const_iterator att_name = att_names.begin(); att_name != att_names.end(); att_name++){
      
      if (attribute_max_dict.hasMember(*att_name)){
	XmlRpc::XmlRpcValue att_value_list = attribute_max_dict[*att_name];
	ROS_ASSERT(att_value_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
	std::vector <double> att_vals;
	for (int i = 0; i < (int)att_value_list.size(); i++) 
	  {
	    ROS_ASSERT(att_value_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	    double temp_val = static_cast<double>(att_value_list[i]);
	    ROS_INFO("attribute name: %s, with max: %4.3f", att_name->c_str(), temp_val);
	    att_vals.push_back(temp_val);
	  }
	attribute_maxes_.insert(make_pair(*att_name, att_vals));
      }
      else
	ROS_ERROR("attribute_maxes parameter not specified correctly."); 
    }
  }

  // get attribute windows (std deviations)
  XmlRpc::XmlRpcValue attribute_win_dict;
  if(nh_.getParam("attribute_windows", attribute_win_dict)){
    ROS_ASSERT(attribute_win_dict.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      
    for (std::vector<std::string>::const_iterator att_name = att_names.begin(); att_name != att_names.end(); att_name++){
      
      if (attribute_win_dict.hasMember(*att_name)){
	XmlRpc::XmlRpcValue att_value_list = attribute_win_dict[*att_name];
	ROS_ASSERT(att_value_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
	std::vector <double> att_vals;
	for (int i = 0; i < (int)att_value_list.size(); i++) 
	  {
	    ROS_ASSERT(att_value_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	    double temp_val = static_cast<double>(att_value_list[i]);
	    ROS_INFO("attribute name: %s, with window: %4.3f", att_name->c_str(), temp_val);
	    att_vals.push_back(temp_val);
	  }
	attribute_windows_.insert(make_pair(*att_name, att_vals));
      }
      else
	ROS_ERROR("attribute_windows parameter not specified correctly."); 
    }
  }

  // get attribute step sizes
  XmlRpc::XmlRpcValue attribute_step_dict;
  if(nh_.getParam("attribute_steps", attribute_step_dict)){
    ROS_ASSERT(attribute_step_dict.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      
    for (std::vector<std::string>::const_iterator att_name = att_names.begin(); att_name != att_names.end(); att_name++){
      
      if (attribute_step_dict.hasMember(*att_name)){
	XmlRpc::XmlRpcValue att_value_list = attribute_step_dict[*att_name];
	ROS_ASSERT(att_value_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
	std::vector <double> att_vals;
	for (int i = 0; i < (int)att_value_list.size(); i++) 
	  {
	    ROS_ASSERT(att_value_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	    double temp_val = static_cast<double>(att_value_list[i]);
	    ROS_INFO("attribute name: %s, with step size from param: %4.3f", att_name->c_str(), temp_val);
	    att_vals.push_back(temp_val);
	  }
	attribute_steps_.insert(make_pair(*att_name, att_vals));
      }
      else
	ROS_ERROR("attribute_windows parameter not specified correctly."); 
    }
  }

  if(attribute_mins_.find("hue") == attribute_mins_.end())
    attribute_mins_.insert(make_pair(std::string("hue"), (double)169.0));

  if(attribute_maxes_.find("hue") == attribute_maxes_.end())
    attribute_maxes_.insert(make_pair(std::string("hue"), (double)179.0));

  if(attribute_windows_.find("hue") == attribute_windows_.end())
    attribute_windows_.insert(make_pair(std::string("hue"), (double)DEFAULT_WINDOW_SIZE));

  if(attribute_steps_.find("hue") == attribute_steps_.end())
    attribute_steps_.insert(make_pair(std::string("hue"), (double)DEFAULT_STEP_SIZE));

  new_data_ = false;
  updated_ = true;

  threshold_image_.reset(new Mat);
  rgb_image_.reset(new Mat);
  cluster_image_.reset(new Mat);
  hue_image_.reset(new Mat);
  pcl_in_.reset(new PointCloudRGB);

  for(int j = (int)attribute_mins_["hue"][0]; j < (int)attribute_maxes_["hue"][0]; j++){
    boost::shared_ptr<cv::Mat> bin_image(new Mat);
    boost::shared_ptr<pcl::PointIndices> color_index(new pcl::PointIndices);
    std::vector<pcl::PointIndices> stable_color;

    color_indices_.push_back(color_index);
    binary_images_.push_back(bin_image);
    stable_color_regions_indexes_.push_back(stable_color);
  }
  

  within_cluster_max_ = .2;   //maximum we would ever like the clusters to be
  within_cluster_scale_ = .001;   //scale for cluster distance ( 1mm )
  within_cluster_scaled_ = (int)(within_cluster_distance_ / within_cluster_scale_);

  node_name_ = node_name;
  window_name_ = node_name_ + " Window";

  if (display_image_) {
    namedWindow(window_name_, CV_WINDOW_NORMAL || CV_WINDOW_KEEPRATIO);
    //namedWindow(window_name_, CV_WINDOW_AUTOSIZE);
    //createTrackbar("Blur Kernel", window_name_, &display_hue_, (display_hue_ - (int)attribute_mins_["hue"][0]), NULL);
    createTrackbar("Blur Kernel", window_name_, &blur_kernel_, 25, NULL);
    createTrackbar("Cluster Distance", window_name_, &within_cluster_scaled_, (int)(within_cluster_max_ / within_cluster_scale_), NULL, this); 
    
    //necessary to have time to initialize display window correctly
    waitKey(50);
  }
  
  pointcloud_sub_ = nh_.subscribe(source_identifier_, 1, &MultipleHueDetector3DMonitor::pointCloudCallback, this);
  blob_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_identifier_,1);
  moments_pub_ = nh_.advertise<action_msgs::GaussianPointAttributeDistributionArray>(output_identifier_,1);
  timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_), &MultipleHueDetector3DMonitor::update, this);
  //register the goal and feedback callbacks
  ROS_INFO("MultipleHue Monitor %s started", node_name_.c_str());
}

void MultipleHueDetector3DMonitor::getPointIndices(int index)
{

  Size s = binary_images_[index]->size();

  //take the points that are non-zero in threshold_image_ and add them as point_indices
  color_indices_[index]->indices.clear();

  for(int i = 0; i < s.height; i++) {
    for(int j = 0; j < s.width; j++) {
      if(std::isfinite(pcl_in_->at(j,i).x) && std::isfinite(pcl_in_->at(j,i).y) && std::isfinite(pcl_in_->at(j,i).z)){
	//std::cout << " i: " << i << " j: " << j << std::endl;
	//std::cout << "point cloud val: " << pcl_in_->at(j,i) << std::endl;
	//std::cout << "image val: " << (int)(binary_images_[index]->at <uchar> (i, j))  << std::endl;

	if((int)(binary_images_[index]->at <uchar> (i, j)))
	  { 
	    color_indices_[index]->indices.push_back(i * s.width + j);
	  }
      }
    }
  }
}

void MultipleHueDetector3DMonitor::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  if(!new_data_ && updated_){    

    new_data_ = true;

    sensor_msgs::Image img;
    PointCloudRGB cloud_in;
    boost::scoped_ptr<Mat> hsv_image;	     // hsv image from cameras
    hsv_image.reset(new Mat);
    
    pcl::fromROSMsg (*msg, *pcl_in_);
    //required wait period to convert to pcl
    waitKey(5);
    pcl::toROSMsg (*pcl_in_, img);

    //std::cout << "w: " << pcl_in_->width << " h: " << pcl_in_->height << std::endl;

    rgb_image_ptr_ = cv_bridge::toCvCopy(img, "bgr8");

    rgb_image_->release();
    rgb_image_ptr_->image.copyTo(*rgb_image_);

    cvtColor(*rgb_image_, *hsv_image, CV_BGR2HSV);
    
    vector <Mat> planes;
    split(*hsv_image, planes);
    
    *hue_image_ = Mat::zeros(hue_image_->size(), CV_8UC1);
    planes[0].copyTo(*hue_image_);

    if((int)pcl_in_->size() > 0)
      updated_ = false;
    else
      new_data_ = false;

  }
}

void MultipleHueDetector3DMonitor::update(const ros::TimerEvent& e) {
  if(new_data_ && !updated_){

    updated_ = true;

    int mean_color_, j, window_size_;

    for(mean_color_ = (int)attribute_mins_["hue"][0], 
	  j = 0, 
	  window_size_ = (int)attribute_windows_["hue"][0]; 
	
	((mean_color_ < (int)attribute_maxes_["hue"][0]) && 
	 (j <(int)binary_images_.size())); 
	
	j++, mean_color_ += (int)attribute_steps_["hue"][0]){

      int rangeMin = mean_color_ - window_size_;
      int rangeMax = mean_color_ + window_size_;
	    
      Size ksize = Size(2 * blur_kernel_ + 1,2 * blur_kernel_ + 1);
      GaussianBlur(*hue_image_, *hue_image_, ksize, -1, -1);
      //blur(*hue_image_, *hue_image_, ksize, Point(-1, -1), BORDER_REPLICATE);
      //dilate(*hue_image_, *hue_image_, Mat(), Point(-1, -1), BORDER_REPLICATE);	    
    
      *(binary_images_[j]) = Mat::zeros(hue_image_->size(), CV_8UC1);

      if((rangeMin >= 0) && (rangeMax <= 179)){
	inRange(*hue_image_, Scalar((double)((uchar)rangeMin)),Scalar((double)((uchar)rangeMax)), *(binary_images_[j]));
      }
      else if (rangeMin >= 0){
	Mat temp_binary_image1(hue_image_->size(), CV_8UC1);
	Mat temp_binary_image2(hue_image_->size(), CV_8UC1);

	inRange(*hue_image_, Scalar((double)((uchar)rangeMin)),Scalar((double)((uchar)179)), temp_binary_image1);
	inRange(*hue_image_, Scalar((double)((uchar)0)),Scalar((double)((uchar)(rangeMax - 179))), temp_binary_image2);
	bitwise_or(temp_binary_image1, temp_binary_image2, *(binary_images_[j]));
      }
      else if (rangeMax <= 179){
	Mat temp_binary_image1(hue_image_->size(), CV_8UC1);
	Mat temp_binary_image2(hue_image_->size(), CV_8UC1);

	inRange(*hue_image_, Scalar((double)((uchar)0)),Scalar((double)((uchar)rangeMax)), temp_binary_image1);
	inRange(*hue_image_, Scalar((double)((uchar)(179 + rangeMin))),Scalar((double)((uchar)179)), temp_binary_image2);
	bitwise_or(temp_binary_image1, temp_binary_image2, *(binary_images_[j]));
      }
      else {
	ROS_ERROR("Window size parameter is too large.  Decrease it!");
	return;
      }  

      *threshold_image_ = Mat::zeros(hue_image_->size(), CV_8UC3);
      rgb_image_->copyTo(*threshold_image_, *(binary_images_[j]));

      getPointIndices(j);
    
      //if(((int)color_indices_[j]->indices.size() > 0) && (((int)color_indices_[j]->indices.size() < (max_cluster_size_ * max_num_features_))) && ((int)pcl_in_->size() > 0)){

      if(((int)color_indices_[j]->indices.size() > 0) && ((int)pcl_in_->size() > 0)){

	//std::cout << "j: " << j << std::endl;
	//std::cout << "# indices in image: " << (int)color_indices_[j]->indices.size() << std::endl;
	//std::cout << "point cloud size: " << (int)pcl_in_->size() << std::endl;
	//std::cout << "stable color region size: " << (int)stable_color_regions_indexes_[j].size() << std::endl;
    
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> cluster;	
	pcl::KdTree<pcl::PointXYZRGB>::Ptr clusters_tree(new pcl::KdTreeFLANN<pcl::PointXYZRGB>());

	cluster.setClusterTolerance (within_cluster_distance_);
	cluster.setMinClusterSize (min_cluster_size_);
	cluster.setMaxClusterSize (max_cluster_size_);
	cluster.setSearchMethod (clusters_tree);

	// Cluster potential stable regions

	cluster.setInputCloud (pcl_in_);
	cluster.setIndices (color_indices_[j]);
	cluster.extract (stable_color_regions_indexes_[j]);

      }
    }
  
    *cluster_image_ = Mat::zeros(hue_image_->size(), CV_8UC1);
      
    //array of clusters
    action_msgs::GaussianPointAttributeDistributionArray dist_array;
    visualization_msgs::MarkerArray marker_array;

    //fill in header
    dist_array.header.frame_id = base_frame_;
    dist_array.header.stamp = ros::Time::now();
    dist_array.feature_name = feature_name_;
    dist_array.feature_type = feature_type_;

    //fill in attributes
    //for (std::map<std::string, std::vector<double> >::iterator attrib = attributes_.begin(); attrib != attributes_.end(); attrib++){
    //  action_msgs::Attribute temp_attribute;
    //  temp_attribute.name = attrib->first;
    //  temp_attribute.values = attrib->second;
    //  dist_array.attributes.push_back(temp_attribute);
    //}

    int total_count = 0;

    for(mean_color_ = (int)attribute_mins_["hue"][0], 
	  j = 0, 
	  window_size_ = (int)attribute_windows_["hue"][0]; 
	
	((mean_color_ < (int)attribute_maxes_["hue"][0]) && 
	 (j <(int)binary_images_.size())); 
	
	j++, mean_color_ += (int)attribute_steps_["hue"][0]){

      int count = 0;

      for (std::vector<pcl::PointIndices>::const_iterator stable_color_region_index = stable_color_regions_indexes_[j].begin(); stable_color_region_index != stable_color_regions_indexes_[j].end(); stable_color_region_index++){

	//std::cout << "cluster: " << count << " size: " << stable_color_region_index->indices.size() << std::endl;
    	if(count <= max_num_features_) {

	  if(stable_color_region_index->indices.size() > 0){
	    Eigen::Vector4f centroid;
	    pcl::compute3DCentroid<pcl::PointXYZRGB> (*pcl_in_, stable_color_region_index->indices, centroid);
	    Eigen::Matrix3f covariance_matrix;
	    pcl::computeCovarianceMatrixNormalized<pcl::PointXYZRGB> (*pcl_in_, stable_color_region_index->indices, centroid, covariance_matrix);

	    //std::cout << "Covariance: " << covariance_matrix << std::endl;

	    if (display_image_ && (display_hue_ == mean_color_)){
	      for (std::vector<int>::const_iterator pit = stable_color_region_index->indices.begin(); pit != stable_color_region_index->indices.end(); pit++){
		//std::cout << "index: " << (*pit) << std::endl;
		//std::cout << "x: " << (*pit)%(pcl_in_->width) << " y: " << (*pit)/(pcl_in_->width) << std::endl;
		//std::cout << "img val: " << ((int)threshold_image_->at<uchar>((*pit)%(pcl_in_->width), (*pit)/(pcl_in_->width))) << std::endl;
		cluster_image_->at<uchar>((*pit)/(pcl_in_->width), (*pit)%(pcl_in_->width)) = ((uchar)255); 
	      }
	    }

	    action_msgs::GaussianPointAttributeDistribution gaussian_point_dist;
	    gaussian_point_dist.point_distribution.mean.x = centroid(0); 
	    gaussian_point_dist.point_distribution.mean.y = centroid(1);
	    gaussian_point_dist.point_distribution.mean.z = centroid(2); 

	    gaussian_point_dist.point_distribution.covariance.cxx = covariance_matrix(0,0); 
	    gaussian_point_dist.point_distribution.covariance.cxy = covariance_matrix(0,1); 
	    gaussian_point_dist.point_distribution.covariance.cxz = covariance_matrix(0,2); 
	    gaussian_point_dist.point_distribution.covariance.cyy = covariance_matrix(1,1); 
	    gaussian_point_dist.point_distribution.covariance.cyz = covariance_matrix(1,2); 
	    gaussian_point_dist.point_distribution.covariance.czz = covariance_matrix(2,2); 

	    //get rid of any infinity or NaN measurements
	    if(std::isfinite(centroid(0)) &&  std::isfinite(centroid(1)) && std::isfinite(centroid(2)) &&
	       std::isfinite(covariance_matrix(0,0)) &&  std::isfinite(covariance_matrix(0,1)) && 
	       std::isfinite(covariance_matrix(0,2)) &&  std::isfinite(covariance_matrix(1,1)) &&  
	       std::isfinite(covariance_matrix(1,2)) &&  std::isfinite(covariance_matrix(2,2))){

	      action_msgs::GaussianAttributeDistribution temp_attribute;
	      temp_attribute.attribute_name = "hue";
	      temp_attribute.mean.push_back((double)mean_color_);
	      temp_attribute.covariance.push_back(std::pow((double)attribute_windows_["hue"][0], 2.0));
	      gaussian_point_dist.attribute_distributions.push_back(temp_attribute);
	      
	      dist_array.distributions.push_back(gaussian_point_dist);
	      
	      if(publish_markers_){
		visualization_msgs::Marker h = getMarker(mean_color_, total_count, centroid, covariance_matrix);
		marker_array.markers.push_back(h);
	      }
	    }

	    count++;
	    total_count++;
	
	    if (display_image_ && (display_hue_ == mean_color_))
	      binary_images_[j]->copyTo(*(binary_images_[j]), *cluster_image_);
	    
	  }
	}      
      }
    }

    if (display_image_){
      *threshold_image_ = Mat::zeros(hue_image_->size(), CV_8UC3);
      rgb_image_->copyTo(*threshold_image_, *(binary_images_[(display_hue_ - (int)attribute_mins_["hue"][0])]));	  
      imshow(window_name_, *threshold_image_);
    }

    moments_pub_.publish(dist_array);
    if(publish_markers_) blob_marker_pub_.publish(marker_array);
    new_data_ = false;
  }
}

visualization_msgs::Marker MultipleHueDetector3DMonitor::getMarker(int mean_color_, int marker_id, Eigen::Vector4f centroid, Eigen::Matrix3f covariance_matrix){

  //------- Compute Principal Componets for Marker publishing


  //Get the principal components and the quaternion
  Eigen::Matrix3f evecs;
  Eigen::Vector3f evals;
  //pcl::eigen33 (covariance_matrix, evecs, evals);
  Eigen::EigenSolver<Eigen::Matrix3f> es(covariance_matrix);
	
  evecs = es.eigenvectors().real();
  evals = es.eigenvalues().real();
	    
  Eigen::Matrix3f rotation;
  rotation.row (0) = evecs.col (0);
  rotation.row (1) = evecs.col (1);
  rotation.row (2) = rotation.row (0).cross (rotation.row (1));
	    
  rotation.transposeInPlace ();
  Eigen::Quaternion<float> qt (rotation);
  qt.normalize ();
	    
  //Publish Marker for cluster
  visualization_msgs::Marker marker;	
		
  marker.header.frame_id = base_frame_;
  marker.header.stamp = ros::Time().now();
  marker.ns = "Perception";
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = marker_id;	
  marker.lifetime = ros::Duration(1);	
		
  //centroid position
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.pose.position.x = centroid[0];
  marker.pose.position.y = centroid[1];
  marker.pose.position.z = centroid[2];	
  marker.pose.orientation.x = qt.x();
  marker.pose.orientation.y = qt.y();
  marker.pose.orientation.z = qt.z();
  marker.pose.orientation.w = qt.w();			

  //std::cout << "e1: " << evals(0) << " e2: " << evals(1) << " e3: " << evals(2) << std::endl;

  marker.scale.x = sqrt(evals(0)) * 4;
  marker.scale.y = sqrt(evals(1)) * 4;
  marker.scale.z = sqrt(evals(2)) * 4;

  if(marker.scale.x == 0)
    marker.scale.x = .01;

  if(marker.scale.y == 0)
    marker.scale.y = .01;

  if(marker.scale.z == 0)
    marker.scale.z = .01;
	

  //give it some color!
  marker.color.a = 0.5;
  hueToRGB(mean_color_, marker.color.r, marker.color.g, marker.color.b);

  //std::cout << "marker being published" << std::endl;

  return marker;
}

void MultipleHueDetector3DMonitor::hueToRGB(int mean_color_, float &r, float &g, float &b){

  mean_color_ += 1;

  if ((2.0 * (float)mean_color_) < 120.0) {
    r = (120.0 - (2.0 * (float)mean_color_)) / 120.0;
    g = (2.0 * (float)mean_color_) / 120.0;
    b = 0;
  } 
  else if ((2.0 * (float)mean_color_) < 240.0) {
    r = 0;
    g = (240.0 - (2.0 * (float)mean_color_)) / 120.0;
    b = ((2.0 * (float)mean_color_) - 120.0) / 120.0;
  } 
  else {
    r = ((2.0 *(float)mean_color_) - 240.0) / 120.0;
    g = 0;
    b = (360.0 - (2.0 * (float)mean_color_)) / 120.0;
  }

  float max_val = std::max( std::max(r,g) , b);
  if(max_val != 0){
    r /= max_val;
    g /= max_val;
    b /= max_val;
  }

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "hue_detector_monitor");
  ros::NodeHandle nh("~");
  
  MultipleHueDetector3DMonitor node(ros::this_node::getName(), nh);
  ros::spin();
    
  return 0;
}
  
