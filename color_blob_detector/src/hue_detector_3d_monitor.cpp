#include "color_blob_detector/hue_detector_3d_monitor.h"

namespace enc = sensor_msgs::image_encodings;
using namespace cv;

HueDetector3DMonitor::HueDetector3DMonitor(std::string node_name, ros::NodeHandle nh){

  nh_ = nh;

  nh_.param<std::string>("feature_type", feature_type_, DEFAULT_FEATURE_TYPE);
  nh_.param<std::string>("feature_name", feature_name_, DEFAULT_FEATURE_NAME);
  nh_.param<int>("mean_color", mean_color_, DEFAULT_MEAN_COLOR);
  nh_.param<int>("window_size", window_size_, DEFAULT_WINDOW_SIZE);
  nh_.param<int>("blur_kernel", blur_kernel_, DEFAULT_BLUR_KERNEL);
  nh_.param<int>("max_cluster_size", max_cluster_size_, DEFAULT_MAX_CLUSTER_SIZE);
  nh_.param<int>("min_cluster_size", min_cluster_size_, DEFAULT_MIN_CLUSTER_SIZE);
  nh_.param<double>("within_cluster_distance", within_cluster_distance_, DEFAULT_WITHIN_CLUSTER_DISTANCE);
  nh_.param<double>("cluster_window", cluster_window_, DEFAULT_CLUSTER_WINDOW);
  nh_.param<bool>("display_image", display_image_, DEFAULT_DISPLAY_IMAGE);
  nh_.param<bool>("publish_markers", publish_markers_, DEFAULT_PUBLISH_MARKERS);
  nh_.param<int>("max_num_features", max_num_features_, DEFAULT_MAX_NUM_FEATURES);
  nh_.param<int>("update_rate", update_rate_, DEFAULT_UPDATE_RATE);
  nh_.param<std::string>("source_identifier", source_identifier_, DEFAULT_SOURCE_IDENTIFIER);
  nh_.param<std::string>("output_identifier", output_identifier_, DEFAULT_OUTPUT_IDENTIFIER);
  nh_.param<std::string>("marker_topic", marker_topic_, DEFAULT_MARKER_TOPIC);
  nh_.param<std::string>("base_frame", base_frame_, DEFAULT_BASE_FRAME);

  //create a map of any int or double parameter values
  parameter_map_.insert(make_pair(std::string("mean_color"), (double)mean_color_));
  parameter_map_.insert(make_pair(std::string("window_size"), (double)window_size_));
  parameter_map_.insert(make_pair(std::string("blur_kernel"), (double)blur_kernel_));
  parameter_map_.insert(make_pair(std::string("max_cluster_size"), (double)max_cluster_size_));
  parameter_map_.insert(make_pair(std::string("min_cluster_size"), (double)min_cluster_size_));
  parameter_map_.insert(make_pair(std::string("within_cluster_distance"), (double)within_cluster_distance_));
  parameter_map_.insert(make_pair(std::string("cluster_window"), (double)cluster_window_));
  parameter_map_.insert(make_pair(std::string("display_image"), (double)display_image_));
  parameter_map_.insert(make_pair(std::string("publish_markers"), (double)publish_markers_));
  parameter_map_.insert(make_pair(std::string("max_num_features"), (double)max_num_features_));
  parameter_map_.insert(make_pair(std::string("update_rate"), (double)update_rate_));

  //get attributes (optional)
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


  //next get attribute values
  XmlRpc::XmlRpcValue attribute_dict;
  if(nh_.getParam("attribute_values", attribute_dict)){
    ROS_ASSERT(attribute_dict.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      
    for (std::vector<std::string>::const_iterator att_name = att_names.begin(); att_name != att_names.end(); att_name++){
      
      if (attribute_dict.hasMember(*att_name)){
	XmlRpc::XmlRpcValue att_value_list = attribute_dict[*att_name];
	ROS_ASSERT(att_value_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
	std::vector <double> att_vals;
	for (int i = 0; i < (int)att_value_list.size(); i++) 
	  {
	    ROS_ASSERT(att_value_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
	    std::string temp_val = static_cast<std::string>(att_value_list[i]);
	    ROS_INFO("attribute name: %s, with value from param: %s", att_name->c_str(), temp_val.c_str());
	    if(parameter_map_.find(temp_val) != parameter_map_.end()){
	      att_vals.push_back(parameter_map_[temp_val]);
	      ROS_INFO("value: %4.3f", parameter_map_[temp_val]);
	    }
	    else
	      ROS_ERROR("NO value found for attribute %s", temp_val.c_str());
	  }
	attributes_.insert(make_pair(*att_name, att_vals));
      }
      else
	ROS_ERROR("attribute_values parameter not specified correctly."); 
    }
  }


  new_data_ = false;
  updated_ = true;

  threshold_image_.reset(new Mat);
  rgb_image_.reset(new Mat);
  cluster_image_.reset(new Mat);
  binary_image_.reset(new Mat);
  //binary_image2_.reset(new Mat);
  hue_image_.reset(new Mat);
  pcl_in_.reset(new PointCloudRGB);
  color_indices_.reset(new pcl::PointIndices);

  within_cluster_max_ = .2;   //maximum we would ever like the clusters to be
  within_cluster_scale_ = .001;   //scale for cluster distance ( 1mm )
  within_cluster_scaled_ = (int)(within_cluster_distance_ / within_cluster_scale_);

  node_name_ = node_name;
  window_name_ = node_name_ + " Window";

  if (display_image_) {
    namedWindow(window_name_, CV_WINDOW_NORMAL || CV_WINDOW_KEEPRATIO);
    //namedWindow(window_name_, CV_WINDOW_AUTOSIZE);
    createTrackbar("Mean Color", window_name_, &mean_color_, 179, NULL);
    createTrackbar("Blur Kernel", window_name_, &blur_kernel_, 25, NULL);
    createTrackbar("Window Size", window_name_, &window_size_, 40, NULL);
    createTrackbar("Cluster Distance", window_name_, &within_cluster_scaled_, (int)(within_cluster_max_ / within_cluster_scale_), NULL, this); 
    
    //necessary to have time to initialize display window correctly
    waitKey(50);
  }
   
  pointcloud_sub_ = nh_.subscribe(source_identifier_, 1, &HueDetector3DMonitor::pointCloudCallback, this);
  blob_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_,1);
  moments_pub_ = nh_.advertise<action_msgs::GaussianPointDistributionArray>(output_identifier_,1);
  timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_), &HueDetector3DMonitor::update, this);
  //register the goal and feedback callbacks
  ROS_INFO("Hue Monitor %s started", node_name_.c_str());
}

void HueDetector3DMonitor::getPointIndices()
{

  Size s = threshold_image_->size();

  //take the points that are non-zero in threshold_image_ and add them as point_indices
  color_indices_->indices.clear();

  for(int i = 0; i < s.height; i++) {
    for(int j = 0; j < s.width; j++) {
      //std::cout << " i: " << i << " j: " << j << std::endl;
      //std::cout << "point cloud val: " << pcl_in_->at(j,i) << std::endl;
      //std::cout << "image val: " << (int)(threshold_image_->at <uchar> (i, j))  << std::endl;
      if((int)(binary_image_->at <uchar> (i, j)))
	{ 
      	  color_indices_->indices.push_back(i * s.width + j);
	}
    }
  }
}

void HueDetector3DMonitor::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
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

    updated_ = false;

  }
}

void HueDetector3DMonitor::update(const ros::TimerEvent& e) {
  if(new_data_ && !updated_){

    updated_ = true;

    int rangeMin = mean_color_ - window_size_;
    int rangeMax = mean_color_ + window_size_;
	    
    Size ksize = Size(2 * blur_kernel_ + 1,2 * blur_kernel_ + 1);
    GaussianBlur(*hue_image_, *hue_image_, ksize, -1, -1);
    //blur(*hue_image_, *hue_image_, ksize, Point(-1, -1), BORDER_REPLICATE);
    //dilate(*hue_image_, *hue_image_, Mat(), Point(-1, -1), BORDER_REPLICATE);	    
    
    *binary_image_ = Mat::zeros(hue_image_->size(), CV_8UC1);
    //*binary_image2_ = Mat::zeros(hue_image_->size(), CV_8UC1);

    if((rangeMin >= 0) && (rangeMax <= 179)){
      inRange(*hue_image_, Scalar((double)((uchar)rangeMin)),Scalar((double)((uchar)rangeMax)), *binary_image_);
      //inRange(*hue_image_, Scalar((double)((uchar)rangeMin)),Scalar((double)((uchar)rangeMax)), *binary_image2_);
    }
    else if (rangeMin >= 0){
      Mat temp_binary_image1(hue_image_->size(), CV_8UC1);
      Mat temp_binary_image2(hue_image_->size(), CV_8UC1);

      inRange(*hue_image_, Scalar((double)((uchar)rangeMin)),Scalar((double)((uchar)179)), temp_binary_image1);
      inRange(*hue_image_, Scalar((double)((uchar)0)),Scalar((double)((uchar)(rangeMax - 179))), temp_binary_image2);
      bitwise_or(temp_binary_image1, temp_binary_image2, *binary_image_);
      //bitwise_or(temp_binary_image1, temp_binary_image2, *binary_image2_);
    }
    else if (rangeMax <= 179){
      Mat temp_binary_image1(hue_image_->size(), CV_8UC1);
      Mat temp_binary_image2(hue_image_->size(), CV_8UC1);

      inRange(*hue_image_, Scalar((double)((uchar)0)),Scalar((double)((uchar)rangeMax)), temp_binary_image1);
      inRange(*hue_image_, Scalar((double)((uchar)(179 + rangeMin))),Scalar((double)((uchar)179)), temp_binary_image2);
      bitwise_or(temp_binary_image1, temp_binary_image2, *binary_image_);
      //bitwise_or(temp_binary_image1, temp_binary_image2, *binary_image2_);
    }
    else {
      ROS_ERROR("Window size parameter is too large.  Decrease it!");
      return;
    }  

    *threshold_image_ = Mat::zeros(hue_image_->size(), CV_8UC3);
    rgb_image_->copyTo(*threshold_image_, *binary_image_);

    getPointIndices();
    
    if(color_indices_->indices.size() > 0){
    
      pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> cluster;	
      stable_color_regions_indexes_.clear();
      pcl::KdTree<pcl::PointXYZRGB>::Ptr clusters_tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZRGB> > ();

      cluster.setClusterTolerance (within_cluster_distance_);
      cluster.setMinClusterSize (min_cluster_size_);
      cluster.setMaxClusterSize (max_cluster_size_);
      cluster.setSearchMethod (clusters_tree);

      // Cluster potential stable regions

      cluster.setInputCloud (pcl_in_);
      cluster.setIndices (color_indices_);
      cluster.extract (stable_color_regions_indexes_);
     
      *cluster_image_ = Mat::zeros(hue_image_->size(), CV_8UC1);
      
      //array of clusters
      action_msgs::GaussianPointDistributionArray dist_array;
      visualization_msgs::MarkerArray marker_array;

      //fill in header
      dist_array.header.frame_id = base_frame_;
      dist_array.header.stamp = ros::Time::now();
      dist_array.feature_name = feature_name_;
      dist_array.feature_type = feature_type_;

      //fill in attributes
      for (std::map<std::string, std::vector<double> >::iterator attrib = attributes_.begin(); attrib != attributes_.end(); attrib++){
	action_msgs::Attribute temp_attribute;
	temp_attribute.name = attrib->first;
	temp_attribute.values = attrib->second;
	dist_array.attributes.push_back(temp_attribute);
      }

      int count = 0;
      for (std::vector<pcl::PointIndices>::const_iterator stable_color_region_index = stable_color_regions_indexes_.begin(); stable_color_region_index != stable_color_regions_indexes_.end(); stable_color_region_index++){

	//std::cout << "cluster: " << count << " size: " << stable_color_region_index->indices.size() << std::endl;

	if(count <= max_num_features_) {

	  if(stable_color_region_index->indices.size() > 0){
	    Eigen::Vector4f centroid;
	    pcl::compute3DCentroid<pcl::PointXYZRGB> (*pcl_in_, stable_color_region_index->indices, centroid);
	    Eigen::Matrix3f covariance_matrix;
	    pcl::computeCovarianceMatrixNormalized<pcl::PointXYZRGB> (*pcl_in_, stable_color_region_index->indices, centroid, covariance_matrix);

	    //std::cout << "Covariance: " << covariance_matrix << std::endl;

	    if (display_image_){
	      for (std::vector<int>::const_iterator pit = stable_color_region_index->indices.begin(); pit != stable_color_region_index->indices.end(); pit++){
		//std::cout << "index: " << (*pit) << std::endl;
		//std::cout << "x: " << (*pit)%(pcl_in_->width) << " y: " << (*pit)/(pcl_in_->width) << std::endl;
		//std::cout << "img val: " << ((int)threshold_image_->at<uchar>((*pit)%(pcl_in_->width), (*pit)/(pcl_in_->width))) << std::endl;
		cluster_image_->at<uchar>((*pit)/(pcl_in_->width), (*pit)%(pcl_in_->width)) = ((uchar)255); 
	      }
	    }

	    action_msgs::GaussianPointDistribution gaussian_point_dist;
	    gaussian_point_dist.mean.x = centroid(0); 
	    gaussian_point_dist.mean.y = centroid(1);
	    gaussian_point_dist.mean.z = centroid(2); 

	    gaussian_point_dist.covariance.cxx = covariance_matrix(0,0); 
	    gaussian_point_dist.covariance.cxy = covariance_matrix(0,1); 
	    gaussian_point_dist.covariance.cxz = covariance_matrix(0,2); 
	    gaussian_point_dist.covariance.cyy = covariance_matrix(1,1); 
	    gaussian_point_dist.covariance.cyz = covariance_matrix(1,2); 
	    gaussian_point_dist.covariance.czz = covariance_matrix(2,2); 

	    dist_array.distributions.push_back(gaussian_point_dist);
	  
	    if(publish_markers_){
	      visualization_msgs::Marker h = getMarker(count, centroid, covariance_matrix);
	      marker_array.markers.push_back(h);
	    }

	    count++;
	
	    if (display_image_)
	      binary_image_->copyTo(*binary_image_, *cluster_image_);
		  
	  }
	}
      }
      

      if (display_image_){
	*threshold_image_ = Mat::zeros(hue_image_->size(), CV_8UC3);
	rgb_image_->copyTo(*threshold_image_, *binary_image_);	  
	imshow(window_name_, *threshold_image_);
	//imshow(window_name_, *binary_image_);
      }

      moments_pub_.publish(dist_array);
      if(publish_markers_) blob_marker_pub_.publish(marker_array);
    }
    new_data_ = false;
  }   
}

visualization_msgs::Marker HueDetector3DMonitor::getMarker(int marker_id, Eigen::Vector4f centroid, Eigen::Matrix3f covariance_matrix){

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
	

  //give it some color!
  marker.color.a = 0.75;
  hueToRGB(marker.color.r, marker.color.g, marker.color.b);

  //std::cout << "marker being published" << std::endl;

  return marker;
}

void HueDetector3DMonitor::hueToRGB(float &r, float &g, float &b){

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
  
  HueDetector3DMonitor node(ros::this_node::getName(), nh);
  ros::spin();
    
  return 0;
}
  
