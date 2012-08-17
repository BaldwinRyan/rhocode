
#include <iostream>
#include <vector>
#include <string>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <ros/ros.h>
#include <ros/init.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <action_msgs/GaussianPointDistributionArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/features/feature.h>
#include <pcl/ros/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>

//#define DEBUG

const std::string DEFAULT_BASE_FRAME = "/baseLink";
//const std::string DEFAULT_BASE_FRAME = "/openni_rgb_optical_frame";
const std::string DEFAULT_SOURCE_IDENTIFIER = "/kinect/camera/rgb/filtered_points";
const std::string DEFAULT_OUTPUT_IDENTIFIER = "/kinect/camera/rgb/feature/hue/some_hue";
const std::string DEFAULT_MARKER_TOPIC = "/kinect/camera/rgb/feature/hue/some_hue/marker";

const std::string DEFAULT_FEATURE_TYPE = "color";
const std::string DEFAULT_FEATURE_NAME = "red";

const int DEFAULT_UPDATE_RATE = 50;

const int DEFAULT_MAX_NUM_FEATURES = 20;
const int DEFAULT_MIN_CLUSTER_SIZE = 100;
const int DEFAULT_MAX_CLUSTER_SIZE = 10000;
const double DEFAULT_WITHIN_CLUSTER_DISTANCE = .01;
const double DEFAULT_CLUSTER_WINDOW = .001;
const int DEFAULT_MEAN_COLOR = 175;    //red-ish
const int DEFAULT_WINDOW_SIZE = 15;
const int DEFAULT_BLUR_KERNEL = 1;
const bool DEFAULT_DISPLAY_IMAGE = true;
const bool DEFAULT_PUBLISH_MARKERS = true;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

class ConnectedComp {	//class for connected components
public:
  ConnectedComp() {
    rank = 0;		     							
    area = 0;			
    idx = -1;
  };

  ~ConnectedComp() {
  };
	
  ConnectedComp(const ConnectedComp& source) 
  {
    idx = source.idx;
    rank = source.rank;
    area = source.area;
  };

  ConnectedComp& operator = (const ConnectedComp& source)
  {
    if (this != &source)
      {
	rank = source.rank;
	area = source.area;
	idx = source.idx;
      }
    return *this;
  };

  int rank;	       //rank of blob
  double area;         //area of blob
  int idx;	       //contour idx

};

class HueDetector3DMonitor {
protected:
  
  std::map<std::string, double> parameter_map_;

  ros::NodeHandle nh_;
  std::string action_name_;
  ros::Timer timer_;

  ros::Subscriber pointcloud_sub_;
  ros::Publisher moments_pub_;
  ros::Publisher blob_marker_pub_;

  std::string marker_topic_;                          //name for marker topic
  std::string output_identifier_;                     //name for output topic
  std::string source_identifier_;                     //name for input topic

  //these strings must match those given in the ACDF
  std::string feature_type_;
  std::string feature_name_;

  std::vector<pcl::PointIndices> stable_color_regions_indexes_;
  cv_bridge::CvImagePtr rgb_image_ptr_;                            //boost::shared_ptr
  std::map<std::string, std::vector<double> > attributes_;

  int update_rate_;
  int mean_color_;
  int blur_kernel_;        //the size of the Gaussian kernel
  int window_size_;
  int max_num_features_;   //the maximum number of features for this signal
  int width_;              //the width and height of the input image
  int height_;
  bool display_image_;     //true => display image
  bool publish_markers_;     //true => publish markers
  bool new_data_;          //whether or not there is new data 
  bool updated_;           //whether or not new data has been processed
  int max_cluster_size_;
  int min_cluster_size_;
  std::string node_name_;
  std::string window_name_;
  std::string base_frame_;
  

  boost::shared_ptr<cv::Mat> threshold_image_;        // thresholded color image
  boost::shared_ptr<cv::Mat> hue_image_;	      // input image -> hue space
  boost::shared_ptr<cv::Mat> rgb_image_;	      // input image -> rgb space
  boost::shared_ptr<cv::Mat> binary_image_;	      // input image -> cluster + color space
  boost::shared_ptr<cv::Mat> binary_image2_;	      // input image -> cluster + color space
  boost::shared_ptr<cv::Mat> cluster_image_;	      // input image -> cluster + color space
  boost::shared_ptr<pcl::PointIndices> color_indices_;   
  boost::shared_ptr<PointCloudRGB> pcl_in_;   

public:

  HueDetector3DMonitor(std::string node_name, ros::NodeHandle nh);

  ~HueDetector3DMonitor() {
    this->clear();
  };
  
  void clear(){
    stable_color_regions_indexes_.clear();
    rgb_image_ptr_.reset();
    threshold_image_.reset();
    hue_image_.reset();
    rgb_image_.reset();
    binary_image_.reset();
    cluster_image_.reset();
    color_indices_.reset();
    pcl_in_.reset();
    source_identifier_.clear();
    output_identifier_.clear();
    node_name_.clear();
    base_frame_.clear();
    window_name_.clear();
  };

  void getPointIndices();
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void update(const ros::TimerEvent& e);

  visualization_msgs::Marker getMarker(int marker_id, Eigen::Vector4f centroid, Eigen::Matrix3f covariance_matrix);
  void hueToRGB(float &r, float &g, float &b);

  double within_cluster_distance_; // distance between points within a cluster
  double cluster_window_; // distance between points within a cluster
  double within_cluster_scale_;
  double within_cluster_max_;
  int within_cluster_scaled_; 
};
