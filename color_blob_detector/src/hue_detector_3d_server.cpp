
#include <ros/ros.h>
#include <ros/init.h>
#include <pcl/features/feature.h>
#include <Image_msgs/FeatureMoments3D.h>
#include <feature_tracker/feature_blob_tracker.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <affordance_msgs/feature_detector_searchAction.h>
#include <affordance_msgs/track.h>
#include <actionlib/server/simple_action_server.h>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <time.h>
#include <stdlib.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
//#include <pcl/features/feature.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

//#define DEBUG

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

#define MAX_FEATURES 10
#define DEFAULT_MIN_CC_THRESHOLD 150
#define DEFAULT_MAX_CC_THRESHOLD 1000
#define DEFAULT_SOURCE_IDENTIFIER "/kinect/camera/rgb/points"
#define DEFAULT_WIDTH 320
#define DEFAULT_HEIGHT 240
//how often per seconds
#define update_rate 5

//#define DEFAULT_NAME "TestColor"

const double PI = 3.141592;

int init_mean_color = 80;
int init_window_size = 10;
int init_blur_kernel = 5;
bool init_display_image = false;

int mean_color = init_mean_color;    //the color being search for +/- (window_size/2)
int colorIndex = 0;
int window_size = init_window_size;	 
int minCCThreshold = DEFAULT_MIN_CC_THRESHOLD;	//the minimum area (in pixels) for a connected component
int maxCCThreshold = DEFAULT_MAX_CC_THRESHOLD;	//the maximum area (in pixels) for a connected component
int maxNumComponents = MAX_FEATURES;  //the maximum number of connected components
int blurKernel = init_blur_kernel;    //the size of the Gaussian kernel
int width = DEFAULT_WIDTH;     //the width and height of the input image
int height = DEFAULT_HEIGHT;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

void on_trackbar(int h, void *f) {
}

class ConnectedComp {	//class for connected components
public:
  ConnectedComp() {
    rank = 0;		     							
    area = 0;			
    idx = -1;
  }

  ~ConnectedComp() {
  }
	
  ConnectedComp(const ConnectedComp& source) 
  {
    idx = source.idx;
    rank = source.rank;
    area = source.area;
  }

  ConnectedComp& operator = (const ConnectedComp& source)
  {
    if (this != &source)
      {
	rank = source.rank;
	area = source.area;
	idx = source.idx;
      }
    return *this;
  }

  int rank;	       //rank of blob
  double area;         //area of blob
  int idx;	       //contour idx

};

class HueDetector3DServer {
protected:
  
  ros::NodeHandle n_;
  actionlib::SimpleActionServer<affordance_msgs::feature_detector_searchAction>	server_;
  std::string action_name_;
  ros::Timer timer_;

  affordance_msgs::feature_detector_searchGoal goal_;
  affordance_msgs::feature_detector_searchResult result_;
  affordance_msgs::feature_detector_searchFeedback feedback_;

  ros::Subscriber pointcloud_sub_;
  //ros::Publisher moments_pub_;
  ros::Publisher object_pose_marker_pub_;
  cv_bridge::CvImagePtr ptr_rgb;

  bool first;				//first = true => initialize vars
  char color_topic[100];		//name for output topic
  char pointcloud_topic[100];		//name for input topic
  char marker_topic[100];

  bool display_image;
  vector<int> numCC;				     //the number of valid connected components

  vector<vector <ConnectedComp> > comps;		//array of connected components
  vector<vector <vector <Point> > > contours;	       //storage for linked list contours
  vector<vector <Vec4i> > hierarchy;

  vector<FeatureBlobTracker> blobTracker;	 //Kalman filter and storage for blobs (across timesteps)
  Mat *flipped;				//image flipped vertically
  Mat* back_img;                        //Binary image
  Mat *temp_blurred_image; 
  Mat *blurred_image;			//blurred images 
  Mat* hue_image;		       //input image -> hue space
  Mat* rgb_image;	 		// rgb image from cameras
  Mat* hsv_image;		        // hsv image from cameras
 
  sensor_msgs::Image img;
  PointCloudRGB pcl_in;
  bool is_running;
  std::string name;
  
  //Image_msgs::FeatureMoments3D moments;   //Message for publishing moments
  char *window_name; //name of the feature
  bool got_goals;
  int num_feature_goals_;
  vector <double> feature_mean_goals_;
  vector <double> feature_var_goals_;

public:
  
  HueDetector3DServer(std::string s_name) :
    server_(n_, s_name), action_name_(s_name) {
    
    first = true;
    
    if (!n_.getParam("HueServer/maxNumComponents", maxNumComponents)) {
      ROS_INFO("Could not retrieve 'maxNumComponents' parameter, setting to default value of '%d'", MAX_FEATURES);
      maxNumComponents = MAX_FEATURES;
    }

    if (!n_.getParam("HueServer/blurKernel", blurKernel)) {
      ROS_INFO("Could not retrieve 'blurKernel' parameter, setting to default value of '%d'", init_blur_kernel);
      blurKernel = init_blur_kernel;
    }

    if (!n_.getParam("HueServer/minCCThreshold", minCCThreshold)) {
      ROS_INFO("Could not retrieve 'minCCThreshold' parameter, setting to default value of '%d'", DEFAULT_MIN_CC_THRESHOLD);
      minCCThreshold = DEFAULT_MIN_CC_THRESHOLD;
    }

    if (!n_.getParam("HueServer/maxCCThreshold", maxCCThreshold)) {
      ROS_INFO("Could not retrieve 'maxCCThreshold' parameter, setting to default value of '%d'", DEFAULT_MAX_CC_THRESHOLD);
      maxCCThreshold = DEFAULT_MAX_CC_THRESHOLD;
    }
    
    if (!n_.getParam("HueServer/display_image", display_image)) {
      ROS_INFO("Could not retrieve 'display_image' parameter, setting to default value of '%d'", init_display_image);
      display_image = init_display_image;
    }	

    std::string source_identifier;
    if (!n_.getParam("HueServer/source_identifier", source_identifier)) {
      ROS_INFO("Could not retrieve 'source_identifier' parameter, setting to default value of '%s'", DEFAULT_SOURCE_IDENTIFIER);
      source_identifier = DEFAULT_SOURCE_IDENTIFIER;
    }
  
    std::stringstream hue_name;
    hue_name << name << "HueServer";
    sprintf(pointcloud_topic, "%s", source_identifier.c_str());	//input topic
    sprintf(color_topic, "%s/Feature/ColorBlobs/%s", source_identifier.c_str(), hue_name.str().c_str());		
    srand ( time(NULL) );

    //output topic
    sprintf(marker_topic,"ellipsoid_marker/%s",hue_name.str().c_str());
    printf("Topic name : %s\n", color_topic);
    pointcloud_sub_ = n_.subscribe(pointcloud_topic, 1, &HueDetector3DServer::pointCloudCallback, this);
    //moments_pub_ = n_.advertise<Image_msgs::FeatureMoments3D>(color_topic, 1);
    object_pose_marker_pub_ = n_.advertise<visualization_msgs::Marker>(marker_topic,1);
    
    if (display_image) {
      namedWindow(name, CV_WINDOW_AUTOSIZE);
      //namedWindow("feat", CV_WINDOW_AUTOSIZE);
      createTrackbar("Blur Kernel", name, &blurKernel, 25, &on_trackbar);
      createTrackbar("Min Size", name, &minCCThreshold, 3000, &on_trackbar); 
      createTrackbar("Max Size", name, &maxCCThreshold, 10000, &on_trackbar);
    }
   	
    hue_image = new Mat(height, width, CV_8UC1);
    back_img = new Mat(hue_image->size(), CV_8UC1);
    temp_blurred_image = new Mat(hue_image->size(), CV_8UC1);
    blurred_image = new Mat(hue_image->size(), CV_8UC1);
    flipped = new Mat(hue_image->size(), CV_8UC1);
    rgb_image = new Mat(hue_image->size(), CV_8UC3);
    hsv_image = new Mat(hue_image->size(), CV_8UC3);
    
    is_running = false;
    got_goals = false;
    num_feature_goals_ = 0;
    timer_ = n_.createTimer(ros::Duration(1.0 / update_rate), &HueDetector3DServer::Update, this);
    //register the goal and feeback callbacks
    server_.registerGoalCallback(boost::bind(&HueDetector3DServer::GoalCB, this));
    server_.registerPreemptCallback(boost::bind(&HueDetector3DServer::PreemptCB, this));
    ROS_INFO("Hue Server started ");
  }

  ~HueDetector3DServer() {
    hue_image->release();
    back_img->release();
    temp_blurred_image->release();
    blurred_image->release();
    flipped->release();
    rgb_image->release();

    //delete blobTracker;
  }

  void PreemptCB() {
    
    got_goals = false;
    server_.setPreempted(); 
  }

  void GoalCB() {
    ROS_INFO("%s: received new goal.", action_name_.c_str());
    
    blobTracker.clear();
    comps.clear();
    hierarchy.clear();
    contours.clear();
    numCC.clear();
    
    // accept the new goal
    goal_ = *server_.acceptNewGoal();
    
    if (goal_.numSearchGoals <= 0)
      return;

    num_feature_goals_ = goal_.numSearchGoals;
    feature_mean_goals_.clear();
    feature_var_goals_.clear();
    feature_mean_goals_.resize(num_feature_goals_);
    feature_var_goals_.resize(num_feature_goals_);
    numCC.resize(num_feature_goals_);

    comps = vector<vector<ConnectedComp> >();
    contours = vector<vector<vector<Point> > >();
    hierarchy = vector<vector<Vec4i> >();
    comps.resize(num_feature_goals_);
    contours.resize(num_feature_goals_);
    hierarchy.resize(num_feature_goals_);
    blobTracker.resize(num_feature_goals_);

    for(int i=0; i<num_feature_goals_; i++)
      {
	comps[i].resize(maxNumComponents);
	blobTracker[i] = FeatureBlobTracker(maxNumComponents, width, height);
	blobTracker[i].initialize();
	feature_mean_goals_[i] = goal_.signal[i].mean[0];
	feature_var_goals_[i] = goal_.signal[i].variance[0];
      }
    
    got_goals = true;
    ROS_INFO("%d features for color feature search received.", num_feature_goals_);
  }

  int getThresholdConnectedComponents(int index) {
    int count = 0;
    double area;
    double contour_areas[(int)contours[index].size()];
    for(int idx = 0; idx < (int)contours[index].size(); idx++){
      vector<Point> approx;
      vector<Point> current = contours[index][idx];
      Mat m(current);
      m = abs(m);
      approxPolyDP(m, approx, 3, true);
      area = fabs(contourArea(Mat(approx)));
      contour_areas[idx] = area;
#ifdef DEBUG
      cout << "contour " << idx << " area: " << contour_areas[idx] << endl;
#endif
    }

    for(count = 0; count < maxNumComponents;){
      int largestIdx = 0;
      int largestArea = 0;
      for(int idx = 0; idx < (int)contours[index].size(); idx++){
	if ((contour_areas[idx] > minCCThreshold) && (contour_areas[idx] > largestArea)){
	  largestArea = contour_areas[idx];
	  largestIdx = idx;		
	}
      }
      if(largestArea > minCCThreshold){
	comps[index][count].area = largestArea;
	comps[index][count].idx = largestIdx;
	//cout << "largest area " << largestArea << " idx: " << largestIdx << " count: " << count << endl;
	contour_areas[largestIdx] = 0;
	count++;
      }	
      else
	break;
    }
    for(int left = (count); left < maxNumComponents; left++){
      comps[index][left].area = -1;
      comps[index][left].idx = -1;
    }
#ifdef DEBUG
    cout << "Number of Connected Components " << count << endl;
#endif
    return (count);
  }

  void getConnectedComponents(int index) {
    findContours(*back_img, contours[index], hierarchy[index], CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE, Point());
    if(!contours[index].empty())
      numCC[index] = getThresholdConnectedComponents(index);
  }


  void getMoments(int index) {   
 
    blobTracker[index].reset();
   		
    Mat first(2,1,CV_32F);
    Mat second(2,2,CV_32F);
    for (int k = 0; (k < numCC[index]) && (k < (int)comps[index].size()); k++) {
      if((comps[index][k].idx < (int)contours[index].size()) && (comps[index][k].idx >= 0)){
	vector<Point> current = contours[index][comps[index][k].idx];
	vector<Point> approx;
	Mat m(current);
	m = abs(m);
	approxPolyDP(m, approx, 3, true);
			
	m = Mat(approx);
	m = abs(m);
	Moments moment = cv::moments(m);

	first.at<float>(0,0) = moment.m10/moment.m00;  //x mean
	first.at<float>(1,0) = moment.m01/moment.m00;	//y mean
	second.at<float>(0,0) = (moment.mu20/moment.m00);
	second.at<float>(0,1) = (moment.mu11/moment.m00);
	second.at<float>(1,0) = (moment.mu11/moment.m00);
	second.at<float>(1,1) = (moment.mu02/moment.m00);
	blobTracker[index].addNewMeasurement(first, second, k,"color",k);
	//Print the points
	//for(int i=0; i<current.size(); i++)
	//ROS_INFO("%d : %d --> %d,%d",k,i,current[i].x,current[i].y);
	get3DMoments(current, index, k);
      }
    }

    for (int k = numCC[index]; k < maxNumComponents; k++)
      {
	//set the other features invalid
	first.at<float>(0,0) = -1;
	first.at<float>(1,0) = -1;
	second.at<float>(0,0) = 1; second.at<float>(0,1) = 0; second.at<float>(1,0) = 0; second.at<float>(1,1) = 1;
	blobTracker[index].addNewMeasurement(first, second, k,"color",k);
      }
    
    blobTracker[index].matchRawDataToEstimates(false, numCC[index]);
  }

  void get3DMoments(vector<Point> feature_points, int feat_index, int index)
  {
    //    for(int i=0; i<feature_points.size(); i++)
    //   ROS_INFO("%d --> %d,%d",i,feature_points[i].x,feature_points[i].y);
    //ROS_INFO("Getting 3D Moments : %d --> %d,%d", feature_points.size(), width, height);
    
    //Extract the indices for the points in the point cloud data
    pcl::PointIndices point_indices;
     
    for(int i=0; i<feature_points.size(); i++)
      {
	//ROS_INFO("Feature Index : %d, %d",feature_points[i].x, feature_points[i].y);
	point_indices.indices.push_back(feature_points[i].y * width + feature_points[i].x);
      }
    
    //ROS_INFO("Computing 3D Centroid : %d",point_indices.indices.size());
    Eigen::Vector4f centroid;
    Eigen::Matrix3f covariance_matrix;
    
    // Estimate the XYZ centroid
    pcl::compute3DCentroid (pcl_in, point_indices, centroid); 
#ifdef DEBUG
    ROS_INFO("Centroid %d: %f, %f, %f, %f",index,centroid(0),centroid(1),centroid(2),centroid(3));
#endif

    //ROS_INFO("Computing Covariance ");
    //Compute the centroid and the covariance of the points
    computeCovarianceMatrix(pcl_in, point_indices.indices, centroid, covariance_matrix);
    
    //Print the 3D Moments
    //ROS_INFO("Centroid : %f, %f, %f, %f",centroid(0),centroid(1),centroid(2),centroid(3));
#ifdef DEBUG
    std::cout<<"Covariance : "<<std::endl<<covariance_matrix <<std::endl;
#endif

    for(int i=0; i<3; i++)
      {
	feedback_.features[feat_index].moments[index].mean[i] = centroid(i);
	for(int j=0; j<3; j++)
	  {
	    feedback_.features[feat_index].moments[index].covariance[i*3+j] = covariance_matrix(i,j);
	  }
      }

    //Get the principal components and the quaternion
    Eigen::Matrix3f evecs;
    Eigen::Vector3f evals;
    pcl::eigen33 (covariance_matrix, evecs, evals);
    
    Eigen::Matrix3f rotation;
    rotation.row (0) = evecs.col (0);
    rotation.row (1) = evecs.col (1);
    //rotation.row (2) = evecs.col (2);
    rotation.row (2) = rotation.row (0).cross (rotation.row (1));
    //rotation.transposeInPlace ();
#ifdef DEBUG
    std::cerr << "Rotation matrix: " << endl;
    std::cerr << rotation << endl;
    std::cout<<"Eigen vals : "<<evals<<std::endl;
#endif

    rotation.transposeInPlace ();
    Eigen::Quaternion<float> qt (rotation);
    qt.normalize ();

    //Publish Marker
    visualization_msgs::Marker marker;	
    
    marker.header.frame_id = "/openni_rgb_optical_frame";
    marker.header.stamp = ros::Time().now();
    marker.ns = "Triangulation";
    marker.id = index+1;	
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(5);		
    
    //centroid position
    marker.type = visualization_msgs::Marker::SPHERE;
    
    
    marker.pose.position.x = centroid(0);
    marker.pose.position.y = centroid(1);
    marker.pose.position.z = centroid(2);	
    marker.pose.orientation.x = qt.x();
    marker.pose.orientation.y = qt.y();
    marker.pose.orientation.z = qt.z();
    marker.pose.orientation.w = qt.w();			
    
    marker.scale.x = sqrt(evals(0)) *2;
    marker.scale.y =  sqrt(evals(1)) *2;
    marker.scale.z =  sqrt(evals(2)) *2;
    
    //red
    marker.color.a = 0.5;
    marker.color.r = rand()/((double)RAND_MAX + 1);
    marker.color.g = rand()/((double)RAND_MAX + 1);
    marker.color.b = rand()/((double)RAND_MAX + 1);
    object_pose_marker_pub_.publish(marker);	
    
  }

  void computeCovarianceMatrix(PointCloudRGB cloud, std::vector<int> indices, Eigen::Vector4f centroid, Eigen::Matrix3f &covariance_matrix)
  {
    // ROS_INFO("Inside computeCovarianceMatrix() ");
    // Initialize to 0
    covariance_matrix.setZero ();
    
    if (indices.empty ())
      return;
  
    // If the data is dense, we don't need to check for NaN
    if (cloud.is_dense)
      {
	// For each point in the cloud
	for (size_t i = 0; i < indices.size (); ++i)
	  {
	    //Eigen::Vector4f ptx = cloud.points[indices[i]].getVector4fMap ();
	    ///std::cout << "Index : "<< i <<" : "<<ptx<<std::endl;
	    Eigen::Vector4f pt = cloud.points[indices[i]].getVector4fMap () - centroid;
	    
	    covariance_matrix (0, 0) += pt.x () * pt.x ();
	    covariance_matrix (0, 1) += pt.x () * pt.y ();
	    covariance_matrix (0, 2) += pt.x () * pt.z ();
	    covariance_matrix (1, 1) += pt.y () * pt.y ();
	    covariance_matrix (1, 2) += pt.y () * pt.z ();	
	    covariance_matrix (2, 2) += pt.z () * pt.z ();
	  }

      }
    // NaN or Inf values could exist => check for them
    else
      {
	//std::cout<<"Cloud is not dense "<<std::endl;
	// For each point in the cloud
	for (size_t i = 0; i < indices.size (); ++i)
	  {
	    // Check if the point is invalid
	    if (!pcl_isfinite (cloud.points[indices[i]].x) || 
		!pcl_isfinite (cloud.points[indices[i]].y) || 
		!pcl_isfinite (cloud.points[indices[i]].z))
	      continue;
	    
	    // Eigen::Vector4f ptx = cloud.points[indices[i]].getVector4fMap ();
	    // std::cout << "Index : "<< i <<" : "<<ptx<<std::endl;
	    Eigen::Vector4f pt = cloud.points[indices[i]].getVector4fMap () - centroid;
	    
	    covariance_matrix (0, 0) += pt.x () * pt.x ();
	    covariance_matrix (0, 1) += pt.x () * pt.y ();
	    covariance_matrix (0, 2) += pt.x () * pt.z ();
	    covariance_matrix (1, 1) += pt.y () * pt.y ();
	    covariance_matrix (1, 2) += pt.y () * pt.z ();	
	    covariance_matrix (2, 2) += pt.z () * pt.z ();
	    covariance_matrix (1, 1) += pt.y () * pt.y ();
	  }
      }
    covariance_matrix (1, 0) = covariance_matrix (0, 1);
    covariance_matrix (2, 0) = covariance_matrix (0, 2);
    covariance_matrix (2, 1) = covariance_matrix (1, 2);
    
    covariance_matrix /= indices.size();
  }

 
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    if(is_running)
      return;
    
    //numCC = 0;
    is_running = true;
    pcl::fromROSMsg (*msg, pcl_in);
    pcl::toROSMsg (pcl_in, img);
    ptr_rgb = cv_bridge::toCvCopy(img, "bgr8");
    ptr_rgb->image.copyTo(*rgb_image);


    //ROS_INFO("Inside point cloud callback");

    if(first)
      {
	waitKey(10);
	first = false;
      }

    cvtColor(*rgb_image, *hsv_image, CV_BGR2HSV);
    vector <Mat> planes;
    split(*hsv_image, planes);
    
    *hue_image = planes[0];
           
    //moments_pub_.publish(moments);
      
    waitKey(10);

    is_running = false;
  }

  void Update(const ros::TimerEvent& e) {
    if(!server_.isActive() ||  is_running)
      return;
    
    if(got_goals)
      {
	is_running = true;
	ROS_INFO("Update--> Got goals of size %d", num_feature_goals_);
	feedback_.features.resize(num_feature_goals_);
	feedback_.signal.resize(num_feature_goals_);
	//	feedback_.signal.resize(num_feature_goals_);

	for(int i=0; i<num_feature_goals_; i++)
	  {
	    feedback_.signal[i].signal_type = "hue";
	    feedback_.signal[i].mean.resize(1);
	    feedback_.signal[i].variance.resize(1);
	    feedback_.signal[i].mean[0] = feature_mean_goals_[i];
	    feedback_.signal[i].variance[0] = feature_var_goals_[i];
	    feedback_.signal[i].size = 1;

	    mean_color = feature_mean_goals_[i];
	    window_size = feature_var_goals_[i];
	    int rangeMin = ((mean_color - window_size + 255)%255);
	    int rangeMax = ((mean_color + window_size + 255)%255);
	    ROS_INFO("Range [%d] : %d,%d",i,rangeMin,rangeMax);
	    ROS_INFO("Threshold : %d,%d",minCCThreshold,maxCCThreshold);
	    
	    if(rangeMin > rangeMax){
	      int temp = rangeMax;
	      rangeMax = rangeMin;
	      rangeMin = temp;
	    }
	    
	    if(minCCThreshold >= maxCCThreshold){
	      //ROS_INFO("Min radius must be smaller than Max radius");
	      is_running = false;
	      return;
	    }
	    
	    if(fabs(rangeMin - rangeMax) <= 2*window_size){
	      //ROS_INFO("REG rangeMin %d rangeMax %d", rangeMin, rangeMax);
	      inRange(*hue_image, Scalar((double)((uchar)rangeMin)),Scalar((double)((uchar)rangeMax)), *back_img);
	    }
	    else if ((mean_color + window_size) > 255){
	      //ROS_INFO("BIG rangeMin %d rangeMax %d", rangeMin, rangeMax);
	      inRange(*hue_image, Scalar((double)((uchar)rangeMax)),Scalar((double)((uchar)255)), *back_img);
	    }
	    else {
	      //ROS_INFO("SML rangeMin %d rangeMax %d", rangeMin, rangeMax);
	      inRange(*hue_image, Scalar((double)((uchar)0)),Scalar((double)((uchar)rangeMin)), *back_img);
	    }  
	    
	    Size ksize = Size(2 * blurKernel + 1,2 * blurKernel + 1);
	    ROS_INFO("Adding Gaussian Blur");
	    GaussianBlur(*back_img, *blurred_image, ksize, -1, -1);
	    
	    ROS_INFO("Thresholding --->");
	    threshold(*blurred_image, *temp_blurred_image, 110, 255, THRESH_BINARY); 
	    convertScaleAbs(*temp_blurred_image, *back_img, 1, 0);
	    
	    //Find Connected Components
	    ROS_INFO("Finding Connected Comps for feature %d",i);
	    getConnectedComponents(i);
	    ROS_INFO("After finding conn comps, num[%d] : %d",i,numCC[i]);
	    feedback_.features[i].num = numCC[i];
	    if(numCC[i] > 0)
	      feedback_.features[i].moments.resize(numCC[i]);
	  
	    ROS_INFO("Getting Moments");
	    if(numCC[i] > 0)
	      getMoments(i);

	    ROS_INFO("Updating KF ");
	    blobTracker[i].updateKalmanFiltersConnectedComponents();
	    if (numCC[i] > 0)
	      blobTracker[i].getFilteredBlobs(true);
	    else
	      blobTracker[i].getFilteredBlobs(false);
	    
	    RotatedRect box;
	    Point pt;
	    	    
	    for(int j=0; j<maxNumComponents; j++)
	      {
		FeatureBlob ftemp;
		blobTracker[i].filters_[j].getFiltered(ftemp);
		if(ftemp.getValid() && display_image)
		  {
		    Mat firstTemp, secondTemp;
		    ftemp.getValues(firstTemp, secondTemp);
		    pt.x = firstTemp.at<float>(0,0); pt.y = firstTemp.at<float>(1,0);
		    blobTracker[i].getBoxFromCov(pt, secondTemp, box);
		    
		    if (box.size.width > 0 && box.size.height > 0 && box.size.width < width && box.size.height < height)
		      {
			ellipse(*rgb_image, box, CV_RGB(255,255,255), 3, 8);
			circle(*rgb_image, pt, 3, CV_RGB(255, 255, 255), -1, 8);
		      }	    
		    
		  }
	      }
	    
	  }
	if (display_image) {
	  imshow(name, *rgb_image);
	}
	
	server_.publishFeedback(feedback_);
	is_running = false;
	waitKey(100);
      }
  }

};
  
int main(int argc, char** argv) {
  ros::init(argc, argv, "hue_server");
  //ros::NodeHandle n("~");
    
  HueDetector3DServer node(ros::this_node::getName());
  ros::spin();
    
  return 0;
}
  
