/*************************************************************
 * argv[1] : camera topic name (for eg: /BB2, /SonyX700_Left 
 * argv[2] : blob topic name 
 *            (will be publshed in /<cam_topic>/Feature/ColoBlobs<blob topic>
 * argv[3] :Minimum are of connected components
 ************************************************************************/

#include <ros/ros.h>
#include <ros/init.h>
#include <pcl/features/feature.h>
#include <Image_msgs/FeatureMoments3D.h>
#include <FeatureTracker/FeatureBlobTracker.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>

//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/core/core.hpp>
//#include <opencv2/core/mat.hpp>
//#include <opencv2/core/operations.hpp>
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
//#define DEFAULT_NAME "TestColor"

const double PI = 3.141592;

int init_mean_color = 80;
int init_window_size = 10;
int init_blur_kernel = 5;
int init_display_image = 1;

int mean_color = init_mean_color;    //the color being search for +/- (window_size/2)
int colorIndex = 0;
int window_size = init_window_size;	 
int minCCThreshold = DEFAULT_MIN_CC_THRESHOLD;	//the minimum area (in pixels) for a connected component
int maxCCThreshold = DEFAULT_MAX_CC_THRESHOLD;	//the maximum area (in pixels) for a connected component
int maxNumComponents = MAX_FEATURES;  //the maximum number of connected components
int blurKernel = init_blur_kernel;    //the size of the Gaussian kernel
int width = DEFAULT_WIDTH;     //the width and height of the input image
int height = DEFAULT_HEIGHT;

Rect new_box;
Rect old_box;

bool drawing_box = false;
bool find_avg_color = false;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

void on_trackbar(int h, void *f) {
}

// Implement mouse callback
void mouseCallback( int event, int x, int y, int flags, void* param ){
  
  switch(event){
  case CV_EVENT_MOUSEMOVE: 
    if(drawing_box){
      new_box.width = x-new_box.x;
      new_box.height = y-new_box.y;
    }
    break;
    
  case CV_EVENT_LBUTTONDOWN:
    drawing_box = true;
    new_box = Rect(x, y, 0, 0);
    break;
    
  case CV_EVENT_LBUTTONUP:

    drawing_box = false;
    
    old_box.x = new_box.x;
    old_box.y = new_box.y;
    old_box.width = new_box.width;
    old_box.height = new_box.height;

    find_avg_color = true;

    new_box.width = 0;
    new_box.height = 0;
    break;
  }
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

class SatDetector3D {
protected:
  
  ros::NodeHandle n_;
  ros::Subscriber pointcloud_sub_;
  image_transport::ImageTransport it_;

  image_transport::Subscriber sat_sub_;
  image_transport::Subscriber rgb_sub_;
  ros::Publisher moments_pub_;
  ros::Publisher object_pose_marker_pub_;

  cv_bridge::CvImagePtr ptr_sat;
  cv_bridge::CvImagePtr ptr_rgb;

  VideoCapture cap;
  Mat frame;				    //the current frame being processed
  Mat cv_output_;				//the output image
  
  bool first;				       //first = true => initialize vars
  char color_topic[100];					//name for output topic
  char rgb_topic[100];			//name for input topic
  char sat_topic[100];			//name for input topic
  char pointcloud_topic[100];		//name for input topic
  char marker_topic[100];

  int display_image;
  int numCC;				     //the number of valid connected components
  int COLOR_LEGENDS[MAX_FEATURES][3];				//colors for display
  int ccMeans[MAX_FEATURES][2];			 //the means for storage

  vector <ConnectedComp> comps;			//array of connected components
  vector <vector <Point> > contours;	       //storage for linked list contours
  vector <Vec4i> hierarchy;

  FeatureBlobTracker *blobTracker;	 //Kalman filter and storage for blobs (across timesteps)
  Mat *flipped;				//image flipped vertically
  Mat *temp_blurred_image; 
  Mat *blurred_image;			//blurred images 
  Mat *color_cc_image;			//connected component image
  Mat* back_img;	       		//unknown use (output image?)	
  Mat* sat_image;		       //input image -> sat space
  Mat* hue_image;
  Mat* int_image;		       //input image -> intensity space
  Mat* copy_image;	      		//a copy of cv_image
  Mat* rgb_image;	 		// rgb image from cameras
  Mat* hsv_image;		        // hsv image from cameras
  sensor_msgs::Image img;
  PointCloudRGB pcl_in;
  bool is_running;
  std::string name;

  Image_msgs::FeatureMoments3D moments;   //Message for publishing moments
  char *window_name; //name of the feature

public:
  
  SatDetector3D(ros::NodeHandle &n, char **argv) :
    n_(n), it_(n_) {
    
    first = true;
    
    if (!n_.getParam("maxNumComponents", maxNumComponents)) {
      ROS_INFO("Could not retrieve 'maxNumComponents' parameter, setting to default value of '%d'", MAX_FEATURES);
      maxNumComponents = MAX_FEATURES;
    }

    if (!n_.getParam("blurKernel", blurKernel)) {
      ROS_INFO("Could not retrieve 'blurKernel' parameter, setting to default value of '%d'", init_blur_kernel);
      blurKernel = init_blur_kernel;
    }

    if (!n_.getParam("colorIndex", colorIndex)) {
      ROS_INFO("Could not retrieve 'colorIndex' parameter, setting to default value of '0'");
      colorIndex = 0;
    }
    else{
      //assumes a window size of 10
      mean_color = (colorIndex*10)+5;
      ROS_INFO("Mean Color set to %d", mean_color);
    }

    if (!n_.getParam("minCCThreshold", minCCThreshold)) {
      ROS_INFO("Could not retrieve 'minCCThreshold' parameter, setting to default value of '%d'", DEFAULT_MIN_CC_THRESHOLD);
      minCCThreshold = DEFAULT_MIN_CC_THRESHOLD;
    }

    if (!n_.getParam("maxCCThreshold", maxCCThreshold)) {
      ROS_INFO("Could not retrieve 'maxCCThreshold' parameter, setting to default value of '%d'", DEFAULT_MAX_CC_THRESHOLD);
      maxCCThreshold = DEFAULT_MAX_CC_THRESHOLD;
    }
    
    if (!n_.getParam("display_image", display_image)) {
      ROS_INFO("Could not retrieve 'display_image' parameter, setting to default value of '%d'", init_display_image);
      display_image = init_display_image;
    }	

    std::string source_identifier;
    if (!n_.getParam("source_identifier", source_identifier)) {
      ROS_INFO("Could not retrieve 'source_identifier' parameter, setting to default value of '%s'", DEFAULT_SOURCE_IDENTIFIER);
      source_identifier = DEFAULT_SOURCE_IDENTIFIER;
    }
  
    std::stringstream sat_name;
    sat_name << name << "Sat" << colorIndex;
    sprintf(pointcloud_topic, "%s", source_identifier.c_str());	//input topic
    sprintf(color_topic, "%s/Feature/ColorBlobs/%s", source_identifier.c_str(), sat_name.str().c_str());		
    srand ( time(NULL) );

    //output topic
    sprintf(marker_topic,"ellipsoid_marker/%s",sat_name.str().c_str());
    printf("Topic name : %s\n", color_topic);
    pointcloud_sub_ = n_.subscribe(pointcloud_topic, 1, &SatDetector3D::pointCloudCallback, this);
    moments_pub_ = n_.advertise<Image_msgs::FeatureMoments3D>(color_topic, 1);
    object_pose_marker_pub_ = n_.advertise<visualization_msgs::Marker>(marker_topic,1);
    
    if (display_image) {
      namedWindow(name, CV_WINDOW_AUTOSIZE);
      //namedWindow("feat", CV_WINDOW_AUTOSIZE);
      createTrackbar("Mean Color", name, &mean_color, 255, &on_trackbar);
      createTrackbar("Window Size", name, &window_size, 89, &on_trackbar);
      createTrackbar("Blur Kernel", name, &blurKernel, 25, &on_trackbar);
      createTrackbar("Min Size", name, &minCCThreshold, 3000, &on_trackbar); 
      createTrackbar("Max Size", name, &maxCCThreshold, 10000, &on_trackbar);

      setMouseCallback(name, &mouseCallback);
    }
   	
    sat_image = new Mat(height, width, CV_8UC1);
    sat_image = new Mat(height, width, CV_8UC1);
    int_image = new Mat(height, width, CV_8UC1);
    back_img = new Mat(sat_image->size(), CV_8UC1);
    temp_blurred_image = new Mat(sat_image->size(), CV_8UC1);
    blurred_image = new Mat(sat_image->size(), CV_8UC1);
    color_cc_image = new Mat(sat_image->size(), CV_8UC3, Scalar(0));
    copy_image = new Mat(sat_image->size(), CV_8UC1);
    flipped = new Mat(sat_image->size(), CV_8UC1);
    rgb_image = new Mat(sat_image->size(), CV_8UC3);
    hsv_image = new Mat(sat_image->size(), CV_8UC3);
    
    blobTracker = new FeatureBlobTracker(maxNumComponents, width, height);
    blobTracker->initialize();

    comps = vector<ConnectedComp>(); 
    contours = vector<vector<Point> >();
    hierarchy = vector<Vec4i>();

    comps.resize(maxNumComponents);
    is_running = false;
    drawing_box = false;

  }

  ~SatDetector3D() {
    comps.clear();
    contours.clear();
    hierarchy.clear(); 

    sat_image->release();
    back_img->release();
    temp_blurred_image->release();
    blurred_image->release();
    color_cc_image->release();
    copy_image->release();
    flipped->release();
    rgb_image->release();

    //delete blobTracker;
  }

  int getThresholdConnectedComponents() {
    int count = 0;
    double area;
    double contour_areas[(int)contours.size()];
    for(int idx = 0; idx < (int)contours.size(); idx++){
      vector<Point> approx;
      vector<Point> current = contours[idx];
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
      for(int idx = 0; idx < (int)contours.size(); idx++){
	if ((contour_areas[idx] > minCCThreshold) && (contour_areas[idx] > largestArea)){
	  largestArea = contour_areas[idx];
	  largestIdx = idx;		
	}
      }
      if(largestArea > minCCThreshold){
	comps[count].area = largestArea;
	comps[count].idx = largestIdx;
	//cout << "largest area " << largestArea << " idx: " << largestIdx << " count: " << count << endl;
	contour_areas[largestIdx] = 0;
	count++;
      }	
      else
	break;
    }
    for(int left = (count); left < maxNumComponents; left++){
      comps[left].area = -1;
      comps[left].idx = -1;
    }
#ifdef DEBUG
    cout << "Number of Connected Components " << count << endl;
#endif
    return (count);
  }

  void getConnectedComponents() {
    findContours(*back_img, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE, Point());
    if(!contours.empty())
      numCC = getThresholdConnectedComponents();
  }


  void getMoments() {   
 
    blobTracker->reset();
   		
    Mat first(2,1,CV_32F);
    Mat second(2,2,CV_32F);
    for (int k = 0; (k < numCC) && (k < (int)comps.size()); k++) {
      if((comps[k].idx < (int)contours.size()) && (comps[k].idx >= 0)){
	vector<Point> current = contours[comps[k].idx];
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
	blobTracker->addNewMeasurement(first, second, k,"color",k);
	//Print the points
	//for(int i=0; i<current.size(); i++)
	//ROS_INFO("%d : %d --> %d,%d",k,i,current[i].x,current[i].y);
	get3DMoments(current, k);
      }
    }

    for (int k = numCC; k < maxNumComponents; k++)
      {
	//set the other features invalid
	first.at<float>(0,0) = -1;
	first.at<float>(1,0) = -1;
	second.at<float>(0,0) = 1; second.at<float>(0,1) = 0; second.at<float>(1,0) = 0; second.at<float>(1,1) = 1;
	blobTracker->addNewMeasurement(first, second, k,"color",k);
      }
    
    blobTracker->matchRawDataToEstimates(false, numCC);
  }

  void get3DMoments(vector<Point> feature_points, int index)
  {
    //    for(int i=0; i<feature_points.size(); i++)
    //   ROS_INFO("%d --> %d,%d",i,feature_points[i].x,feature_points[i].y);
    //ROS_INFO("Getting 3D Moments : %d --> %d,%d", feature_points.size(), width, height);
    
    //Extract the indices for the points in the point cloud data
    pcl::PointIndices point_indices;
     
    for(int i=0; i<(int)feature_points.size(); i++)
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
	moments.moments[index].mean[i] = centroid(i);
	for(int j=0; j<3; j++)
	  {
	    moments.moments[index].covariance[i*3+j] = covariance_matrix(i,j);
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
    
    numCC = 0;
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
    
    *sat_image = planes[1];
       
    if(display_image && find_avg_color){
      
      Mat sat_roi(*sat_image, old_box);

      Scalar sat_mean;
      Scalar sat_std_dev;
      meanStdDev(sat_roi, sat_mean, sat_std_dev);
      
      ROS_INFO("Average Color in selected region %+4.3f Std. Deviation %+4.3f", sat_mean(0), sat_std_dev(0));

      find_avg_color = false;
      
    }

 
    //Size s_sat = sat_image->size();
    //for(int i = 0; i < s_sat.width; i++){
    //  for(int j =0; j < s_sat.height; j++){
    // 	ROS_INFO("%d ", (sat_image->at<uchar>(i,j)));
    //  }
    //  ROS_INFO("\n");
    //}
    
    int rangeMin = ((mean_color - window_size + 255)%255);
    int rangeMax = ((mean_color + window_size + 255)%255);

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
      inRange(*sat_image, Scalar((double)((uchar)rangeMin)),Scalar((double)((uchar)rangeMax)), *back_img);
    }
    else if ((mean_color + window_size) > 255){
      //ROS_INFO("BIG rangeMin %d rangeMax %d", rangeMin, rangeMax);
      inRange(*sat_image, Scalar((double)((uchar)rangeMax)),Scalar((double)((uchar)255)), *back_img);
    }
    else {
      //ROS_INFO("SML rangeMin %d rangeMax %d", rangeMin, rangeMax);
      inRange(*sat_image, Scalar((double)((uchar)0)),Scalar((double)((uchar)rangeMin)), *back_img);
    }  

    *color_cc_image = Scalar(0);
    back_img->copyTo(*sat_image);
    Size ksize = Size(2 * blurKernel + 1,2 * blurKernel + 1);
    GaussianBlur(*back_img, *blurred_image, ksize, -1, -1);
      
    threshold(*blurred_image, *temp_blurred_image, 110, 255, THRESH_BINARY); 
    convertScaleAbs(*temp_blurred_image, *back_img, 1, 0);
    sat_image->copyTo(*copy_image);
      
    //if (display_image){
    //imshow(color_topic, *back_img);
    //}
      

    //Find Connected Components
    getConnectedComponents();
    moments.num = numCC;
    moments.moments.resize(numCC);
      
    for (int i = 0; (i < (int)comps.size()) && (comps[i].idx >= 0) && (comps[i].idx < (int)contours.size()); i++) {
      Scalar color( (rand()&255), (rand()&255), (rand()&255) );
      drawContours(*color_cc_image, contours, comps[i].idx, color, 3, 8, hierarchy,0, Point());
      drawContours(*sat_image, contours, 0, comps[i].idx, 3, 8, hierarchy,0, Point());
    }
      
    getMoments();
    blobTracker->updateKalmanFiltersConnectedComponents();
    if (numCC > 0)
      blobTracker->getFilteredBlobs(true);
    else
      blobTracker->getFilteredBlobs(false);
          
    RotatedRect ellipse_box;
    Point pt;
    
    
    for(int i=0; i<maxNumComponents; i++)
      {
	FeatureBlob ftemp;
	blobTracker->filters_[i].getFiltered(ftemp);
	if(ftemp.getValid() && display_image)
	  {
	    Mat firstTemp, secondTemp;
	    ftemp.getValues(firstTemp, secondTemp);
	    pt.x = firstTemp.at<float>(0,0); pt.y = firstTemp.at<float>(1,0);
	    blobTracker->getBoxFromCov(pt, secondTemp, ellipse_box);
	      
	    if (ellipse_box.size.width > 0 && ellipse_box.size.height > 0 && ellipse_box.size.width < width && ellipse_box.size.height < height)
	      {
		ellipse(*rgb_image, ellipse_box, CV_RGB(255,255,255), 3, 8);
		circle(*rgb_image, pt, 3, CV_RGB(255, 255, 255), -1, 8);
	      }	    
	      
	  }
      }
    if (display_image) {
      if(drawing_box ) 
	rectangle( *rgb_image, Point(new_box.x, new_box.y), Point(new_box.x + new_box.width, new_box.y + new_box.height), Scalar(255,0,0), 2);
      imshow(name, *rgb_image);
    }
    
    moments_pub_.publish(moments);
      
    waitKey(10);

    is_running = false;
  }
};
  
int main(int argc, char** argv) {
  ros::init(argc, argv, "SatDetector3D");
  ros::NodeHandle n("~");
    
  SatDetector3D node(n, argv);
  ros::spin();
    
  return 0;
}
