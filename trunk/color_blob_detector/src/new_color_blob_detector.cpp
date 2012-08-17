/*************************************************************
 * argv[1] : camera topic name (for eg: /BB2, /SonyX700_Left 
 * argv[2] : blob topic name 
 *            (will be publshed in /<cam_topic>/Feature/ColoBlobs<blob topic>
 * argv[3] :Minimum are of connected components
 ************************************************************************/

#include <ros/ros.h>
#include <pcl/features/feature.h>
#include <Image_msgs/FeatureMoments2D.h>
#include <feature_tracker/feature_blob_tracker.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/operations.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/features/feature.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

//#define DEBUG

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

#define MAX_FEATURES 10
#define DEFAULT_MIN_CC_THRESHOLD 200
#define DEFAULT_COLOR_BLOB_NUM 1
//#define DEFAULT_SOURCE_IDENTIFIER "/kinect/camera/rgb/points"
#define DEFAULT_SOURCE_IDENTIFIER "/stereo/right/image_raw"
#define DEFAULT_DISPLAY_IMAGE 1 
int width = 640;
int height = 480;

const double PI = 3.141592;

int init_hue_thresh = 0;
int init_sat_thresh = 0;
int init_int_thresh = 95;

int hue_thresh = init_hue_thresh;
int sat_thresh = init_sat_thresh;
int int_thresh = init_int_thresh;
char file[100];
char suffix[50] = "new_wo_2";

int num_cols[3], num_rows[3], im_size[3], num_elements;
double im_mean[3], im_variance[3], im_sum[3];
double cov_sum[3][3], im_cov[3][3];
int im_hist[3][256];
int log_count_ = 0.0; 
int glob_im_hist_[3][256];
double variance_coeff_; 
double hue_var_coeff_;
double sat_var_coeff_;
double val_var_coeff_;
double glob_im_mean_[3];
double glob_im_cov_[3][3];	
double glob_obj_mean_[3];
double glob_obj_cov_[3][3];
double glob_norm_hist_[3][101];
ofstream myfile;
std::string with_cone="false";
vector <Mat> old_planes;




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

class ColorBlobDetector {
protected:
  
  ros::NodeHandle n_;
  ros::Subscriber pointcloud_sub_;
  image_transport::ImageTransport it_;

  image_transport::Subscriber hue_sub_;
  image_transport::Subscriber rgb_sub_;
  image_transport::Publisher image_pub_;

  cv_bridge::CvImagePtr ptr_hue;
  cv_bridge::CvImagePtr ptr_rgb;

  VideoCapture cap;
  Mat frame;				    //the current frame being processed
  Mat cv_output_;				//the output image
  
  bool first;				       //first = true => initialize vars
  char color_topic[100];					//name for output topic
  char rgb_topic[100];					//name for input topic
  char hue_topic[100];					//name for input topic
  char pointcloud_topic[100];					//name for input topic

  bool display_image_;
  bool new_file;
  int numCC;				     //the number of valid connected components
  int COLOR_LEGENDS[MAX_FEATURES][3];				//colors for display
  int ccMeans[MAX_FEATURES][2];			 //the means for storage

  vector <ConnectedComp> comps;			//array of connected components
  vector <vector <Point> > contours;	       //storage for linked list contours
  vector <Vec4i> hierarchy;

  Mat* hue_image;		       //input image -> hue space
  Mat* sat_image;		       //input image -> satuation space
  Mat* int_image;		       //input image -> intensity space
  Mat* copy_image;	      		//a copy of cv_image
  Mat* rgb_image;	 		// rgb image from cameras
  Mat* hsv_image;		        // hsv image from cameras
  Mat* old_image;
  Mat* back_img; 
  Mat disp_image;
  sensor_msgs::Image img;
  PointCloudRGB pcl_in;
  bool is_running;

public:
  
  ColorBlobDetector(ros::NodeHandle &n, char **argv) :
    n_(n), it_(n_) {
    first = true;

    if (!n_.getParam("display_image", display_image_)) {
      ROS_INFO("Could not retrieve 'display_image' parameter, setting to default value of '%d'", DEFAULT_DISPLAY_IMAGE);
      display_image_ = DEFAULT_DISPLAY_IMAGE;
    }
    
    std::string source_identifier;
    if (!n_.getParam("source_identifier", source_identifier)) {
      ROS_INFO("Could not retrieve 'source_identifier' parameter, setting to default value of '%s'", DEFAULT_SOURCE_IDENTIFIER);
      source_identifier = DEFAULT_SOURCE_IDENTIFIER;
    }
    
    //Not entirely clear on what this does
    sprintf(hue_topic, "%s", source_identifier.c_str());	//input topic
    sprintf(rgb_topic, "%s", source_identifier.c_str());	//input topic

    sprintf(color_topic, "%s/Feature/ColorBlobs", source_identifier.c_str());	
    printf("Topic name : %s\n", color_topic);
    
    //hue_sub_ = it_.subscribe(hue_topic, 1, &ColorBlobDetector::hueCallback, this);


    //Gets the most recently published image from the topic and activates the callback method
    rgb_sub_ = it_.subscribe(rgb_topic, 1, &ColorBlobDetector::rgbCallback, this);
    image_pub_ = it_.advertise(color_topic, 1);

    if (display_image_){
	namedWindow("New Points", CV_WINDOW_AUTOSIZE);
	createTrackbar("hue threshold", "New Points", &hue_thresh, 255, &on_trackbar);
	createTrackbar("sat threshold", "New Points", &sat_thresh, 255, &on_trackbar);
	createTrackbar("int threshold", "New Points", &int_thresh, 255, &on_trackbar);
    }
    
  


	//Create matrices to store the three layers of the HSV image	
    hue_image = new Mat(height, width, CV_8UC1);
    sat_image = new Mat(height, width, CV_8UC1);
    int_image = new Mat(height, width, CV_8UC1);
    back_img = new Mat(height, width, CV_8UC1);
    disp_image = Mat(height, width, CV_8UC1);

	//Creates the blurred image, and other modifications

    rgb_image = new Mat(hue_image->size(), CV_8UC3);
    hsv_image = new Mat(hue_image->size(), CV_8UC3);
    old_image = new Mat(hue_image->size(), CV_8UC3);

    
	//Blobtracker records the location of blobs between time frames (
    //blobTracker = new FeatureBlobTracker(maxNumComponents, width, height);
    //blobTracker->initialize();


    is_running = false;
	for (int i=0;i<3;i++){
		glob_im_mean_[i]=0.0;
		glob_obj_mean_[i]=0.0;
		for (int j=0;j<3;j++){		
			glob_im_cov_[i][j]=0.0;
			glob_obj_cov_[i][j]=0.0;
		}
		for (int k=0;k<256;k++){		
			glob_im_hist_[i][k]=0;
		}
	}
	for (int i=0;i<3;i++){	//3--- 1-norm, 2-norm and frob-norm
		for (int k=0;k<100;k++){		
			glob_norm_hist_[i][k]=0.0;		
		}
	}
   

  }

  ~ColorBlobDetector() {
  }
	//Called on each new image from the topic
  void rgbCallback(const sensor_msgs::ImageConstPtr& msg_ptr) {
    
	//Convert the image to OpenCV form
    try {
      ptr_rgb = cv_bridge::toCvCopy(msg_ptr, "bgr8");
      ptr_rgb->image.copyTo(*rgb_image);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    waitKey(2);

	//Create a test image

	//Create a HSV image from the RGB image
    cvtColor(*rgb_image, *hsv_image, CV_BGR2HSV);

	//Split the three image channels into separate matrices 
    vector <Mat> planes;
    split(*hsv_image, planes);
    *hue_image = planes[0];
    *sat_image = planes[1];
    *int_image = planes[2];
   
  

	for(int i=0;i<3;i++){
		num_cols[i]=planes[i].cols;
		num_rows[i]=planes[i].rows;
		im_size[i]=num_cols[i]*num_rows[i];
		im_sum[i]=0;
		for(int j=0;j<256;j++){
			im_hist[i][j]=0;
		}		
	}
	if (with_cone == "true"){
		log_count_ = 0.0;
		new_file = true;
		with_cone = "false";
		num_elements = 0;
		for (int i=0;i<3;i++){
			glob_im_mean_[i]=0.0;
			glob_obj_mean_[i]=0.0;
			for (int j=0;j<3;j++){		
				glob_im_cov_[i][j]=0.0;
				glob_obj_cov_[i][j]=0.0;
			}
			for (int k=0;k<256;k++){		
				glob_im_hist_[i][k]=0;
			}
		}
		for (int i=0;i<3;i++){	//3--- 1-norm, 2-norm and frob-norm
			for (int k=0;k<100;k++){		
				glob_norm_hist_[i][k]=0.0;		
			}
		}
	}		
	if (new_file){
		for(int i=0;i<3;i++){
			num_cols[i]=planes[i].cols;
			num_rows[i]=planes[i].rows;
			im_size[i]=num_cols[i]*num_rows[i];
			im_sum[i]=0;
			for(int j=0;j<256;j++){
				im_hist[i][j]=0;
			}		
		}
   		for (int k=0; k<3; k++){

			for (int i=0; i<im_size[k]; i++){
				if (abs(planes[0].data[i]-old_planes[0].data[i]) < hue_thresh)
					planes[2].data[i] = 0.0;
				else if (abs(planes[1].data[i]-old_planes[1].data[i]) < sat_thresh)
					planes[2].data[i] = 0.0;
				else if(abs(planes[2].data[i] - old_planes[2].data[i]) < int_thresh)
					planes[2].data[i] = 0.0;

			}
		}
		int_image-> copyTo(disp_image);
		for (int i=0; i<im_size[2]; i++){
			if (disp_image.data[i] > 0){
				disp_image.data[i] = 255;
			}
		} 

		imshow("New Points", disp_image);
		

	}


		num_elements=0;	
		for(int i=0;i<im_size[0];i++){	
			if (planes[2].data[i]>0){ // if the intensity is zero, this means that filtered point doesn't have any information about that point.
					num_elements = num_elements+ 1;			
				for (int k=0;k<3;k++){
					im_sum[k] = im_sum[k] + planes[k].data[i]; // for mean				
					im_hist[k][planes[k].data[i]] = im_hist[k][planes[k].data[i]]+1; //for histogram								
				}
			}
		}	
		if(num_elements > 0){
			for (int k=0;k<3;k++){
				im_mean[k] = im_sum[k]/num_elements;			
			}
		}else{
			for (int k=0;k<3;k++){
				im_mean[k] = 0;
			}
		}

		//---------------compute covariance----------------
		for (int i=0;i<3;i++){
			for (int j=0;j<3;j++){
				cov_sum[i][j]=0.0;
			}
		}	
		for(int i=0;i<im_size[0];i++){	
			//im_sum=im_sum+pow((planes[hsv_j].data[i]-im_mean[hsv_j]),2);		
			if (planes[2].data[i]>0){		
				cov_sum[0][0]=cov_sum[0][0] + (planes[0].data[i]-im_mean[0])*(planes[0].data[i]-im_mean[0]);
				cov_sum[1][0]=cov_sum[1][0] + (planes[1].data[i]-im_mean[1])*(planes[0].data[i]-im_mean[0]);
				cov_sum[2][0]=cov_sum[2][0] + (planes[2].data[i]-im_mean[2])*(planes[0].data[i]-im_mean[0]);
				
				cov_sum[0][1]=cov_sum[0][1] + (planes[0].data[i]-im_mean[0])*(planes[1].data[i]-im_mean[1]);
				cov_sum[1][1]=cov_sum[1][1] + (planes[1].data[i]-im_mean[1])*(planes[1].data[i]-im_mean[1]);			
				cov_sum[2][1]=cov_sum[2][1] + (planes[2].data[i]-im_mean[2])*(planes[1].data[i]-im_mean[1]);
				
				cov_sum[0][2]=cov_sum[0][2] + (planes[0].data[i]-im_mean[0])*(planes[2].data[i]-im_mean[2]);
				cov_sum[1][2]=cov_sum[1][2] + (planes[1].data[i]-im_mean[1])*(planes[2].data[i]-im_mean[2]);			
				cov_sum[2][2]=cov_sum[2][2] + (planes[2].data[i]-im_mean[2])*(planes[2].data[i]-im_mean[2]);
			}
		}	
		for (int i=0;i<3;i++){
			for (int j=0;j<3;j++){
				if(num_elements>0){
					im_cov[i][j]=cov_sum[i][j]/num_elements;
				}else{
					im_cov[i][j]=0.0;
				}
			}
		}
		//---- Compute the mean of the mean and covariance ----
		for (int i=0;i<3;i++){
			glob_im_mean_[i]=glob_im_mean_[i]+im_mean[i];
			for (int j=0;j<3;j++){
				glob_im_cov_[i][j]=glob_im_cov_[i][j]+im_cov[i][j];
			}
			for (int k=0;k<256;k++){
				glob_im_hist_[i][k]=glob_im_hist_[i][k]+im_hist[i][k];
			}
		}
		log_count_=log_count_+1;
		waitKey(2);	
		if(log_count_==50){
			for (int i=0;i<3;i++){

				glob_im_mean_[i]=glob_im_mean_[i]/log_count_;
				for (int j=0;j<3;j++){
					glob_im_cov_[i][j]=glob_im_cov_[i][j]/log_count_;
				}
				for(int k=0;k<256;k++){
					glob_im_hist_[i][k]=glob_im_hist_[i][k];
				}
			}
			strcpy(file, "test_");
			strcat(file, suffix);
			strcat(file, ".txt");
			myfile.open(file);			
					
			myfile << ("=================================================\n");
			myfile << ("-------------count = %d ----------\n",log_count_);
			myfile <<"hue: mean=" <<glob_im_mean_[0] <<"variance=" << glob_im_cov_[0][0] <<"\n";
			myfile <<"sat: mean=" <<glob_im_mean_[1] <<"variance=" << glob_im_cov_[1][1] <<"\n";
			myfile <<"val: mean=" <<glob_im_mean_[2] <<"variance=" << glob_im_cov_[2][2] <<"\n";
			myfile <<"hue_thresh=" <<hue_thresh <<"sat_thresh=" <<sat_thresh <<"int_thresh=" <<int_thresh <<"\n";
			//myfile <<("sat: mean=%6.4f, variance=%6.4f\n",glob_im_mean_[1],glob_im_cov_[1][1]);
			//myfile <<("val: mean=%6.4f, variance=%6.4f\n",glob_im_mean_[2],glob_im_cov_[2][2]);
			
			myfile <<("cov: %6.4f %6.4f %6.4f\n",glob_im_cov_[0][0],glob_im_cov_[0][1],glob_im_cov_[0][2]);
			myfile <<("   : %6.4f %6.4f %6.4f\n",glob_im_cov_[1][0],glob_im_cov_[1][1],glob_im_cov_[1][2]);
			myfile <<("   : %6.4f %6.4f %6.4f\n",glob_im_cov_[2][0],glob_im_cov_[2][1],glob_im_cov_[2][2]);
			myfile <<("=================================================\n");
			myfile <<("========== hue hist =============\n");
			for (int l=0;l<32;l++){
				for (int n=0;n<8;n++){
					myfile <<("%d ",glob_im_hist_[0][8*l+n]) << " ";
				}
				myfile <<("\n");
			}

			myfile <<("========== sat hist =============\n");
			for (int l=0;l<32;l++){
				for (int n=0;n<8;n++){
					myfile <<("%d ",glob_im_hist_[1][8*l+n]) << " ";
				}
				myfile <<("\n");
			}
			myfile <<("========== val hist =============\n");
			for (int l=0;l<32;l++){
				for (int n=0;n<8;n++){
					myfile <<("%d ",glob_im_hist_[2][8*l+n]) << " ";
				}
				myfile <<("\n");
			}
			myfile.close();
			if (with_cone == "false"){

				hsv_image-> copyTo(*old_image);
				split(*old_image, old_planes);
				cout << "with cone, true or false?";
	                        cin >> with_cone;
				cout << "suffix?";
				cin >> suffix;
				cout << "hue?";
				cin >> hue_thresh;
				cout << "sat?";
				cin >> sat_thresh;
				cout << "int?";
				cin >> int_thresh;
			}		
		}else if (log_count_<50){
			printf("-------------log_count=%d----------\n",log_count_);
			printf("hue: mean=%6.4f, variance=%6.4f\n",im_mean[0],im_cov[0][0]);
			printf("sat: mean=%6.4f, variance=%6.4f\n",im_mean[1],im_cov[1][1]);
			printf("val: mean=%6.4f, variance=%6.4f\n",im_mean[2],im_cov[2][2]);
			
			printf("cov: %6.4f %6.4f %6.4f\n",im_cov[0][0],im_cov[0][1],im_cov[0][2]);
			printf("   : %6.4f %6.4f %6.4f\n",im_cov[1][0],im_cov[1][1],im_cov[1][2]);
			printf("   : %6.4f %6.4f %6.4f\n",im_cov[2][0],im_cov[2][1],im_cov[2][2]);
		}


  }
  
};


	//Make visible as a ROS node
int main(int argc, char** argv) {
  ros::init(argc, argv, "ColorBlobDetector2D");
  ros::NodeHandle n;

  ColorBlobDetector node(n, argv);
  ros::spin();
  //cout << "with cone, true or false?";
  //cin >> with_cone;


  return 0;
}
