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
#include <geometry_msgs/Point.h>
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

#define FOCAL_LENGTH 6.37e-3   //meters
#define PIXEL_WIDTH 7.05e-6 //meters
#define BASELINE 16e-2     //meters

//#define DEBUG

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;


class Localizer {
protected:
  
  ros::NodeHandle n_;

  ros::Subscriber uv_left_sub_;
  ros::Subscriber uv_right_sub_;
  ros::Publisher center_pub_;

  bool first;				       //first = true => initialize vars
  char uv_left_topic[100];
  char uv_right_topic[100];
  char center_topic[100];
  float left_u;
  float right_u;



public:
  
  Localizer(ros::NodeHandle &n, char **argv) :
    n_(n) {
    sprintf(uv_left_topic, "/stereo/left/image_raw/Feature/Center1");		//output topic
    sprintf(uv_right_topic, "/stereo/right/image_raw/Feature/Center1");
    sprintf(center_topic, "/center");

    
    //Not entirely clear on what this does
   
  
    left_u = 0.0;
    right_u = 0.0;
    //Gets the most recently published image from the topic and activates the callback method
    uv_left_sub_ = n_.subscribe(uv_left_topic, 1, &Localizer::uvLeftCallback, this);
    uv_right_sub_ = n_.subscribe(uv_right_topic, 1, &Localizer::uvRightCallback, this);
    center_pub_ = n_.advertise<geometry_msgs::Point>(center_topic, 1);


  }

  ~Localizer() {
	
  }
  void uvLeftCallback(const geometry_msgs::PointConstPtr& msg_ptr){
	left_u = msg_ptr->x;
        triangulate();
  }
  void uvRightCallback(const geometry_msgs::PointConstPtr& msg_ptr){
	right_u = msg_ptr->x;
        triangulate();
  }
  void triangulate(){
	float z;       //distance to object (in meters)

	  //NOTE: This is equation 8.10 from Rod Grupen's book
	if (left_u == right_u)
		return;
	 z = BASELINE * FOCAL_LENGTH / ((left_u - right_u) * PIXEL_WIDTH);
	 //ROS_INFO("%3.2f", z);
	geometry_msgs::Point center;
	center.x = z;
	center.y = left_u;
	center.z = right_u;
        center_pub_.publish(center);
  }

};



	//Make visible as a ROS node
int main(int argc, char** argv) {
  ros::init(argc, argv, "Localizer");
  ros::NodeHandle n ("~");

  Localizer node(n, argv);
  ros::spin();

  return 0;
}
