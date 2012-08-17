/*************************************************************
Localization- Executable = Triangulation,
Take in the two center points and compute the distance to the cone
 ************************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
#include <fstream>
#include <vector>

#define FOCAL_LENGTH 6.37e-3   //meters
#define PIXEL_WIDTH 7.05e-6 //meters
#define BASELINE 16e-2     //meters

//#define DEBUG

using namespace std;


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
  float left_v;
  float right_v;
  int left_time;
  int right_time;



public:
  
  Localizer(ros::NodeHandle &n, char **argv) :
    n_(n) {
    sprintf(uv_left_topic, "/stereo/left/image_raw/Feature/Center1");		//output topic
    sprintf(uv_right_topic, "/stereo/right/image_raw/Feature/Center1");
    sprintf(center_topic, "/center");
   
    left_v = 0.0;
    right_v = 0.0;
    left_u = 0.0;
    right_u = 0.0;
    //Gets the most recently published image from the topic and activates the callback method
    uv_left_sub_ = n_.subscribe(uv_left_topic, 1, &Localizer::uvLeftCallback, this);
    uv_right_sub_ = n_.subscribe(uv_right_topic, 1, &Localizer::uvRightCallback, this);
    center_pub_ = n_.advertise<geometry_msgs::PointStamped>(center_topic, 1);


  }

  ~Localizer() {
	
  }
  void uvLeftCallback(const geometry_msgs::PointStampedConstPtr& msg_ptr){
	left_u = msg_ptr->point.x;
	left_v = msg_ptr->point.y;
	left_time = msg_ptr->header.stamp.sec;
        triangulate();
  }
  void uvRightCallback(const geometry_msgs::PointStampedConstPtr& msg_ptr){
	right_u = msg_ptr->point.x;
	left_v = msg_ptr->point.y;
	right_time = msg_ptr->header.stamp.sec;
        triangulate();
  }
  void triangulate(){
	//only triangulate if the center of both images is delivered at the same time
	if (abs(left_time-right_time) > 2){
		return;
	}
	//ignore cases where the centers don't make sense
	if ((left_u - right_u) <= 0.0){
		return;
	}
	float z;       //distance to object (in meters)
	//ignore pairs of centers that are too far apart vertically	
	if (abs(left_v-right_v) > 300){
		return;
	}
	  //NOTE: This is equation 8.10 from Rod Grupen's book
	 z = BASELINE * FOCAL_LENGTH / ((left_u - right_u) * PIXEL_WIDTH);
	 //ROS_INFO("%3.2f", z);
	//ignore objects that are reported as far away to avoid some false positivies
	if (z> 12.0){
		return;
	}
	geometry_msgs::PointStamped center;
	center.point.x = z;
	center.point.y = left_u;
	center.point.z = right_u;
        center.header.stamp = ros::Time::now();
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
