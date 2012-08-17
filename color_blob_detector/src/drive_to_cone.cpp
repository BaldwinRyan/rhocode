/******************************
Takes distance to cone and sends distance and angle command to atrvjr node
making it drive to the cone.
Executable = cone_present
****************************/
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <simple/controller_pub.h>
#include <simple/controller_pub2.h>
#include <list>
#include <boost/tuple/tuple.hpp>

#define _USE_MATH_DEFINES
#define FOCAL_LENGTH 6.5e-3   //meters
#define PIXEL_WIDTH 7.05e-6 //meters
#define BASELINE 16e-2     //meters

//#define DEBUG

using namespace std;

int num_cones = 0;
boost::tuple<double, double> cones[50];
  




class ConePresentController {
protected:
  
  ros::NodeHandle n_;

  
  ros::Subscriber center_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher heading_pub_;


  char center_topic[100];
  char heading_topic[100];
  char odom_topic[100];		
  float distances[15];	//keep old distances from cone
  float left_u;		//pixel that contains center of cone in left image
  float right_u;	//pixel that contains center of cone in right image
  float dist_x;		//distance from cone
  float robot_heading;	//read from odom
  float robot_x;	//read from odom
  float robot_y;	//read from odom

  int new_time;


public:
  
  ConePresentController(ros::NodeHandle &n, char **argv) :
    n_(n) {
    sprintf(center_topic, "/center");
    sprintf(heading_topic, "/current_heading");
    sprintf(odom_topic, "/odom");

    
    //Not entirely clear on what this does
   
    for (int i = 0; i<15; i++){
	distances[i] = 0.0;
    }
    left_u = 0.0;
    right_u = 0.0;
    dist_x = 0.0;
//std::ofstream cone_file;
//cone_file.open("cone_locations.csv");
    //Gets the most recently published image from the topic and activates the callback method	
    center_sub_ = n_.subscribe<geometry_msgs::PointStamped>(center_topic, 1, &ConePresentController::distanceCallback, this);
    odom_sub_ = n_.subscribe<nav_msgs::Odometry>(odom_topic, 1, &ConePresentController::headingCallback, this);
    heading_pub_ = n_.advertise<geometry_msgs::Pose>(heading_topic, 1);

  }

  ~ConePresentController() { 
	
  }
  void headingCallback(const nav_msgs::OdometryConstPtr& msg_ptr){
	robot_x = msg_ptr-> pose.pose.position.x;
	robot_y = msg_ptr-> pose.pose.position.y;
  	float w = msg_ptr-> pose.pose.orientation.w;
	float z = msg_ptr-> pose.pose.orientation.z;
	float angle = 2*acos(w);
  	if (w == 1 || z == 0)
    		robot_heading = angle;
  	else
    		robot_heading = angle * z / sin( angle / 2 );
  }
  void distanceCallback(const geometry_msgs::PointStampedConstPtr& msg_ptr){
	dist_x = msg_ptr->point.x;
	left_u = msg_ptr->point.y;
	right_u = msg_ptr->point.z;
	new_time = msg_ptr-> header.stamp.sec;
	for (int i = 0; i < 15; i++){
		if (i == 14)
			distances[i] = dist_x;
		else
        		distances[i] = distances[i+1];
	}
	getCommand();
  }

  void getCommand(){
	//srv tells motion controller whether or not to publish
	//srv2 tells atrvjr_node whether or not motion controller is publishing
	ros::ServiceClient client = n_.serviceClient<simple::controller_pub>("/controller_pub");
  	simple::controller_pub srv;
	srv.request.publish_active = false;
	ros::ServiceClient client2 = n_.serviceClient<simple::controller_pub2>("/controller_pub2");
  	simple::controller_pub2 srv2;
	srv2.request.publish_active = false;
	client.call(srv);
	client2.call(srv2);
	//compute distance in y direction to determin angle that needs tob
  	float dist_y = -(dist_x/FOCAL_LENGTH)*(left_u-320.0)*PIXEL_WIDTH + 0.5*BASELINE;
	float theta = atan(dist_y/dist_x) + robot_heading;
	float x_cone = robot_x + dist_x*cos(theta);
	float y_cone = robot_y + dist_x*sin(theta);
		float sum;
	for (int i = 0; i<15; i++){
		sum += distances[i];
	}
	float ave = sum/15.0;
	//if the new distance is drastically different from the old distances, ignore it
	if (abs(ave-dist_x) > 1.0){
		return;
	}
	//if there are consistently far distances, switch back to robot commands from motion controller
	if (ave > 12.0){
		srv.request.publish_active = true;
		client.call(srv);
		srv2.request.publish_active = true;
		client2.call(srv);
		return;	
	}
	//if there are no new commands in two seconds, switch back to robot commands from motion controller 
	if (ros::Time::now().sec-new_time > 2){
		srv.request.publish_active = true;
		client.call(srv);
		srv2.request.publish_active = true;
		client2.call(srv);
		return;
	}
	//if cone in view has already been seen, ignore it and switch to robot commands from motion controller
	for (int i = 0; i < num_cones; i++){
		if ((abs(x_cone - boost::get<0>(cones[i])) < 3.0) || (abs(y_cone - boost::get<1>(cones[i])) < 3.0)){
			srv.request.publish_active = true;
			client.call(srv);
			return;
		}		
	}
	geometry_msgs::Pose heading;
	//if the cone is robot is close enough to cone, switch back to robot commands from motion controller, and print location of cone
	if (dist_x < 1.4){
		cout << x_cone << " " << y_cone <<"\n";
		cones[num_cones] = (x_cone, y_cone);
		num_cones ++;
		if (dist_x < 1.1){
                //heading.position.x = -1.0;
                //heading_pub_.publish(heading);
			srv.request.publish_active = true;
			client.call(srv);
			srv2.request.publish_active = true;
			client2.call(srv2);
		}
	}
	//if cone still in sight and far enough away, command robot to go towards it a the distance it is away and the angle to get there
	else{
		heading.position.x = dist_x-1.0;
		heading.orientation.z = theta;
		heading_pub_.publish(heading);
	}
	
 
  }

};



	//Make visible as a ROS node
int main(int argc, char** argv) {
  ros::init(argc, argv, "ConePresentController");
  ros::NodeHandle n ("~");
  ConePresentController node(n, argv);
  ros::spin();

  return 0;
}
