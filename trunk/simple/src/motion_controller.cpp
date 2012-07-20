#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <math.h>
#include "std_msgs/Bool.h"
#include <boost/lexical_cast.hpp>
#include "simple/GPS.h"
#include "simple/setGoal.h"
#include "simple/set_goal.h"
#define PI 3.1415926536
//#define EARTH 20925524.9 //in FEET
#define EARTH 6378100 //in METERS
#define PRECISION 6
#define PROXIMITY_THRESHOLD 3 //in METERS 
#define QUEUE_EMPTY 9000 // lat/lon value stored in final GPS queue message

//using namespace std; 

ros::Publisher goal_reached_pub;

double lat;
float lon;
float direction = 0;
float goal_lat = -1;//42.393892;//42.39463305;
float goal_lon = 1;//-72.528921;//-72.52801514;
double x_current;
double y_current; 
double x_goal;
double y_goal; 
bool at_goal = false; 
bool trip_complete = false; 
double rounded_value = 3.14159; 
float debug_goal_angle = 0; 
double cone_lat; 
double cone_lon; 
bool new_cone = false; 
ros::ServiceClient client;

float mapDistance(); 

double round(double input) 
{
  return floor(input*pow(10,PRECISION))/pow(10,PRECISION);
}


//Get next coordinate from the queue. Bool indicates whether the previous destination has been reached (false for first call)
void getNewDest(bool first) {
  simple::set_goal srv;
  srv.request.destination_reached = first; 
  if (client.call(srv)){
    goal_lat = srv.response.latitude; 
    goal_lon = srv.response.longitude; 
    ROS_INFO("requesting coordinate"); 
  } else {
    ROS_INFO("Queue failed to provide coordinate"); 
    getNewDest(first); 
  }
  
}

//Store latitude and longitude when a GPS message is recieved 
void locationCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  lat = (*msg).latitude;
  lon = (*msg).longitude;
  
  float dist = mapDistance(); 
  if( dist < PROXIMITY_THRESHOLD ) {
    at_goal = true; 
    /*  std_msgs::Bool msg;
    msg.data = true; 
    goal_reached_pub.publish(msg); 
    sleep(1); 
    */
    getNewDest(true); 
  }
}


//Store heading in radians from magnetic north 
void headingCallback(const sensor_msgs::Imu::ConstPtr& msg)
  {
    direction = 2*asin(msg -> orientation.z);
     if (msg->orientation.z  < 0 ) {
      direction = 2*PI+direction;
    }
    //Convert output to degrees
    //direction =direction*(180/PI);
  }

//Calculate surface distance between current location and goal 
   float mapDistance()
   { 
     double phi_current = (90 - lat)*PI/180;
     double phi_goal = (90 - goal_lat)*PI/180;
     double theta_current = -lon*PI/180;
     double theta_goal = -goal_lon*PI/180; 

     x_current = EARTH*cos(theta_current)*sin(phi_current);
     y_current = EARTH*sin(theta_current)*cos(phi_current); 

     x_goal = EARTH*cos(theta_goal)*sin(phi_goal);
     y_goal = EARTH*sin(theta_goal)*cos(phi_goal); 


     double cart_dist = sqrt(pow(x_goal-x_current,2)+pow(y_goal-y_current,2));
     double surf_dist = 2*EARTH*asin(cart_dist/(2*EARTH)); 
     
     return surf_dist;
   } 

//Determine the angle between the robot's current heading and the desired heading
float findAngle()
{
  float robot_angle = direction; 
  //float goal_angle = atan((x_goal-x_current)/(y_goal-y_current));
  float goal_angle = atan((goal_lat-lat)/(goal_lon-lon)); 
  debug_goal_angle = goal_angle; 
  if (goal_lon-lon == 0 && goal_lat-lat >= 0) {
    goal_angle = 0; 
  } else if (goal_lon-lon == 0 && goal_lat-lat < 0 ) { 
    goal_angle = PI; 
  } else if( goal_lon > lon ) {
    goal_angle += 3*PI/2; 
  } else  {
    goal_angle += PI/2; 
  }
  
  return (goal_angle-robot_angle); 
}

void nextGoalCallBack(const simple::GPS::ConstPtr& msg) {
  double temp_lat = msg->latitude; 
  double temp_lon = msg->longitude;

  if( temp_lat == QUEUE_EMPTY && temp_lon == QUEUE_EMPTY ) { 
    trip_complete = true; 
  } 
  goal_lat = temp_lat; 
  goal_lon = temp_lon; 
}

void coneCallBack(const std_msgs::Bool::ConstPtr& msg/*simple::GPS::ConstPtr& msg*/) { 
  cone_lat = lat;//msg->latitude; 
  cone_lon = lon; //msg->longitude; 
  new_cone = true; 
}

bool setGoalCallBack( simple::setGoal::Request &req, simple::setGoal::Response &res )
{
  double temp_lat = req.latitude; 
  double temp_lon = req.longitude;

  if( temp_lat == QUEUE_EMPTY && temp_lon == QUEUE_EMPTY ) { 
    trip_complete = true; 
  } 
  goal_lat = temp_lat; 
  goal_lon = temp_lon; 
  /*
  std::stringstream ss; 
  ss << "lat = " << goal_lat << " long = " << goal_lon; 
  std::string debug =(std::string) ss.str();
  //cout << goal_lat
  ROS_INFO(debug); 
  */
  res.success = true; 
  return true; 
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_controller");

  ros::NodeHandle n;
  
  ros::Publisher heading_pub = n.advertise<geometry_msgs::Pose>("current_heading",1);
  //  goal_reached_pub = n.advertise<std_msgs::Bool>("goal_reached",1); 

  ros::Subscriber fix_sub = n.subscribe("/fix", 1, locationCallback);
  ros::Subscriber imu_sub = n.subscribe("/android/imu", 1, headingCallback);
  //ros::Subscriber next_goal_sub = n.subscribe("next_goal", 5, nextGoalCallBack); 
  ros::Subscriber cone_location_sub = n.subscribe("cone_location",1, coneCallBack); 
  //  ros::ServiceServer service = n.advertiseService("set_goal", setGoalCallBack); 
  client = n.serviceClient<simple::set_goal>("set_goal"); 
  
  std::ofstream map_file; 
  map_file.open("output_map_data.csv");
  if(map_file.is_open() ) { 

    ROS_INFO("Output file opened"); 
    map_file << "latitude,longitude,cone,type\n"; 
  } else {
    ROS_INFO("File opening failed"); 
  }
  
  getNewDest(false); 
  //Recalculate heading at a frequency of 1 Hz
  ros::Rate loop_rate(5);

  while (ros::ok())
    {
      std::stringstream ss; 
      ss.precision(10); 
      //ROS_INFO("Print coordinates"); 
      ss << lat << "," << lon << "," << "0,T\n";
      if ( new_cone ) {
	new_cone = false; 
	ss << cone_lat << "," << cone_lon << ",1,W\n";
      }
      map_file << ss.str(); 
      //Command to be sent to motor translator
      geometry_msgs::Pose msg;
      /*
      if (atGoal()) {
	at_goal = true; 
      }*/
      //at_goal = false; 
      //Provide a direction command if the goal has not been reached
      if ( trip_complete ) {

	msg.position.x = -1; 
	msg.orientation.z = 0; 
	heading_pub.publish(msg); 
      } else /* if (!at_goal) */{

	//Calculate distance and angle to goal relative to current heading
      float distance = mapDistance();
      float angle = findAngle(); 
      
      //Publish directions in a Pose message
      msg.position.x = distance;
      msg.orientation.z = angle;
      //  msg.orientation.w = debug_goal_angle; 
      msg.orientation.y = goal_lat; 
      msg.orientation.x = goal_lon; 
      heading_pub.publish(msg);
      
      ros::spinOnce();
      

      //++count;
      
    } /*else {
    
    //Send a stop command if at the destination
    msg.position.x = 0;
    msg.orientation.z = 0; 
    heading_pub.publish(msg);

    
      }*/
      loop_rate.sleep();
    }
  map_file.close(); 
  return 0;
}

