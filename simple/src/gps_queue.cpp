/* This node loads the list of user-defined GPS waypoints and 
 * provides them to motion_controller when a new destination
 * is requested. 
 *
 * Nathan Bricault
 */


//#include <tuple>
#include "boost/tuple/tuple.hpp"
#include <list> 
//#include <ifstream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <fstream>
#include <iostream>
#include "simple/GPS.h"
#include "std_msgs/Bool.h"
#include "simple/setGoal.h"
#include "simple/set_goal.h"
#include "ros/package.h"

using namespace std;

#define QUEUE_EMPTY 9000

list<boost::tuple<double,double> > L; 
ros::Publisher destination_pub;

//Read in the desired GPS coordinates from file and load them into a list
void loadQueue() {

  std::string directory = ros::package::getPath("simple"); 
  std::string input_file = directory + "/bin/gps_input_num.txt"; 
  
  ROS_INFO("loading file");
  string line;
  ifstream file;
  ROS_INFO("File assigned");
  file.open(input_file.c_str());
  if (file.is_open())
    {
      ROS_INFO("File opened"); 
      while ( file.good() ) 
	{
	  ROS_INFO("Retrieve line");
	  getline(file, line);

	  if(!( line.substr(0,1) == "#") && line.length() > 0 ) {
	    size_t end = line.find("/");
	    string lat_str = line.substr(0,end); 
	    string lon_str = line.substr(end+1);
	    
	    char *a = (char*)lat_str.c_str();
	    char *b = (char*)lon_str.c_str();
	    double new_lat = atof( a ); 
	    double new_lon = atof( b ); 
	    boost::tuple<double,double> coord (new_lat,new_lon);
	    L.push_back( coord ); 
	  }
	}
      
      boost::tuple<double,double> queue_end (QUEUE_EMPTY,QUEUE_EMPTY); 
      L.push_back( queue_end ); 
    } else {
    ROS_INFO("File opening failed"); 
  }
}

//Publish the next coordinate from the queue if available
void publishNext() {
  
  simple::GPS gps_msg; 
  boost::tuple<double,double> coord;
  ROS_INFO("called"); 
  if(!L.empty()){
    coord = L.front();
    L.pop_front(); 
    gps_msg.latitude =    boost::get<0>(coord); 
    gps_msg.longitude =    boost::get<1>(coord); 
  } else {
    //debugging values
    gps_msg.latitude = 314; 
    gps_msg.longitude = 314; 

  }
  ROS_INFO("Publishing"); 
  destination_pub.publish(gps_msg);
  
}

//Set new GPS goal from the queue when a given destination is reached
void goalCallback(const std_msgs::Bool::ConstPtr& msg) {
  publishNext();
  ROS_INFO("Published"); 
}
/*
void serveNext() 
{
  
  simple::setGoal srv; 
  boost::tuple<double,double> coord;
  ROS_INFO("called"); 
  if(!L.empty()){
    coord = L.front();
    L.pop_front(); 
    srv.request.latitude =    boost::get<0>(coord); 
    srv.request.longitude =    boost::get<1>(coord); 
  } else {
    //debugging values
    srv.request.latitude = 314; 
    srv.request.longitude = 314; 

  }
  ROS_INFO("Publishing"); 
  //destination_pub.publish(gps_msg);
  client.call(srv); 
}
*/

 //If the service call indicates that a new destination has been reached, set a new goal. 
 //Otherwise, maintain original goal. 
bool setGoalCallBack( simple::set_goal::Request &req, simple::set_goal::Response &res )
{
  if ( req.destination_reached ) {
    L.pop_front(); 
  } 

  boost::tuple<double,double> coord;
  coord = L.front();
  
  res.latitude = boost::get<0>(coord);
  res.longitude = boost::get<1>(coord);
  
  return true; 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps_queue");

  ros::NodeHandle n;
  ros::Subscriber goal_reached_sub; 
  goal_reached_sub = n.subscribe("goal_reached",1,goalCallback); 
  //destination_pub = n.advertise<simple::GPS>("next_goal", 10);
  // client = n.serviceClient<simple::setGoal>("set_goal"); 
  ros::ServiceServer service = n.advertiseService("set_goal", setGoalCallBack); 
  
  loadQueue();

  ros::spin();
  return 0;
}





















