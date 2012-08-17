/* This node reads input from the /joy topic (for use with the xbox controller) 
 * and translates it into commands for atrv jr, published on the /cmd_vel topic. 
 * Controls: 
 * Left joystick vertical axis - linear motion
 * Left joystick horizontal axis - angular motion
 * A Button - tap to set break (timing needs work)
 * B Button - tap to set publishing to /cmd_vel on/off (to allow autonomous operation);
 * X Button - hold while using joystick to operate robot at full velocity
 *
 * Change VELOCITY_SCALING_ANGULAR and VELOCITY_SCALING_LINEAR to adjust base velocity. 
 *
 * Additional note: robot will keep executing last command briefly when joystick is released
 * to center. This seems to be an artifact of the frequency of the update loop. This issue
 * could be corrected in the callback for joystick positions.  
 *
 * Nathan Bricault
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <math.h>
#include "std_msgs/Bool.h"
#include "simple/controller_pub.h"
#define PI 3.1415926536

//From joystick_control.h
#define JOYSTICK_HZ 5.0 // Dirk and I tested with 5.0
#define ANGLE_OFFSET_DELTA 0.8
#define JOYSTICK_DEADBAND 0.35

#define MESSAGE_PUBLISH_RATE_HZ 10
#define NUMBER_OF_JOYSTICK_AXES 6
#define NUMBER_OF_JOYSTICK_BUTTONS 15
#define MAX_VELOCITY 0.45
#define MAX_TURNRATE 0.5
#define VELOCITY_SCALING_ANGULAR 0.15
#define VELOCITY_SCALING_LINEAR 0.15

std::vector<float> joystickAxes_;
std::vector<int> joystickButtons_;
std::vector<int> joystickButtonsLast_;
bool receivedJoystickMsg_ = false; 
ros::Subscriber joystick_sub_;
ros::Publisher break_pub;
bool stopped = false; 
bool publishing = true; 
bool boost_active = false; 

void switchBrake( bool change ) {
  if( change ) {

    bool setbreak = change && !stopped; 
    stopped = !stopped; 
    std_msgs::Bool msg; 
    msg.data = setbreak; 
    break_pub.publish(msg); 
    sleep(0.030); 
  }  
}
/*
void switchPublish( bool change ) {
  if( change ) {

    bool setPublish = change && !; 
    stopped = !stopped; 
    std_msgs::Bool msg; 
    msg.data = setbreak; 
    break_pub.publish(msg); 
    sleep(0.030); 
  }  
}
*/

void joystickCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  if (joy->axes.size() < NUMBER_OF_JOYSTICK_AXES)
    {
      //  ROS_WARN( "Joystick Axes size mismatch: %d instead of %d", joy->axes.size(), NUMBER_OF_JOYSTICK_AXES );
      return;
    }

  if (joy->buttons.size() < NUMBER_OF_JOYSTICK_BUTTONS)
    {
      //ROS_WARN( "Joystick Button size mismatch: %d instead of %d", joy->buttons.size() , NUMBER_OF_JOYSTICK_BUTTONS );
      return;
    }

  joystickAxes_ = joy->axes;
  joystickButtons_ = joy->buttons;
  ROS_INFO("%4.3f",joystickButtons_[1]); 
  switchBrake( joystickButtons_[0] == 1 ); 
  boost_active =  (joystickButtons_[2] == 1 ) ; 
  //switchPublish( joystickButtons_[1] == 1 ); 
  if ( joystickButtons_[1] == 1 ) 
    {
      publishing = !publishing;
      sleep(0.030); 
    } 
  receivedJoystickMsg_ = true;
  ROS_INFO("%4.3f, %4.3f, %4.3f, %4.3f, %4.3f, %4.3f", joystickAxes_[0], joystickAxes_[1], joystickAxes_[2], joystickAxes_[3], joystickAxes_[4], joystickAxes_[5] );
  ROS_INFO("%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d", joystickButtons_[0], joystickButtons_[1], joystickButtons_[2], joystickButtons_[3], joystickButtons_[4],joystickButtons_[5], joystickButtons_[6], joystickButtons_[7], joystickButtons_[8], joystickButtons_[9], joystickButtons_[10], joystickButtons_[11], joystickButtons_[12], joystickButtons_[13], joystickButtons_[14]); 
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "joystick_controller");

  ros::NodeHandle n;

  
  ros::Publisher command_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  //ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/android/imu",1);
  joystick_sub_ = n.subscribe<sensor_msgs::Joy> ("/joy", 10, joystickCallback);
  ros::ServiceClient client = n.serviceClient<simple::controller_pub>("controller_pub"); 
  break_pub = n.advertise<std_msgs::Bool>("atrvjr/cmd_brake_power",1); 
  ros::Rate loop_rate(5);
  /*
  simple::controller_pub srv; 
  srv.request.publish_active = false; 
  //  client.call(srv); 
  */
  //int count = 0; 
  //  ros::spin(); 
  
  while (ros::ok())
  {
    float linear;
    float angular; 
    ros::spinOnce(); 
    if ((true == receivedJoystickMsg_)) {
      // filter axes with deadband filter
      for (int i=0; i<joystickAxes_.size(); i++) {
	if (fabs(joystickAxes_[i]) < JOYSTICK_DEADBAND) {
	  joystickAxes_[i] = 0.0;
	} else {
	  if (joystickAxes_[i] > 0.0)
	    joystickAxes_[i] = (joystickAxes_[i] - JOYSTICK_DEADBAND) / (1.0 - JOYSTICK_DEADBAND);
	  else
	    joystickAxes_[i] = (joystickAxes_[i] + JOYSTICK_DEADBAND) / (1.0 - JOYSTICK_DEADBAND);
	}
	if (i == 0) { 
	  linear = joystickAxes_[i]; 
	} else if ( i == 1 ) { 
	  angular = joystickAxes_[i]; 
	}
      }
    }
    
    
    if ( ! (boost_active == true ) )  {
      angular = angular*VELOCITY_SCALING_ANGULAR;
      linear = linear*VELOCITY_SCALING_LINEAR; 
    }

    geometry_msgs::Twist msg; 
    msg.linear.x = angular;//joystickAxes_[0]; 
    msg.angular.z = linear; // joystickAxes_[1]; 
    
    if( publishing ) {
      command_pub.publish(msg); 
    }
    loop_rate.sleep();
  }
  
  
  return 0;
}
