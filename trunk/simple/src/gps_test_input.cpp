/* This node publisheds to the same topic as the actual GPS unit. 
 * It is used for debugging interactions between gps_queue and motion_controller
 * without the need to be connected to the GPS hardware. 
 *
 * Nathan Bricault
 */


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include <sstream>
#include <math.h>
#include "simple/controller_pub.h"
#define PI 3.1415926536

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps_test_input");

  ros::NodeHandle n;


  ros::Publisher fix_pub = n.advertise<sensor_msgs::NavSatFix>("/fix", 1);
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/android/imu",1);
  ros::ServiceClient client = n.serviceClient<simple::controller_pub>("controller_pub"); 
  
  //Publish a GPS message every 4 seconds
  ros::Rate loop_rate(.25);
  
  simple::controller_pub srv; 
  srv.request.publish_active = false; 
  client.call(srv); 

  int count = 0; 
  while (ros::ok())
    {
      /**
       * This is a message object. You stuff it with data, and then publish it.
       */
      sensor_msgs::NavSatFix fix_msg;
      sensor_msgs::Imu imu_msg; 

      fix_msg.latitude = count;
      fix_msg.longitude = count; 

      imu_msg.orientation.z = sin(0); 

            //ROS_INFO("%s", msg.data.c_str());

      imu_pub.publish(imu_msg);
      fix_pub.publish(fix_msg); 

      ros::spinOnce();
      count++; 
      loop_rate.sleep();
    }


  return 0;
}
