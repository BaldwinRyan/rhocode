#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include <sstream>
#include <math.h>

#define PI 3.1415926536

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps_test_input");

  ros::NodeHandle n;


  ros::Publisher fix_pub = n.advertise<sensor_msgs::NavSatFix>("/android/fix", 1);
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/android/imu",1);

  ros::Rate loop_rate(.5);


  while (ros::ok())
    {
      /**
       * This is a message object. You stuff it with data, and then publish it.
       */
      sensor_msgs::NavSatFix fix_msg;
      sensor_msgs::Imu imu_msg; 

      fix_msg.latitude = 0;
      fix_msg.longitude = 0; 

      imu_msg.orientation.z = sin(0); 

            //ROS_INFO("%s", msg.data.c_str());

      imu_pub.publish(imu_msg);
      fix_pub.publish(fix_msg); 

      ros::spinOnce();

      loop_rate.sleep();
    }


  return 0;
}
