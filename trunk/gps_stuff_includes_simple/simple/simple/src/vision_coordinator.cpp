/* node was never completed due to problems with using the Biclops head over a serial-to-USB
 * adapter. As it stands, it would set the head to pan continuously. 
 *
 * Nathan Bricault
 */

#include "ros/ros.h"
#include <DPPanTilt_msgs/SetAbsolutePosition.h>
int main ( int argc, char **argv ) 
{
  ros::init(argc, argv, "vision_coordinator"); 

  ros::NodeHandle n; 

  ros::Publisher position_pub = n.advertise<DPPanTilt_msgs::SetAbsolutePosition>("/PanTilt/set_absolute_position",100); 
  
  ros::Rate loop_rate(0.1); 
  
  bool flip = true; 

  while(ros::ok())
    {
      float pan; 
      float tilt =0; 
      if (flip) {
	pan = 2.75; 
      } else {
	pan = -2.75;
      }

      DPPanTilt_msgs::SetAbsolutePosition msg; 
      msg.panPosition = pan; 
      msg.tiltPosition = tilt; 
      flip = !flip; 
      position_pub.publish(msg); 
      ros::spinOnce(); 
      loop_rate.sleep(); 
    }

  return 0; 
}
