#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <math.h>
#include <DPPanTilt_msgs/SetAbsolutePositionPan.h>
#include <DPPanTilt_msgs/SetAbsolutePositionTilt.h>

/**
 * PTU46 ROS Package
 * Copyright (C) 2009 Erik Karulf (erik@cse.wustl.edu)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */


int main(int argc, char** argv) {
    ros::init(argc, argv, "ptu_test");
    ros::NodeHandle n;

 // Publishers : Only publish the most recent reading
    ros::Publisher  m_pan_pub = n.advertise<DPPanTilt_msgs::SetAbsolutePositionPan>("/UpperPanTilt/set_absolute_position_pan", 1);
    ros::Publisher  m_tilt_pub = n.advertise<DPPanTilt_msgs::SetAbsolutePositionTilt>("/UpperPanTilt/set_absolute_position_tilt", 1);

    int hz = 40;
    ros::Rate loop_rate(hz);

   
    ros::Time start = ros::Time::now();


    while (ros::ok()) {

float tmp = (ros::Time::now() - start).toSec();
    printf("%4.3f\n", tmp);

    double pan = sin(tmp) *1.57;
    double tilt = cos(tmp) * -0.8;

    //double panspeed = 0.9;
    //double tiltspeed = 0.6;


 // Publish Position & Speed
    DPPanTilt_msgs::SetAbsolutePositionPan panPos;
    panPos.panPositionInRadians = pan;
    m_pan_pub.publish(panPos);

    DPPanTilt_msgs::SetAbsolutePositionTilt tiltPos;
    tiltPos.tiltPositionInRadians = tilt;
    m_tilt_pub.publish(tiltPos);

       
	   


        // Process a round of subscription messages
        ros::spinOnce();

        // This will adjust as needed per iteration
	loop_rate.sleep();
    }


    return 0;
}
