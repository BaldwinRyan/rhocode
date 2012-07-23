#include <Bisight_server/BiclopsAccess.h>
#include <Bisight_server/BiclopsServer.h>
#include <PMDUtils.h>
// for debugging:
#include <iostream>
#include <string.h>
#include <ros/ros.h>

#include <DPPanTilt_msgs/SetAbsolutePosition.h>
#include <DPPanTilt_msgs/ReportDynamicInfo.h>
#include <DPPanTilt_msgs/ReportStaticInfo.h>
#include <DPPanTilt_msgs/Reset.h>
#include <DPPanTilt_msgs/HaltMotion.h>

#define BISIGHT_DEFAULT_HZ 3
#define PAN_MAX 2.8
#define TILT_MAX 0.95

BiclopsServer::BiclopsServer(ros::NodeHandle& node_handle) :
  m_nodeHandle(node_handle)
{
  // Connect to Bisight
  biclops_access = BiclopsAccess::instance();
   // Publishers : Only publish the most recent reading
    m_dynamic_pub = m_nodeHandle.advertise
      <DPPanTilt_msgs::ReportDynamicInfo>("/PanTilt/report_dynamic_info", 1);
    m_static_pub = m_nodeHandle.advertise
      <DPPanTilt_msgs::ReportStaticInfo>("/PanTilt/report_static_info", 1);
  // Subscribers : Only subscribe to the most recent instructions
  m_joint_sub_halt = m_nodeHandle.subscribe
    <DPPanTilt_msgs::HaltMotion>("/PanTilt/halt_motion", 1, &BiclopsServer::HaltMotionCB, this);
  m_joint_sub = m_nodeHandle.subscribe
    <DPPanTilt_msgs::SetAbsolutePosition>("/PanTilt/set_absolute_position", 1, &BiclopsServer::SetGoalCB, this);
  m_reset = m_nodeHandle.subscribe
    <DPPanTilt_msgs::Reset>("/PanTilt/reset", 1, &BiclopsServer::reset, this);

}

BiclopsServer::~BiclopsServer()
{
  delete biclops_access;
}

/** Callback for setting new Pan/Tilt Position */
void BiclopsServer::SetGoalCB(const DPPanTilt_msgs::SetAbsolutePosition::ConstPtr& msg) {
  double jnt[2];
  jnt[0] = msg->panPosition;
  if (jnt[0] > PAN_MAX) jnt[0] = PAN_MAX;
  if (jnt[0] < -PAN_MAX) jnt[0] = -PAN_MAX;
  jnt[0] += .11;  //Correct the pan position so that the center is in the right place
  jnt[1] = msg->tiltPosition / .988;  //Scale the tilt position
  if (jnt[1] > TILT_MAX) jnt[1] = TILT_MAX;
  if (jnt[1] < -TILT_MAX) jnt[1] = -TILT_MAX;
  ROS_INFO("Set positions : %f, %f",jnt[0],jnt[1]);  
  biclops_access->setJointPosition(jnt);
}

/** Callback for HaltMotion command -> interpreted as PARK command */
void BiclopsServer::HaltMotionCB(const DPPanTilt_msgs::HaltMotion::ConstPtr& msg) {
  biclops_access->setMode(BiclopsAccess::PARKED);
}

void BiclopsServer::publishPositions()
{
  DPPanTilt_msgs::ReportDynamicInfo reportDynamic;
  reportDynamic.header.stamp = ros::Time::now();
  reportDynamic.currentlyInPositionControl = true;	
  reportDynamic.panVelocityInRadiansPerSec = 0.0;
  reportDynamic.tiltVelocityInRadiansPerSec = 0.0;
  reportDynamic.panPositionInRadians = biclops_access->getJointPosition(0) - 0.11;  //Correct the pan position so that the center is in the right place
  reportDynamic.tiltPositionInRadians = biclops_access->getJointPosition(1) * .988;  //Scale the tilt position
  
  m_dynamic_pub.publish(reportDynamic);	
}

void BiclopsServer::publishLimits()
{
  DPPanTilt_msgs::ReportStaticInfo reportStatic;
  reportStatic.panMinimumValidPositionInRadians = -PAN_MAX;
  reportStatic.tiltMinimumValidPositionInRadians = -TILT_MAX;
  reportStatic.panMaximumValidPositionInRadians = PAN_MAX;
  reportStatic.tiltMaximumValidPositionInRadians = TILT_MAX;

  
  m_static_pub.publish(reportStatic);	
}


void BiclopsServer::reset()
{
  biclops_access::home();
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "Biclops_server");
  ros::NodeHandle n("");
  BiclopsServer biclopsserver(n);
  // Query for polling frequency
  int hz;
    
  if (!n.getParam("servoRate", hz)) {
    ROS_INFO("Could not retrieve 'servoRate parameter', setting to default value of %d", BISIGHT_DEFAULT_HZ);       
    hz = BISIGHT_DEFAULT_HZ;
  }
    
  ROS_INFO("Starting Biclops Server at %d Hz.", hz);
  
  ros::Rate loop_rate(hz);
   
  ROS_INFO("Connected.");
  while(ros::ok())
    {
      ros::spinOnce();
      biclopsserver.publishPositions();
      biclopsserver.publishLimits();
      
      // This will adjust as needed per iteration
      loop_rate.sleep();
    }
  return 0;
}
