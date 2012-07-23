#include <DPPanTilt_msgs/SetAbsolutePosition.h>
#include <DPPanTilt_msgs/ReportDynamicInfo.h>
#include <DPPanTilt_msgs/ReportStaticInfo.h>
#include <DPPanTilt_msgs/Reset.h>
#include <DPPanTilt_msgs/HaltMotion.h>

#include <Bisight_server/BiclopsAccess.h>
#include <PMDUtils.h>
// for debugging:
#include <iostream>
#include <string.h>
#include <ros/ros.h>

class BiclopsServer {
 public:
   BiclopsServer(ros::NodeHandle& node_handle);
  ~BiclopsServer();
  void publishPositions();
  void publishLimits();
  void reset();
  void SetGoalCB(const DPPanTilt_msgs::SetAbsolutePosition::ConstPtr& msg);
  void HaltMotionCB(const DPPanTilt_msgs::HaltMotion::ConstPtr& msg);

 private:
  BiclopsAccess *biclops_access; 
  ros::NodeHandle m_nodeHandle;
  ros::Publisher  m_dynamic_pub;
  ros::Publisher  m_static_pub;
  ros::Subscriber  m_joint_sub;
  ros::Subscriber  m_joint_sub_halt;
  ros::Subscriber m_reset;

};
