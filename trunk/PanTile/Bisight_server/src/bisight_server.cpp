#include <string>
#include <ros/ros.h>
#include <Bisight_server/BiclopsAccess.h>
#include <sensor_msgs/JointState.h>


#include <DPPanTilt_msgs/SetAbsolutePositionPan.h>
#include <DPPanTilt_msgs/SetAbsolutePositionTilt.h>
#include <DPPanTilt_msgs/ReportDynamicInfo.h>
#include <DPPanTilt_msgs/ReportStaticInfo.h>
#include <DPPanTilt_msgs/Reset.h>
#include <DPPanTilt_msgs/HaltMotion.h>
//Inserted

BisightServer::BisightServer(ros::NodeHandle& node_handle) :
    m_nodeHandle(node_handle)
{
   // Subscribers : Only subscribe to the most recent instructions
    m_joint_sub_reset = m_node.subscribe
      <DPPanTilt_msgs::Reset>("/UpperPanTilt/reset", 1, &Bisight_Node::ResetCB, this);
    m_joint_sub_halt = m_node.subscribe
      <DPPanTilt_msgs::HaltMotion>("/UpperPanTilt/halt_motion", 1, &Bisight_Node::HaltMotionCB, this);
    m_joint_sub_pan = m_node.subscribe
      <DPPanTilt_msgs::SetAbsolutePositionPan>("/UpperPanTilt/set_absolute_position_pan", 1, &Bisight_Node::SetGoalPanCB, this);
    m_joint_sub_tilt = m_node.subscribe
      <DPPanTilt_msgs::SetAbsolutePositionTilt>("/UpperPanTilt/set_absolute_position_tilt", 1, &Bisight_Node::SetGoalTiltCB, this);
}

 /** Callback for getting new Pan Position */
  void BisightServer::SetGoalPanCB(const DPPanTilt_msgs::SetAbsolutePositionPan::ConstPtr& msg) {
    //Begin inserted code	
    int jnt = 0; 
    if(joint_commands[jnt] != joint_last_set[jnt])
      {
	pthread_mutex_lock(&position_mutex);
	double val = joint_commands[jnt];
	joint_last_set[jnt] = val;
	updated = true;
	pthread_mutex_unlock(&position_mutex);

	double unit = radiansToUnit(val, jnt);
	
	PMDAxisControl::Profile p;
	PMDAxisControl *axis = _biclops->GetAxis(jnt);

	axis->SetPosition(axis->UnitsToCounts(unit));
	axis->GetProfile(p);
	p.pos = unit;
	axis->SetProfile(p);

	axis->Move(false, true);
      }
    if(updated)
      {
	usleep(BICLOPS_UPDATE_SLEEP_TIME_MS*1000);
	updated = false;
      }
    else
      {
	usleep(0);
      }
    //End inserted code
    
    /*
      if (! ok())
      return;

      desired_joint_positions[PAN]  = msg->panPositionInRadians;
    */
  }

  /** Callback for getting new Tilt Position */
  void BisightServer::SetGoalTiltCB(const DPPanTilt_msgs::SetAbsolutePositionTilt::ConstPtr& msg) {
    if (! ok())
      return;
   
    desired_joint_positions[TILT] = -msg->tiltPositionInRadians;	//invert to match ARM-S GFE
  }
