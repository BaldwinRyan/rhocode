/**
 * Dirk Ruiken
 * Laboratory for Perceptual Robotics, University of Massachusetts Amherst
 * 02/2012
 * Joystick control interface for uBot
 **/

#ifndef JOYSTICKCONTROL_H
#define JOYSTICKCONTROL_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <ubot_msgs/DriveCommands.h>
#include <ubot_msgs/SetBalancer.h>
#include <ubot_msgs/SetStationkeep.h>
#include <ubot_msgs/SetBalancerAngleOffset.h>
#include <ubot_msgs/BalancerStatus.h>
#include <ubot_msgs/JointPositions.h>

#include <ubot_controllers/ProneToBalancingAction.h>

//H+T
#include <vector>
#include <geometry_msgs/PoseArray.h>

#include <uBotCartesianPositionController/ubot_arm_cartesian_positionAction.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
//H+T-end


#define JOYSTICK_HZ 5.0 // Dirk and I tested with 5.0
#define ANGLE_OFFSET_DELTA 0.8
#define JOYSTICK_DEADBAND 0.35

#define MESSAGE_PUBLISH_RATE_HZ 10
#define NUMBER_OF_JOYSTICK_AXES 6
#define NUMBER_OF_JOYSTICK_BUTTONS 15
#define MAX_VELOCITY 0.45
#define MAX_TURNRATE 0.5


//H+T
#define DELTA_X 0.07
#define DELTA_Y 0.07
#define DELTA_Z 0.07
#define ST_DEFAULT_URDF "/home/hjung/UMass/ROS/ubot_cfg/ubot5/urdf/ubot5.urdf"
//H+T-end


class JoystickControl
{

  enum Mode {
    DRIVE_MODE = 0,
    //CARTESIAN_RIGHT_MODE,
    //CARTESIAN_LEFT_MODE,
    CARTESIAN_BOTH_MODE
  };

  enum Button {
    A_BUTTON = 0,
    B_BUTTON,
    X_BUTTON,
    Y_BUTTON,
    LB_BUTTON,
    RB_BUTTON,
    BACK_BUTTON,
    START_BUTTON,
    XBOX_BUTTON,
    LEFT_STICK,
    RIGHT_STICK,
    POV_LEFT,
    POV_RIGHT,
    POV_UP,
    POV_DOWN
  };

  enum Axis {
    LEFT_X = 0,
    LEFT_Y,
    LT,
    RIGHT_X,
    RIGHT_Y,
    RT
  };


public:
  JoystickControl();
  ~JoystickControl();

private:

  ros::NodeHandle handle_;

  ros::Publisher drive_pub_;
  ros::Publisher angle_offset_pub_;
  ros::Subscriber joystick_sub_;
  ros::Subscriber balancer_status_sub_;
  ros::Subscriber jointPositions_sub_;

  ros::Timer timer_;

  actionlib::SimpleActionClient<ubot_controllers::ProneToBalancingAction>* ac_pushup_;

  bool receivedJoystickMsg_;

  int controlMode_;
  std::vector<float> joystickAxes_;
  std::vector<int> joystickButtons_;
  std::vector<int> joystickButtonsLast_;

  double angle_offset_;
  bool left_button_pressed_;
  bool right_button_pressed_;
	bool joystick_pressed_one_step_before_;

  //callbacks
  void timerCallback(const ros::TimerEvent& e);
  void joystickCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void balancerStatusCallback(const ubot_msgs::BalancerStatus::ConstPtr& msg);
  void jointPositionsCallback(const ubot_msgs::JointPositionsConstPtr& msg);

  /**
   * change the angle offset by amount change
   * @params change in degrees
   */
  void changeAngleOffset(double change);
  void pushUp();
  void pushDown();

  //H+T


  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_left_;
  KDL::Chain kdl_chain_right_;
  KDL::ChainFkSolverPos_recursive* left_fk_pos_solver_; //
  KDL::ChainFkSolverPos_recursive* right_fk_pos_solver_; //
  KDL::JntArray left_desired_joint_positions_; //
  KDL::JntArray right_desired_joint_positions_; //

  std::string root_name_;
  std::string right_tip_name_;
  std::string left_tip_name_;
  std::string file_name_;
  //std::string action_name_;

std::vector<actionlib::SimpleActionClient<uBotCartesianPositionController::ubot_arm_cartesian_positionAction>*> ac_;

  uBotCartesianPositionController::ubot_arm_cartesian_positionGoal goal_left_;
  uBotCartesianPositionController::ubot_arm_cartesian_positionGoal goal_right_;
  double left_hand_position_[3], right_hand_position_[3];

  void doneLeftServer(const actionlib::SimpleClientGoalState& state,
                      const uBotCartesianPositionController::ubot_arm_cartesian_positionResultConstPtr& result);
  void doneRightServer(const actionlib::SimpleClientGoalState& state,
                       const uBotCartesianPositionController::ubot_arm_cartesian_positionResultConstPtr& result);
  void activeLeftServer();
  void activeRightServer();
  void feedbackLeftServer(const uBotCartesianPositionController::ubot_arm_cartesian_positionFeedbackConstPtr& feedback);
  void feedbackRightServer(const uBotCartesianPositionController::ubot_arm_cartesian_positionFeedbackConstPtr& feedback);
  void executeBothAction(uBotCartesianPositionController::ubot_arm_cartesian_positionGoal goal_left,
                     uBotCartesianPositionController::ubot_arm_cartesian_positionGoal goal_right);
	void executeRightAction(uBotCartesianPositionController::ubot_arm_cartesian_positionGoal goal_right);
	void executeLeftAction(uBotCartesianPositionController::ubot_arm_cartesian_positionGoal goal_left);
	void cancelLeftAction();
	void cancelRightAction();
	void cancelBothAction();
  //H+T-end

};

#endif
