/**
 * Dirk Ruiken
 * Laboratory for Perceptual Robotics, University of Massachusetts Amherst
 * 02/2012
 * Joystick control interface for uBot
 **/

#include <ubot_joystick/joystick_control.h>



JoystickControl::JoystickControl()
{
  //subscribe to data

  drive_pub_ = handle_.advertise<ubot_msgs::DriveCommands>("/uBot/drive_commands", 1);
  angle_offset_pub_ = handle_.advertise<ubot_msgs::SetBalancerAngleOffset>("/uBot/set_angle_offset", 1);

  joystick_sub_ = handle_.subscribe<sensor_msgs::Joy> ("/joy", 10, &JoystickControl::joystickCallback, this);
  balancer_status_sub_ = handle_.subscribe("/uBot/balancer_status", 1, &JoystickControl::balancerStatusCallback, this);

	ROS_INFO("Pushup server attempted...");
  ac_pushup_ = new actionlib::SimpleActionClient<ubot_controllers::ProneToBalancingAction>("/uBot/controllers/ubot5_prone_to_balancing", true);
/*
  if ( !ac_pushup_->waitForServer() ) {
    ROS_WARN("No pushup server");
  }
*/


  //H+T
  //jointPositions_sub_= handle_.subscribe<ubot_msgs::JointPositions>("/uBot/joint_positions", 1, &JoystickControl::jointPositionsCallback, this);
//jointPositions_sub_= handle_.subscribe<ubot_msgs::JointPositions>("/uBot/joint_desired_positions", 1, &JoystickControl::jointPositionsCallback, this);
	ROS_INFO("Cartesian controller servers attempted...");
  ac_.push_back(
      new actionlib::SimpleActionClient<uBotCartesianPositionController::ubot_arm_cartesian_positionAction>(
          "uBotEndPositionControllerLeft", true));
  ac_.push_back(
      new actionlib::SimpleActionClient<uBotCartesianPositionController::ubot_arm_cartesian_positionAction>(
          "uBotEndPositionControllerRight", true));

  //ROS_INFO("Waiting for action server to start.");
  //ac_[0]->waitForServer(); // will wait for infinite time
  //ac_[1]->waitForServer(); // will wait for infinite time

  left_desired_joint_positions_.resize(6);
  right_desired_joint_positions_.resize(6);

  //file_name_ = ST_DEFAULT_URDF;
  root_name_ = "base_footprint";
  left_tip_name_ = "left_hand";
  right_tip_name_ = "right_hand";

  if (!ros::param::get("~urdf_file", file_name_))
  {
    //ROS_WARN("%s: Could not retrieve 'urdf_file' parameter, setting to default value of '%s'", action_name_.c_str(), ST_DEFAULT_URDF);
		ROS_ERROR("Failed to retrieve urdf file");
    file_name_ = ST_DEFAULT_URDF;
  }

  if (!kdl_parser::treeFromFile(file_name_, kdl_tree_))
  {
    ROS_ERROR("Failed to construct kdl tree");
  }

  if (!kdl_tree_.getChain(root_name_, left_tip_name_, kdl_chain_left_))
  {
    ROS_ERROR("Could not load kdl tree");
  }

  if (!kdl_tree_.getChain(root_name_, right_tip_name_, kdl_chain_right_))
  {
    ROS_ERROR("Could not load kdl tree");
  }

  left_fk_pos_solver_ = new KDL::ChainFkSolverPos_recursive(kdl_chain_left_);
  right_fk_pos_solver_ = new KDL::ChainFkSolverPos_recursive(kdl_chain_right_);

  //H+T-end


  receivedJoystickMsg_ = false;
  controlMode_ = CARTESIAN_BOTH_MODE;//CARTESIAN_RIGHT_MODE;

  joystickAxes_.resize(NUMBER_OF_JOYSTICK_AXES, 0.0);
  joystickButtons_.resize(NUMBER_OF_JOYSTICK_BUTTONS, 0);
  joystickButtonsLast_.resize(NUMBER_OF_JOYSTICK_BUTTONS, 0);


  ROS_INFO("Axes: %d", joystickAxes_.size());
  ROS_INFO("Buttons: %d", joystickButtons_.size());

  angle_offset_ = 0.0;
  left_button_pressed_ = false;
  right_button_pressed_ = false;
	joystick_pressed_one_step_before_ = false;

  // create and start timer
  timer_ = handle_.createTimer(ros::Duration(1.0/JOYSTICK_HZ), &JoystickControl::timerCallback, this);

	ROS_INFO("Everything started up...");
}

JoystickControl::~JoystickControl(void)
{

}

void JoystickControl::joystickCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  if (joy->axes.size() < NUMBER_OF_JOYSTICK_AXES)
  {
    ROS_WARN( "Joystick Axes size mismatch: %d instead of %d", joy->axes.size(), NUMBER_OF_JOYSTICK_AXES );
    return;
  }

  if (joy->buttons.size() < NUMBER_OF_JOYSTICK_BUTTONS)
  {
    ROS_WARN( "Joystick Button size mismatch: %d instead of %d", joy->buttons.size() , NUMBER_OF_JOYSTICK_BUTTONS );
    return;
  }

  joystickAxes_ = joy->axes;
  joystickButtons_ = joy->buttons;

  receivedJoystickMsg_ = true;

}

void JoystickControl::balancerStatusCallback(const ubot_msgs::BalancerStatus::ConstPtr& msg)
{
  angle_offset_ = msg->angle_offset;
}


//////////////////////////////////////////////////
//H+T



void JoystickControl::jointPositionsCallback(const ubot_msgs::JointPositions::ConstPtr& msg)
{


  left_desired_joint_positions_(0) = 0.0;
  left_desired_joint_positions_(1) = msg->positions[2];
  left_desired_joint_positions_(2) = msg->positions[4];
  left_desired_joint_positions_(3) = msg->positions[6];
  left_desired_joint_positions_(4) = msg->positions[8];
  left_desired_joint_positions_(5) = msg->positions[10];

  right_desired_joint_positions_(0) = 0.0;
  right_desired_joint_positions_(1) = msg->positions[2];
  right_desired_joint_positions_(2) = msg->positions[3];
  right_desired_joint_positions_(3) = msg->positions[5];
  right_desired_joint_positions_(4) = msg->positions[7];
  right_desired_joint_positions_(5) = msg->positions[9];

  KDL::Frame current_frame; // result frame
  int ret = 0;

  ret = left_fk_pos_solver_->JntToCart(left_desired_joint_positions_, current_frame);
  if (ret < 0)
    ROS_WARN("FK error");

  left_hand_position_[0] = current_frame.p.x();
  left_hand_position_[1] = current_frame.p.y();
  left_hand_position_[2] = current_frame.p.z();

  ret = right_fk_pos_solver_->JntToCart(right_desired_joint_positions_, current_frame);
  if (ret < 0)
    ROS_WARN("FK error");

  right_hand_position_[0] = current_frame.p.x();
  right_hand_position_[1] = current_frame.p.y();
  right_hand_position_[2] = current_frame.p.z();

  //ROS_INFO("left  x:%f, y:%f, z:%f",left_hand_position_[0],left_hand_position_[1],left_hand_position_[2]);
  //ROS_INFO("right x:%f, y:%f, z:%f",right_hand_position_[0],right_hand_position_[1],right_hand_position_[2]);
}

void JoystickControl::doneLeftServer(const actionlib::SimpleClientGoalState& state,
                                     const uBotCartesianPositionController::ubot_arm_cartesian_positionResultConstPtr& result)
{
  ROS_INFO("[LEFT done] %s, %d", state.toString().c_str(), result->success);
  // print info about when the feature server finishes processing
}

void JoystickControl::doneRightServer(const actionlib::SimpleClientGoalState& state,
                                      const uBotCartesianPositionController::ubot_arm_cartesian_positionResultConstPtr& result)
{
  ROS_INFO("[RIGHT done] %s, %d", state.toString().c_str(), result->success);
  // print info about when the feature server finishes processing
}

void JoystickControl::activeLeftServer()
{
  // print info regarding the feature server being active
}

void JoystickControl::activeRightServer()
{
  // print info regarding the feature server being active
}

void JoystickControl::feedbackLeftServer(const uBotCartesianPositionController::ubot_arm_cartesian_positionFeedbackConstPtr& feedback)
{
  //state_left_ = feedback->state;
  //left_error_ = feedback->error;
  //left_error_dot_ = feedback->error_dot;
  //ROS_INFO("[LEFT feedback] %d, %f, %f", state_left_, left_error_, left_error_dot_);
  //cout << "feedback coming from left server" << endl;
}

void JoystickControl::feedbackRightServer(const uBotCartesianPositionController::ubot_arm_cartesian_positionFeedbackConstPtr& feedback)
{
  //state_right_ = feedback->state;
  //right_error_ = feedback->error;
  //right_error_dot_ = feedback->error_dot;
  //ROS_INFO("[RIGHT feedback] %d, %f, %f", state_right_, right_error_, right_error_dot_);
  //cout << "feedback coming from right server" << endl;
}

void JoystickControl::executeBothAction(uBotCartesianPositionController::ubot_arm_cartesian_positionGoal goal_left, uBotCartesianPositionController::ubot_arm_cartesian_positionGoal goal_right)
{
  ROS_INFO("Executing the arm movements.");
//  ac_[0]->sendGoal(goal_left, boost::bind(&JoystickControl::doneLeftServer, this, _1, _2),
//                   boost::bind(&JoystickControl::activeServer, this),
//                   boost::bind(&JoystickControl::feedbackLeftServer, this, _1));
//  ac_[1]->sendGoal(goal_right, boost::bind(&JoystickControl::doneRightServer, this, _1, _2),
//                   boost::bind(&JoystickControl::activeServer, this),
//                   boost::bind(&JoystickControl::feedbackRightServer, this, _1));
  ac_[0]->sendGoal(goal_left);
  ac_[1]->sendGoal(goal_right);

  ROS_INFO("Executing the arm movements completed.");
}

void JoystickControl::executeRightAction(uBotCartesianPositionController::ubot_arm_cartesian_positionGoal goal_right)
{
		//ROS_INFO("Executing the arm movements.");
		ac_[1]->sendGoal(goal_right);
	/*	ac_[1]->sendGoal(goal_right,
				 boost::bind(&JoystickControl::doneRightServer, this, _1, _2),
				 boost::bind(&JoystickControl::activeRightServer, this),
				 boost::bind(&JoystickControl::feedbackRightServer, this, _1)
		);*/
		//ROS_INFO("Executing the arm movements completed.");
}

void JoystickControl::executeLeftAction(uBotCartesianPositionController::ubot_arm_cartesian_positionGoal goal_left)
{
		//ROS_INFO("Executing the arm movements.");
		ac_[0]->sendGoal(goal_left);
	/*	ac_[1]->sendGoal(goal_right,
				 boost::bind(&JoystickControl::doneRightServer, this, _1, _2),
				 boost::bind(&JoystickControl::activeRightServer, this),
				 boost::bind(&JoystickControl::feedbackRightServer, this, _1)
		);*/
		//ROS_INFO("Executing the arm movements completed.");
}


void JoystickControl::cancelLeftAction()
{
		ac_[0]->cancelGoal();
}

void JoystickControl::cancelRightAction()
{
		ac_[1]->cancelGoal();
}

void JoystickControl::cancelBothAction()
{
		ac_[0]->cancelGoal();
		ac_[1]->cancelGoal();
}

//H+T-end
///////////////////////////////////








void JoystickControl::changeAngleOffset(double change) {
  ubot_msgs::SetBalancerAngleOffset msg;
  msg.stamp = ros::Time::now();
  msg.angle_offset = (angle_offset_) + (change * M_PI / 180.0);

  angle_offset_pub_.publish(msg);

  ROS_INFO("Angle offset: %3.2f degree (%+4.3f rad)", msg.angle_offset*180.0/M_PI, msg.angle_offset);
}

void JoystickControl::pushUp() {
  ROS_INFO("Prone --> Balancing");
  actionlib::SimpleActionClient<ubot_controllers::ProneToBalancingAction> ac_pushup("/uBot/controllers/ubot5_prone_to_balancing", true);
  if ( ac_pushup.waitForServer(ros::Duration(5.0))) {
    ubot_controllers::ProneToBalancingGoal goal;
    goal.going_up = true;
    ac_pushup_->sendGoal(goal);
  } else {
    ROS_WARN("Cannot connect to pushup server.");
  }
}

void JoystickControl::pushDown() {
  ROS_INFO("Balancing --> Prone");
  actionlib::SimpleActionClient<ubot_controllers::ProneToBalancingAction> ac_pushup("/uBot/controllers/ubot5_prone_to_balancing", true);
  if ( ac_pushup.waitForServer(ros::Duration(5.0))) {
    ubot_controllers::ProneToBalancingGoal goal;
    goal.going_up = false;
    ac_pushup_->sendGoal(goal);
  } else {
    ROS_WARN("Cannot connect to pushup server.");
  }
}

void JoystickControl::timerCallback(const ros::TimerEvent& e)
{
  if ((true == receivedJoystickMsg_))
  {

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
    }

    //Debug output of all buttons and axes
    //ROS_INFO("%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d", joystickButtons_[0], joystickButtons_[1], joystickButtons_[2], joystickButtons_[3], joystickButtons_[4], joystickButtons_[5], joystickButtons_[6], joystickButtons_[7], joystickButtons_[8], joystickButtons_[9], joystickButtons_[10], joystickButtons_[11], joystickButtons_[12], joystickButtons_[13], joystickButtons_[14]);
    ROS_INFO("%4.3f, %4.3f, %4.3f, %4.3f, %4.3f, %4.3f", joystickAxes_[0], joystickAxes_[1], joystickAxes_[2], joystickAxes_[3], joystickAxes_[4], joystickAxes_[5] );





    //---- Buttons ----

    //A
    if (joystickButtonsLast_[A_BUTTON] != joystickButtons_[A_BUTTON]) {
      if (joystickButtons_[A_BUTTON] == 1) {
        ROS_DEBUG("A");

				controlMode_ = (controlMode_ + 1) % 2;
			}
      else
        ROS_DEBUG("!A");
    }

    //B
    if (joystickButtonsLast_[B_BUTTON] != joystickButtons_[B_BUTTON]) {
      if (joystickButtons_[B_BUTTON] == 1)
        ROS_DEBUG("B");
      else
        ROS_DEBUG("!B");
    }

    //X
    if (joystickButtonsLast_[X_BUTTON] != joystickButtons_[X_BUTTON]) {
      if (joystickButtons_[X_BUTTON] == 1) {
        ROS_DEBUG("X");
      }
      else {
        ROS_DEBUG("!X");
      }
    }

    //Y
    if (joystickButtonsLast_[Y_BUTTON] != joystickButtons_[Y_BUTTON]) {
      if (joystickButtons_[Y_BUTTON] == 1) {
        ROS_DEBUG("Y");
      }
      else {
        ROS_DEBUG("!Y");
      }
    }

    //Start
    if (joystickButtonsLast_[START_BUTTON] != joystickButtons_[START_BUTTON]) {
      if (joystickButtons_[START_BUTTON] == 1) {
        ROS_DEBUG("START");

        // Hold Y + press start -> start balancing
        if (joystickButtons_[Y_BUTTON] == 1) {
          ROS_INFO("Enable Balancing.");

          ros::ServiceClient client = handle_.serviceClient<ubot_msgs::SetBalancer>("/uBot/set_balancer");

          ubot_msgs::SetBalancer srv;
          srv.request.enable = true;
          if (!client.call(srv))
            ROS_ERROR("Failed to call SetBalancer service!!");

        }
      }
      else {
        ROS_DEBUG("!START");
      }
    }

    //Back
    if (joystickButtonsLast_[BACK_BUTTON] != joystickButtons_[BACK_BUTTON]) {
      if (joystickButtons_[BACK_BUTTON] == 1) {
        ROS_DEBUG("BACK");

        // Hold Y + press start -> stop balancing
        if (joystickButtons_[Y_BUTTON] == 1) {
          ROS_INFO("Disable Balancing.");

          ros::ServiceClient client = handle_.serviceClient<ubot_msgs::SetBalancer>("/uBot/set_balancer");

          ubot_msgs::SetBalancer srv;
          srv.request.enable = false;
          if (!client.call(srv))
            ROS_ERROR("Failed to call SetBalancer service!!");

        }
      }
      else {
        ROS_DEBUG("!BACK");
      }
    }

    //LB
    if (joystickButtonsLast_[LB_BUTTON] != joystickButtons_[LB_BUTTON]) {
      if (joystickButtons_[LB_BUTTON] == 1) {
        ROS_DEBUG("LB");
      }
      else {
        ROS_DEBUG("!LB");
      }
    }

    //RB
    if (joystickButtonsLast_[RB_BUTTON] != joystickButtons_[RB_BUTTON]) {
      if (joystickButtons_[RB_BUTTON] == 1) {
        ROS_DEBUG("RB");
      }
      else {
        ROS_DEBUG("!RB");
      }
    }

    //XBOX
    if (joystickButtonsLast_[XBOX_BUTTON] != joystickButtons_[XBOX_BUTTON]) {
      if (joystickButtons_[XBOX_BUTTON] == 1) {
        ROS_DEBUG("XBOX");
      }
      else {
        ROS_DEBUG("!XBOX");
      }
    }

    //LEFT STICK
    if (joystickButtonsLast_[LEFT_STICK] != joystickButtons_[LEFT_STICK]) {
      if (joystickButtons_[LEFT_STICK] == 1) {
        ROS_DEBUG("LEFT_STICK");
      }
      else {
        ROS_DEBUG("!LEFT_STICK");
      }
    }

    //RIGHT_STICK
    if (joystickButtonsLast_[RIGHT_STICK] != joystickButtons_[RIGHT_STICK]) {
      if (joystickButtons_[RIGHT_STICK] == 1) {
        ROS_DEBUG("RIGHT_STICK");
      }
      else {
        ROS_DEBUG("!RIGHT_STICK");
      }
    }

    //POV_LEFT
    if (joystickButtonsLast_[POV_LEFT] != joystickButtons_[POV_LEFT]) {
      if (joystickButtons_[POV_LEFT] == 1) {
        ROS_DEBUG("POV_LEFT");
      }
      else {
        ROS_DEBUG("!POV_LEFT");
      }
    }

    //POV_RIGHT
    if (joystickButtonsLast_[POV_RIGHT] != joystickButtons_[POV_RIGHT]) {
      if (joystickButtons_[POV_RIGHT] == 1) {
        ROS_DEBUG("POV_RIGHT");
      }
      else {
        ROS_DEBUG("!POV_RIGHT");
      }
    }

    //POV_UP
    if (joystickButtonsLast_[POV_UP] != joystickButtons_[POV_UP]) {
      if (joystickButtons_[POV_UP] == 1) {
        ROS_DEBUG("POV_UP");

        // Hold Y + press POV_UP -> start pushup
        if (joystickButtons_[Y_BUTTON] == 1) {

          pushUp();
        }
      }
      else {
        ROS_DEBUG("!POV_UP");
      }
    }

    //POV_DOWN
    if (joystickButtonsLast_[POV_DOWN] != joystickButtons_[POV_DOWN]) {
      if (joystickButtons_[POV_DOWN] == 1) {
        ROS_DEBUG("POV_DOWN");

        // Hold Y + press POV_UP -> start pushup
        if (joystickButtons_[Y_BUTTON] == 1) {

          pushDown();
        }
      }
      else {
        ROS_DEBUG("!POV_DOWN");
      }
    }

    //LT
    if (left_button_pressed_ && joystickAxes_[LT] > 0.5) {
      left_button_pressed_ = false;
      ROS_DEBUG("!LT");
    }
    else if (!left_button_pressed_ && joystickAxes_[LT] < -0.5) {
      left_button_pressed_ = true;
      ROS_DEBUG("LT");

      //control for angle offset
      changeAngleOffset(ANGLE_OFFSET_DELTA);
    }

    //RT
    if (right_button_pressed_ && joystickAxes_[RT] > 0.5) {
      right_button_pressed_ = false;
      ROS_DEBUG("!RT");
    }
    else if (!right_button_pressed_ && joystickAxes_[RT] < -0.5) {
      right_button_pressed_ = true;
      ROS_DEBUG("RT");

      //control for angle offset
      changeAngleOffset(-ANGLE_OFFSET_DELTA);
    }


    //////////////////////
    //H+T

		double x = 0.0;
		double y = 0.0;
		double z = 0.0;

    if (joystickAxes_[RIGHT_X] != 0)
	  { // H & T
      //ROS_INFO("%f", joystickAxes_[RIGHT_X] * DELTA_X);
      x = joystickAxes_[RIGHT_X] * DELTA_X;
			joystick_pressed_one_step_before_ = true;
    }
    if (joystickAxes_[RIGHT_Y] != 0)
    { // H & T{
      //ROS_INFO("%f", joystickAxes_[RIGHT_Y] * DELTA_Y);
     	y = joystickAxes_[RIGHT_Y] * DELTA_Y;
			joystick_pressed_one_step_before_ = true;
    }
    if (joystickAxes_[LEFT_Y] != 0 && joystickButtons_[LB_BUTTON] == 1)
    { // H & T
      //ROS_INFO("%f", joystickAxes_[LEFT_Y] * DELTA_Z);
      z = joystickAxes_[LEFT_Y] * DELTA_Z;
			joystick_pressed_one_step_before_ = true;
    }
//    //                      if (joystickAxes_[TOP_LEFT_LR_AXIS] > 0)
//    //                              ROS_INFO("TL: Left");
//
//    executeAction(goal_left_, goal_right_);

    //H+T-end
    ////////////////////////////////
//ROS_INFO("delta 1: %4.3f, %4.3f, %4.3f", x, y, z );






    //---- Axes ----

    switch (controlMode_)
    {
      case DRIVE_MODE:
      {
        ubot_msgs::DriveCommands drive_commands_msg;
        drive_commands_msg.stamp = ros::Time::now();
        drive_commands_msg.velocity = joystickAxes_[LEFT_Y] * MAX_VELOCITY;
        drive_commands_msg.turnrate = joystickAxes_[LEFT_X] * MAX_TURNRATE;
        drive_pub_.publish(drive_commands_msg);
        break;
      }
      case CARTESIAN_BOTH_MODE:
      {
        //H+T
//ROS_INFO("delta 2: %4.3f, %4.3f, %4.3f", x, y, z );
				if (x == 0 && y == 0 && z == 0)
				{
					if (joystick_pressed_one_step_before_ == true)
					{
						ROS_INFO("Arm control: Canceling");
						cancelBothAction();
					}
					joystick_pressed_one_step_before_ = false;
				}
				else
				{
					goal_right_.delta.x = -x;
					goal_right_.delta.y = y;
					goal_right_.delta.z = z;
					goal_right_.isDelta = true;
					goal_right_.kinematicCondition = false;
					goal_right_.test = false;

					goal_left_.delta.x = x;
					goal_left_.delta.y = y;
					goal_left_.delta.z = z;
					goal_left_.isDelta = true;
					goal_left_.kinematicCondition = false;
					goal_left_.test = false;

					ROS_INFO("Arm control: Executing New Positions");
					executeBothAction(goal_left_, goal_right_);
//ROS_INFO("new pos: %4.3f, %4.3f, %4.3f", goal_right_.position.point.x, goal_right_.position.point.y, goal_right_.position.point.z );
				}
//
//ROS_INFO("cur pos: %4.3f, %4.3f, %4.3f", right_hand_position_[0], right_hand_position_[1], right_hand_position_[2] );

        //H+T-end
        break;
      }
/*
      case CARTESIAN_RIGHT_MODE:
      {
        //H+T
				ROS_INFO("delta 2: %4.3f, %4.3f, %4.3f", x, y, z );
				if (x == 0 && y == 0 && z == 0)
				{
					if (joystick_pressed_one_step_before_ == true)
					{
						//goal_right_.delta.x = 0;
						//goal_right_.delta.y = 0;
						//goal_right_.delta.z = 0;
						//goal_right_.isDelta = true;
						//goal_right_.kinematicCondition = false;
						//goal_right_.test = false;
						//executeRightAction(goal_right_);

						cancelRightAction();
					}
					joystick_pressed_one_step_before_ = false;
				}
				else
				{
					goal_right_.delta.x = -x;
					goal_right_.delta.y = y;
					goal_right_.delta.z = z;
					goal_right_.isDelta = true;
					goal_right_.kinematicCondition = false;
					goal_right_.test = false;
					executeRightAction(goal_right_);
//ROS_INFO("new pos: %4.3f, %4.3f, %4.3f", goal_right_.position.point.x, goal_right_.position.point.y, goal_right_.position.point.z );
				}
//
//ROS_INFO("cur pos: %4.3f, %4.3f, %4.3f", right_hand_position_[0], right_hand_position_[1], right_hand_position_[2] );

        //H+T-end
        break;
      }
      case CARTESIAN_LEFT_MODE:
      {
        //H+T
//ROS_INFO("delta 2: %4.3f, %4.3f, %4.3f", x, y, z );
				if (x == 0 && y == 0 && z == 0)
				{
				}
				else
				{
					goal_left_.delta.x = x;
					goal_left_.delta.y = y;
					goal_left_.delta.z = z;
					goal_left_.isDelta = true;
					goal_left_.kinematicCondition = false;
					goal_left_.test = false;
					executeLeftAction(goal_left_);
//ROS_INFO("new pos: %4.3f, %4.3f, %4.3f", goal_right_.position.point.x, goal_right_.position.point.y, goal_right_.position.point.z );
				}
//
//ROS_INFO("cur pos: %4.3f, %4.3f, %4.3f", right_hand_position_[0], right_hand_position_[1], right_hand_position_[2] );

        //H+T-end
        break;
      }
*/
      default:
      {
        ROS_WARN( "Invalid control" );
        break;
      }
    }
    joystickButtonsLast_ = joystickButtons_;

    receivedJoystickMsg_ = false;
  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "uBotJointPublisher");

  JoystickControl node;
  ros::spin();

  return 0;
}
