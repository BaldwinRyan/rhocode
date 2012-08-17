#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <affordance_msgs/feature_detector_searchAction.h>


using namespace std;

int main(int argc, char **argv) {
	ros::init(argc, argv, "test_cart_search");

	ros::NodeHandle node_handle_;

	// create the action client
	// true causes the client to spin it's own thread
	actionlib::SimpleActionClient<
			affordance_msgs::feature_detector_searchAction> ac("hue_server", true);

	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");

	// send a goal to the action
	affordance_msgs::feature_detector_searchGoal goal;
	
	// search goals in cartesian space
	goal.numSearchGoals = 2;
	goal.signal.resize(2);
	goal.signal[0].mean.resize(1);
	goal.signal[0].variance.resize(1);
	goal.signal[1].mean.resize(1);
	goal.signal[1].variance.resize(1);

	/*	goal.signal_mean[0] = 10;
	goal.signal_var[0] = 5;
	goal.signal_mean[1] = 170;
	goal.signal_var[1] = 10;
	*/
	
	goal.signal[0].mean[0] = 114;
	goal.signal[0].variance[0] = 10;
	goal.signal[1].mean[0] = 10;
	goal.signal[1].variance[0] = 10;
	
	
	ac.sendGoal(goal);

	

	//exit
	return 0;
}

