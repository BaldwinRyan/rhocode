#include <string>
#include <ros/ros.h>
#include <bisight/bisight_driver.h>
#include <sensor_msgs/JointState.h>



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
class Bisight_Node {
    public:
        Bisight_Node(ros::NodeHandle& node_handle);
        ~Bisight_Node();

        // Service Control
        void Connect();
        bool ok() {
            return m_pantilt != NULL;
        }
        void Disconnect();

        // Service Execution
        void spinOnce();

        // Callback Methods
        void SetGoal(const sensor_msgs::JointState::ConstPtr& msg);

    protected:
        Bisight* m_pantilt;
        ros::NodeHandle m_node;
        ros::Publisher  m_joint_pub;
        ros::Subscriber m_joint_sub;
};

Bisight_Node::Bisight_Node(ros::NodeHandle& node_handle)
        :m_pantilt(NULL), m_node(node_handle) {
    // Empty Constructor
}

Bisight_Node::~Bisight_Node() {
    Disconnect();
}

/** Opens the connection to the PTU and sets appropriate parameters.
    Also manages subscriptions/publishers */
void Bisight_Node::Connect() {
    // If we are reconnecting, first make sure to disconnect
    if (ok()) {
        Disconnect();
    }

    // Query for serial configuration
    std::string port = "/dev/ttyUSB0";
    //    m_node.param<std::string>("port", port, PTU46_DEFAULT_PORT);
    //    m_node.param("baud", baud, PTU46_DEFAULT_BAUD);

    // Connect to the PTU
    ROS_INFO("Attempting to connect to %s...", port.c_str());
    m_pantilt = new Bisight(port.c_str());
    ROS_ASSERT(m_pantilt != NULL);
    if (! m_pantilt->isOpen()) {
        ROS_ERROR("Could not connect to pan/tilt unit [%s]", port.c_str());
        Disconnect();
        return;
    }
    ROS_INFO("Connected!");


    // Publishers : Only publish the most recent reading
    m_joint_pub = m_node.advertise
                  <sensor_msgs::JointState>("/ptu/state", 1);

    // Subscribers : Only subscribe to the most recent instructions
    m_joint_sub = m_node.subscribe
                  <sensor_msgs::JointState>("/ptu/cmd", 1, &Bisight_Node::SetGoal, this);

}

/** Disconnect */
void Bisight_Node::Disconnect() {
    if (m_pantilt != NULL) {
        delete m_pantilt;   // Closes the connection
        m_pantilt = NULL;   // Marks the service as disconnected
    }
}

/** Callback for getting new Goal JointState */
void Bisight_Node::SetGoal(const sensor_msgs::JointState::ConstPtr& msg) {
    if (! ok())
        return;
    double pan = msg->position[0];
    double tilt = msg->position[1];
    //double panspeed = msg->velocity[0];
    //double tiltspeed = msg->velocity[1];
   // m_pantilt->setJointPosition(PAN, pan);
   // m_pantilt->setJointPosition(TILT, tilt);

    double values[4];
    values[PAN] = pan;
    values[TILT] = tilt;
    values[2] = 0.0;
    values[3] = 0.0;

    m_pantilt->setJointPositions(values);
    //m_pantilt->SetSpeed(PTU46_PAN, panspeed);
    //m_pantilt->SetSpeed(PTU46_TILT, tiltspeed);
	
}

/**
 * Publishes a joint_state message with position and speed.
 * Also sends out updated TF info.
 */
void Bisight_Node::spinOnce() {
    if (! ok())
        return;

    // Read Position & Speed
    double pan;//  = m_pantilt->getJointPosition(PAN);	
    double tilt;// = m_pantilt->getJointPosition(TILT);

    double panspeed  = 0.0; //m_pantilt->GetSpeed(PTU46_PAN);
    double tiltspeed = 0.0; //m_pantilt->GetSpeed(PTU46_TILT);

    double values[4]; 

    m_pantilt->getJointPositions(values);
    pan = values[0];
    tilt = values[1];

    // Publish Position & Speed
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.set_name_size(2);
    joint_state.set_position_size(2);
    joint_state.set_velocity_size(2);
    joint_state.name[0] ="pan";
    joint_state.position[0] = pan;
    joint_state.velocity[0] = panspeed;
    joint_state.name[1] ="tilt";
    joint_state.position[1] = tilt;
    joint_state.velocity[1] = tiltspeed;
    m_joint_pub.publish(joint_state);

}



int main(int argc, char** argv) {
    ros::init(argc, argv, "ptu");
    ros::NodeHandle n("~");

    // Connect to PTU
    Bisight_Node bisight_node = Bisight_Node(n);
    bisight_node.Connect();
    if (! bisight_node.ok())
        return -1;

    ROS_INFO("Connected.");
    // Query for polling frequency
    int hz;
    n.param("hz", hz, BISIGHT_DEFAULT_HZ);
    ros::Rate loop_rate(hz);

    while (ros::ok() && bisight_node.ok()) {
        // Publish position & velocity
        bisight_node.spinOnce();

	ROS_INFO("Running...");
        // Process a round of subscription messages
        ros::spinOnce();

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }

    if (! bisight_node.ok()) {
        ROS_ERROR("pan/tilt unit disconncted prematurely");
        return -1;
    }

    return 0;
}
