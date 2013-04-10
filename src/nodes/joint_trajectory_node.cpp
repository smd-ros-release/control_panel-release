
/**
 * \file   joint_trajectory_node.cpp
 * \date   April 16, 2012
 * \author Matt Richard
 */
#include "control_panel/nodes/joint_trajectory_node.h"

JointTrajectoryNode::JointTrajectoryNode(ros::NodeHandle *nh_ptr)
{
	topic_name = "command";

	nh = nh_ptr;
}

void JointTrajectoryNode::addJoint(const std::string &joint_name)
{
	trajectory_msgs::JointTrajectoryPoint init_point;
	init_point.positions.push_back(0.0);
	init_point.velocities.push_back(0.0);
	init_point.accelerations.push_back(0.0);
	init_point.time_from_start = ros::Duration(0.0);

	joint_traject_msg.joint_names.push_back(joint_name);
	joint_traject_msg.points.push_back(init_point);
}

void JointTrajectoryNode::advertise()
{
	joint_traject_pub = nh->advertise<trajectory_msgs::JointTrajectory>(topic_name, 1);
}

int JointTrajectoryNode::jointIndex(const std::string &joint_name)
{
	unsigned int i = 0;

	for(i = 0; i < joint_traject_msg.joint_names.size(); i++)
		if(joint_traject_msg.joint_names[i] == joint_name)
			return i;

	return -1;
}

void JointTrajectoryNode::publish()
{
	joint_traject_pub.publish(joint_traject_msg);
}

void JointTrajectoryNode::setAcceleration(const std::string &joint_name, double acc)
{
	int index = jointIndex(joint_name);

	if(index != -1)
		joint_traject_msg.points[index].accelerations[0] = acc;
	else
		ROS_ERROR("Could not set acceleration for joint %s because it does not exist in the JointTrajectory message", joint_name.c_str());
}

void JointTrajectoryNode::setPosition(const std::string &joint_name, double pos)
{
	int index = jointIndex(joint_name);

	if(index != -1)
		joint_traject_msg.points[index].positions[0] = pos;
	else
		ROS_ERROR("Could not set position for joint %s because it does not exist in the JointTrajectory message", joint_name.c_str());
}

void JointTrajectoryNode::setVelocity(const std::string &joint_name, double vel)
{
	int index = jointIndex(joint_name);

	if(index != -1)
		joint_traject_msg.points[index].velocities[0] = vel;
	else
		ROS_ERROR("Could not set velocity for joint %s because it does not exist in the JointTrajectory message", joint_name.c_str());
}

void JointTrajectoryNode::unadvertise()
{
	joint_traject_pub.shutdown();
}
