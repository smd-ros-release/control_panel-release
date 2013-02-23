/*
 * Copyright (c) 2011, 2012 Matt Richard, Scott K Logan.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


/**
 * \file   control_node.cpp
 * \date   Aug 31, 2011
 * \author Matt Richard
 * \brief  ROS node for controlling a robot's movement.
 */
#include "control_panel/nodes/control_node.h"


ControlNode::ControlNode(ros::NodeHandle *nh_ptr)
{
	topic_name = Globals::DEFAULT_CONTROL_TOPIC;

	// Initialize twist message
	twist_msg.linear.x = 0.0;
	twist_msg.linear.y = 0.0;
	twist_msg.linear.z = 0.0;
	twist_msg.angular.x = 0.0;
	twist_msg.angular.y = 0.0;
	twist_msg.angular.z = 0.0;

	lin_scale = 0.5;
	ang_scale = 0.5;

	nh = nh_ptr;
}

void ControlNode::advertise()
{
	control_pub = nh->advertise<geometry_msgs::Twist>(topic_name, 1);
}

void ControlNode::publish()
{
	control_pub.publish(twist_msg);
}

void ControlNode::setScale(double lin, double ang)
{
	// Set linear scale
	lin_scale = lin;

	// Set angular scale
	ang_scale = ang;
}

void ControlNode::unadvertise()
{
	control_pub.shutdown();
}

/******************************************************************************
 * Public Slots
 *****************************************************************************/

void ControlNode::setAngularVector(double x, double y, double z)
{
	setAngularX(x);
	setAngularY(y);
	setAngularZ(z);
}

void ControlNode::setLinearVector(double x, double y, double z)
{
	setLinearX(x);
	setLinearY(y);
	setLinearZ(z);
}

void ControlNode::setTwist(const geometry_msgs::Twist &twist)
{
	twist_msg = twist;
}

void ControlNode::setTwist(double lx, double ly, double lz,double ax, 
	double ay, double az)
{
	setLinearX(lx);
	setLinearY(ly);
	setLinearZ(lz);
	setAngularX(ax);
	setAngularY(ay);
	setAngularZ(az);
}

/******************************************************************************
 *                           Private Functions
 *****************************************************************************/

bool ControlNode::validVelocity(double value)
{
	if(value >= -1.0 && value <= 1.0)
		return true;

	ROS_ERROR("Invalid velocity: %f", value);

	return false;
}
