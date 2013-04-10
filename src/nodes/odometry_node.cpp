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
 * \file   odometry_node.cpp
 * \date   Sept 8, 2011
 * \author Matt Richard
 */
#include "control_panel/nodes/odometry_node.h"

OdometryNode::OdometryNode(ros::NodeHandle *nh_ptr)
{
	topic_name = Globals::DEFAULT_ODOMETRY_TOPIC;

	nh = nh_ptr;
}

void OdometryNode::subscribe()
{
	odometry_sub = nh->subscribe(topic_name, 1, &OdometryNode::odometryCallback,
		this, ros::TransportHints().unreliable().tcpNoDelay());
}

void OdometryNode::unsubscribe()
{
	odometry_sub.shutdown();
}

void OdometryNode::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
{
	// Check for any NaN values
	if(msg->pose.pose.position.x != msg->pose.pose.position.x ||
	   msg->pose.pose.position.y != msg->pose.pose.position.y ||
	   msg->pose.pose.position.z != msg->pose.pose.position.z ||
	   msg->pose.pose.orientation.w != msg->pose.pose.orientation.w ||
	   msg->pose.pose.orientation.x != msg->pose.pose.orientation.x ||
	   msg->pose.pose.orientation.y != msg->pose.pose.orientation.y ||
	   msg->pose.pose.orientation.z != msg->pose.pose.orientation.z ||
	   msg->twist.twist.linear.x != msg->twist.twist.linear.x ||
	   msg->twist.twist.linear.y != msg->twist.twist.linear.y ||
	   msg->twist.twist.linear.z != msg->twist.twist.linear.z ||
	   msg->twist.twist.angular.x != msg->twist.twist.angular.x ||
	   msg->twist.twist.angular.y != msg->twist.twist.angular.y ||
	   msg->twist.twist.angular.z != msg->twist.twist.angular.z)
	{
		ROS_ERROR("NaN detected in Odometry message");
		return;
	}

	emit odometryDataReceived(
		QVector3D(msg->pose.pose.position.x,
			msg->pose.pose.position.y,
			msg->pose.pose.position.z),
		QQuaternion(msg->pose.pose.orientation.w,
			msg->pose.pose.orientation.x,
			msg->pose.pose.orientation.y,
			msg->pose.pose.orientation.z),
		QVector3D(msg->twist.twist.linear.x,
			msg->twist.twist.linear.y,
			msg->twist.twist.linear.z),
		QVector3D(msg->twist.twist.angular.x,
			msg->twist.twist.angular.y,
			msg->twist.twist.angular.z));
}
