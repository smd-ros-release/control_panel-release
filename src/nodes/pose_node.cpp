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
 * \file   pose_node.cpp
 * \date   Sept 8, 2011
 * \author Matt Richard
 */
#include "control_panel/nodes/pose_node.h"

PoseNode::PoseNode(ros::NodeHandle *nh_ptr, bool _isStamped, bool _hasCovariance)
{
	topic_name = Globals::DEFAULT_ODOMETRY_TOPIC;

	isStamped = _isStamped;
	hasCovariance = _hasCovariance;

	nh = nh_ptr;
}

//template<class T>
void PoseNode::subscribe()
{
	if(hasCovariance)
		pose_sub = nh->subscribe(topic_name, 1, &PoseNode::poseWithCovarianceStampedCallback,
			this, ros::TransportHints().unreliable().tcpNoDelay());
	else if(isStamped)
		pose_sub = nh->subscribe(topic_name, 1, &PoseNode::poseStampedCallback,
			this, ros::TransportHints().unreliable().tcpNoDelay());
	else
		pose_sub = nh->subscribe(topic_name, 1, &PoseNode::poseCallback,
			this, ros::TransportHints().unreliable().tcpNoDelay());
}

void PoseNode::unsubscribe()
{
	pose_sub.shutdown();
}

void PoseNode::poseCallback(const geometry_msgs::PoseConstPtr &msg)
{
	// Check for any NaN values
	if(msg->position.x != msg->position.x ||
	   msg->position.y != msg->position.y ||
	   msg->position.z != msg->position.z ||
	   msg->orientation.w != msg->orientation.w ||
	   msg->orientation.x != msg->orientation.x ||
	   msg->orientation.y != msg->orientation.y ||
	   msg->orientation.z != msg->orientation.z)
	{
		ROS_ERROR("NaN detected in Pose message");
		return;
	}

	emit poseDataReceived(
		QVector3D(msg->position.x,
			msg->position.y,
			msg->position.z),
		QQuaternion(msg->orientation.w,
			msg->orientation.x,
			msg->orientation.y,
			msg->orientation.z),
		QVector3D(0, 0, 0),
		QVector3D(0, 0, 0));
}

void PoseNode::poseStampedCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
	geometry_msgs::PoseConstPtr p = boost::make_shared<geometry_msgs::Pose>( msg->pose );
	poseCallback( p );
}

void PoseNode::poseWithCovarianceStampedCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
	geometry_msgs::PoseConstPtr p = boost::make_shared<geometry_msgs::Pose>( msg->pose.pose );
	poseCallback( p );
}

