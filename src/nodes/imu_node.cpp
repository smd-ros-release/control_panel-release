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
 * \file   imu_node.cpp
 * \date   Sept 8, 2011
 * \author Matt Richard
 */
#include "control_panel/nodes/imu_node.h"

ImuNode::ImuNode(ros::NodeHandle *nh_ptr)
{
	topic_name = Globals::DEFAULT_IMU_TOPIC;

	nh = nh_ptr;
}

void ImuNode::subscribe()
{
	imu_sub = nh->subscribe(topic_name, 1, &ImuNode::imuCallback, this,
		ros::TransportHints().unreliable().tcpNoDelay());
}

void ImuNode::unsubscribe()
{
	imu_sub.shutdown();
}

void ImuNode::imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
	// Check for any NaN values
	if(msg->orientation.w != msg->orientation.w ||
	   msg->orientation.x != msg->orientation.x ||
	   msg->orientation.y != msg->orientation.y ||
	   msg->orientation.z != msg->orientation.z ||
	   msg->angular_velocity.x != msg->angular_velocity.x ||
	   msg->angular_velocity.y != msg->angular_velocity.y ||
	   msg->angular_velocity.z != msg->angular_velocity.z ||
	   msg->linear_acceleration.x != msg->linear_acceleration.x ||
	   msg->linear_acceleration.y != msg->linear_acceleration.y ||
	   msg->linear_acceleration.z != msg->linear_acceleration.z)
	{
		ROS_ERROR("NaN detected in Imu message");
		return;
	}

	emit imuDataReceived(QQuaternion(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z),
		QVector3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
		QVector3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z));
}
