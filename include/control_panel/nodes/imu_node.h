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
 * \file   imu_node.h
 * \date   Sept 8, 2011
 * \author Matt Richard
 */
#ifndef CONTROL_PANEL_IMU_NODE_H
#define CONTROL_PANEL_IMU_NODE_H

#include <QObject>
#include <QQuaternion>
#include <QVector3D>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <string>
#include "control_panel/globals.h"

/**
 * \class ImuNode
 * \brief ROS node for receiving a robot's imu data
 */
class ImuNode : public QObject
{
	Q_OBJECT

	public:
		ImuNode(ros::NodeHandle *nh_ptr);
		void subscribe();
		void unsubscribe();
		void imuCallback(const sensor_msgs::ImuConstPtr &msg);
		void setTopic(const std::string &topic) { topic_name = topic; }
		std::string getTopic() const { return topic_name; }

	signals:
		void imuDataReceived(const QQuaternion &ori, const QVector3D &ang_vel,
			const QVector3D &lin_accel);

	private:
		std::string topic_name;
		ros::NodeHandle *nh;
		ros::Subscriber imu_sub;
};

#endif // CONTROL_PANEL_IMU_NODE_H
