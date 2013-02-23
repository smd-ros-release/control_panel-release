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
 * \file   pose_node.h
 * \date   Aug 12, 2011
 * \author Matt Richard
 */
#ifndef CONTROL_PANEL_POSE_NODE_H
#define CONTROL_PANEL_POSE_NODE_H

#include <QObject>
#include <QVector3D>
#include <QQuaternion>
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <string>
#include "control_panel/globals.h"

/**
 * \class PoseNode
 * \brief ROS node for receving a robot's pose data
 */
class PoseNode : public QObject
{
	Q_OBJECT

	public:
		PoseNode(ros::NodeHandle *nh_ptr, bool isStamped = false, bool hasCovaraince = false);
		void subscribe();
		void unsubscribe();
		void processPose(const geometry_msgs::Pose &msg);
		void poseCallback(const geometry_msgs::PoseConstPtr &msg);
		void poseStampedCallback(const geometry_msgs::PoseStampedConstPtr &msg);
		void poseWithCovarianceStampedCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
		void setTopic(const std::string &topic) { topic_name = topic; }
		std::string getTopic() const { return topic_name; }

	signals:
		void poseDataReceived(const QVector3D &position,
			const QQuaternion &orientation, const QVector3D &linear_velocity,
			const QVector3D &angular_velocity);

	private:
		std::string topic_name;
		ros::NodeHandle *nh;
		ros::Subscriber pose_sub;
		bool isStamped;
		bool hasCovariance;
};

#endif // CONTROL_PANEL_POSE_NODE_H
