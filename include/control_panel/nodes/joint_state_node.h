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
 * \file   joint_state_node.h
 * \date   Dec 7, 2011
 * \author Matt Richard
 */
#ifndef CONTROL_PANEL_JOINT_STATE_NODE_H
#define CONTROL_PANEL_JOINT_STATE_NODE_H

#include <QObject>
#include <QStringList>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <string>
#include <vector>
#include "control_panel/globals.h"

/**
 * \class JointStateNode
 * \brief ROS node to receive a sensor_msgs::JointState message from a robot
 */
class JointStateNode : public QObject
{
	Q_OBJECT

	public:
		/**
		 * \brief Constructor. Initializes the topic name and copys the node handle pointer
		 */
		JointStateNode(ros::NodeHandle *nh_ptr);

		/**
		 * \brief Returns the subscriber's topic
		 */
		std::string getTopic() const { return topic_name; }

		/**
		 * \brief Callback function for when a sensor_msgs::JointState message is received
		 *
		 * \param msg sensor_msgs::JointState message received
		 */
		void jointCallback(const sensor_msgs::JointStateConstPtr &msg);

		/**
		 * \brief Sets the topic over which the subscriber should subscribe to.
		 */
		void setTopic(const std::string &topic) { topic_name = topic; }

		/**
		 * \brief Subscribes over the set topic
		 */
		void subscribe();

		/**
		 * \brief Shutsdown the subscriber
		 */
		void unsubscribe();

	signals:
		/**
		 * \brief Signal emitted after a message is received
		 *
		 * \param names The list of joint names
		 * \param pos   Positions of each joint
		 * \param vel   Velocities of each joint
		 * \param eff   Effort of each joint
		 */
		void jointDataReceived(const QStringList &names, const std::vector<double> &pos,
                               const std::vector<double> &vel, const std::vector<double> &eff);

	private:
		std::string topic_name;
		ros::NodeHandle *nh;
		ros::Subscriber joint_sub;
};

#endif // CONTROL_PANEL_JOINT_STATE_NODE_H
