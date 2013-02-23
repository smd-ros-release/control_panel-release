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
 * \file   control_node.h
 * \date   Aug 31, 2011
 * \author Matt Richard
 */
#ifndef CONTROL_PANEL_CONTROL_NODE_H
#define CONTROL_PANEL_CONTROL_NODE_H

#include <QObject>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <string>
#include "control_panel/globals.h"

/**
 * \class ControlNode
 * \brief ROS node for controlling a robot's movement
 */
class ControlNode : public QObject
{
	Q_OBJECT

	public:
		/**
		 * \brief Contructor. Inizializes the topic name and the Twist message, and copies the node handle pointer.
		 *
		 * \param nh_ptr Node Handle to advertise and publish with
		 */
		ControlNode(ros::NodeHandle *nh_ptr);

		/**
		 * \brief Starts advertizing on the set topic
		 */
		void advertise();

		/**
		 * \brief Returns the current angular x value of the Twist message
		 */
		double getAngularX() const { return twist_msg.angular.x; }

		/**
		 * \brief Returns the current angular y value of the Twist message
		 */
		double getAngularY() const { return twist_msg.angular.y; }

		/**
		 * \brief Returns the current angular z value of the Twist message
		 */
		double getAngularZ() const { return twist_msg.angular.z; }

		/**
		 * \brief Returns the current linear x value of the Twist message
		 */
		double getLinearX() const { return twist_msg.linear.x; }

		/**
		 * \brief Returns the current linear y value of the Twist message
		 */
		double getLinearY() const { return twist_msg.linear.y; }

		/**
		 * \brief Returns the current linear z value of the Twist message
		 */
		double getLinearZ() const { return twist_msg.linear.z; }

		/**
		 * \brief Retruns the value each Twist message angular attribute is scaled by
		 */
		double getAngularScale() const { return ang_scale; }

		/**
		 * \brief Returns the value each Twist message linear attribute is scaled by
		 */
		double getLinearScale() const { return lin_scale; }

		/**
		 * \brief Returns the topic this node advertises over
		 */
		std::string getTopic() const { return topic_name; }

		/**
		 * \brief Returns the current Twist message
		 */
		geometry_msgs::Twist getTwist() const { return twist_msg; }

		/**
		 * \brief Sets the angular x value of the Twist message if the parameter is valid value
		 *
		 * \param x The value to be set as angular x in the Twist message
		 */
		void setAngularX(double x) { if(validVelocity(x)) twist_msg.angular.x = x * ang_scale; }

		/**
		 * \brief Sets the angular y value of the Twist message if the parameter is a valid value
		 *
		 * \param y The value to be set as angular y in the Twist message
		 */
		void setAngularY(double y) { if(validVelocity(y)) twist_msg.angular.y = y * ang_scale; }

		/**
		 * \brief Sets the angular z value of the Twist message if the parameter is a valid value
		 *
		 * \param z The value to be set as angular z in the Twist message
		 */
		void setAngularZ(double z) { if(validVelocity(z)) twist_msg.angular.z = z * ang_scale; }

		/**
		 * \brief Sets the linear x value of the Twist message if the parameter is a valid value
		 *
		 * \param x The value to be set as linear x in the Twist message
		 */
		void setLinearX(double x) { if(validVelocity(x)) twist_msg.linear.x = x * lin_scale; }

		/**
		 * \brief Sets the linear y value of the Twist message if the parameter is a valid value
		 *
		 * \param y The value to be set as linear y in the Twist message
		 */
		void setLinearY(double y) { if(validVelocity(y)) twist_msg.linear.y = y * lin_scale; }

		/**
		 * \brief Sets the linear z value of the Twist message if the parameter is a valid value
		 *
		 * \param z The value to be set as linear z in the Twist message
		 */
		void setLinearZ(double z) { if(validVelocity(z)) twist_msg.linear.z = z * lin_scale; }

		/**
		 * \brief Sets the scale that the Twist message attributes should be multiplied by
		 *
		 * \param lin The value all linear attributes of the Twist message should be scaled by
		 * \param ang The value all angular attributes of the Twist message should be scaled by
		 */
		void setScale(double lin, double ang);

		/**
		 * \brief Sets the topic for which this node should advertise and publish over
		 *
		 * \param topic The topic to advertise and publish over
		 */
		void setTopic(const std::string &topic) { topic_name = topic; }

		/**
		 * \brief Shuts down the publisher
		 */
		void unadvertise();

	public slots:
		/**
		 * \brief Publishes the geometry_msgs::Twist message
		 */
		void publish();

		/**
		 * \brief Sets the angular x, y, and z portions of the Twist message
		 *
		 * \param x The value to be set as angular x in the Twist message
		 * \param y The value to be set as angular y in the Twist message
		 * \param z The value to be set as angular z in the Twist message
		 */
		void setAngularVector(double x = 0.0, double y = 0.0, double z = 0.0);

		/**
		 * \brief Sets the linear x, y, and z portions of the Twist message
		 *
		 * \param x The value to be set as linear x in the Twist message
		 * \param y The value to be set as linear y in the Twist message
		 * \param z The value to be set as linear z in the Twist message
		 */
		void setLinearVector(double x = 0.0, double y = 0.0, double z = 0.0);

		/**
		 * \brief Sets the Twist message to twist
		 *
		 * \param twist The geometry_msgs:Twist message to be published
		 */
		void setTwist(const geometry_msgs::Twist &twist);

		/**
		 * \brief Sets the full Twist message attributes
		 *
		 * \param lx The value to be set as linear x in the Twist message
		 * \param ly The value to be set as linear y in the Twist message
		 * \param lz The value to be set as linear z in the Twist message
		 * \param ax The value to be set as angular x in the Twist message
		 * \param ay The value to be set as angular y in the Twist message
		 * \param az The value to be set as angular z in the Twist message
		 */
		void setTwist(double lx = 0.0, double ly = 0.0, double lz = 0.0, 
			double ax = 0.0, double ay = 0.0, double az = 0.0);

	private:
		/**
		 * \brief Checks if value is between 0.0 and 1.0
		 *
		 * \param value The velociy value to check if valid
		 */
		bool validVelocity(double value);

		std::string topic_name;
		ros::NodeHandle *nh;
		ros::Publisher control_pub;
		geometry_msgs::Twist twist_msg;
		double lin_scale;
		double ang_scale;
};

#endif // CONTROL_PANEL_CONTROL_NODE_H
