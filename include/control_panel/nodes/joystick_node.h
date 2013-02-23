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
 * \file   joystick_node.h
 * \date   October 2011
 * \author Scott K Logan, Matt Richard
 * \brief  SRS Basestation interface to joystick_driver stack.
 */
#ifndef CONTROL_PANEL_JOYSTICK_NODE_H
#define CONTROL_PANEL_JOYSTICK_NODE_H

#include <QObject>
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "control_panel/globals.h"
#include <string>


/**
 * \class JoystickNode
 * \brief ROS node that receives a sensor_msgs::Joy message
 */
class JoystickNode : public QObject
{
	Q_OBJECT

	public:
		JoystickNode(ros::NodeHandle *nh_ptr);
        /**
         * \brief Empty deconstructor
         */
		~JoystickNode() { }

        /**
         * \brief Returns the topic name this node subscribes to
         */
        std::string getTopic() const { return joy_topic_name; }

        /**
         * \brief Emits a signal of all button and axis values that have changed
         *
         * \param msg The sensor_msgs::Joy message received
         */
		void joyCallback(const sensor_msgs::JoyConstPtr &msg);

        /**
         * \brief Sets the topic name for which this node should subscribe to
         *
         * \param topic_name The new topic to subscribe to
         */
        void setTopic(const std::string &topic_name) { joy_topic_name = topic_name; }

        /**
         * \brief Subscribes on the set topic
         */
        void subscribe();

        /**
         * \brief Shuts down the subscriber
         */
        void unsubscribe();

	signals:
        /**
         * \brief Emitted when the value of a joystick axis has changed
         */
		void axis_event(int, double);

        /**
         * \brief Emitted when the value of a joystick button has changed
         */
		void button_event(int, bool);

	private:
		std::string joy_topic_name;
		ros::NodeHandle *nh;
		ros::Subscriber joy_sub;
};

#endif // CONTROL_PANEL_JOYSTICK_NODE_H

