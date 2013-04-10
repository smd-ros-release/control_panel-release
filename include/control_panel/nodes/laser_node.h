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
 * \file   laser_node.h
 * \date   Nov 16, 2011
 * \author Matt Richard
 */
#ifndef CONTROL_PANEL_LASER_NODE_H
#define CONTROL_PANEL_LASER_NODE_H

#include <QObject>
#include <QImage>
#include <QRgb>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <string>
#include <math.h>
#include "control_panel/globals.h"

/**
 * \class LaserNode
 * \brief Recieves a sensor_msgs::LaserScan and converts it to a QImage.
 */
class LaserNode : public QObject
{
	Q_OBJECT

	public:
		/**
		 * \brief Constructor. Initializes the topic name and copys nh_ptr.
		 */
		LaserNode(ros::NodeHandle *nh_ptr);

		/**
		 * \brief Subscribes to the set topic
		 */
		void subscribe();

		/**
		 * \brief Unsubscribes from the current topic.
		 */
		void unsubscribe();

		/**
		 * \brief ROS callback function for the incoming laser scan.
		 */
		void laserCallback(const sensor_msgs::LaserScanConstPtr &msg);

		/**
		 * \brief Sets the topic on which to receive a laser scan.
		 *
		 * \param topic The topic to subscribe to.
		 */
		void setTopic(const std::string &topic) { topic_name = topic; }

		/**
		 * \brief Returns the set topic
		 */
		std::string getTopic() const { return topic_name; }

	signals:
		/**
		 * \brief Signal emitted after a laser scan has been received and converted
		 *
		 * \param buffer   The displayable laser scan image created
		 * \param interval The max range of the laser scan (meters)
		 */
		void laserScanReceived(const QImage &buffer, int interval);

	private:
		std::string topic_name;
		ros::NodeHandle *nh;
		ros::Subscriber laser_sub;

		QRgb white;
		QRgb red;
};

#endif // CONTROL_PANEL_LASER_NODE_H
