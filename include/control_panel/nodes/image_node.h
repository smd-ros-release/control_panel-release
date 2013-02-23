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
 * \file   image_node.h
 * \date   Sept 6, 2011
 * \author Matt Richard
 */
#ifndef CONTROL_PANEL_IMAGE_NODE_H
#define CONTROL_PANEL_IMAGE_NODE_H

#include <QObject>
#include <QImage>
#include "ros/ros.h"
#include "ros/transport_hints.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include <cv.h>
#include "control_panel/globals.h"

namespace enc = sensor_msgs::image_encodings;


/**
 * \class ImageNode
 * \brief ROS node that receives a sensor_msgs::Image message and converts it to a QImage
 */
class ImageNode : public QObject
{
	Q_OBJECT

	public:
		/**
		 * \brief Initializes the ImageTransport
		 *
		 * \param nh_ptr Then node handle to use for subscribing
		 */
		ImageNode(ros::NodeHandle *nh_ptr);

		/**
		 * \brief Deletes the ImageTransport to ensure we unsubscribe from the subscribed topic
		 */
		~ImageNode();

		/**
		 * \brief Returns the topic the subscriber subscribes to
		 */
		std::string getTopic() const { return topic_name; }

		/**
		 * \brief Callback function for when we receive a sensor_msgs::Image.msg
		 *
		 * This converts the sensor_msgs::Image into a QImage.
		 *
		 * \param msg The image received
		 */
		void imageCallback(const sensor_msgs::ImageConstPtr &msg);

		/**
		 * \brief Sets topic for which to subscribe to
		 *
		 * \param topic The topic to subscribe to
		 */
		void setTopic(const std::string &topic) { topic_name = topic; }

		/**
		 * \brief Starts the subscription over the set topic
		 */
		void subscribe();

		/**
		 * \brief Shuts down the subscriber
		 */
        	void unsubscribe();

	signals:
		/**
		 * \brief Signal emitted once the image receive has been converted
		 *
		 * \param buffer The converted image buffer
		 */
		void frameReceived(const QImage &buffer);

	private:
		std::string topic_name;
		ros::NodeHandle *nh;
		image_transport::ImageTransport *it;
		image_transport::Subscriber image_sub;
};

#endif // CONTROL_PANEL_IMAGE_NODE_H
