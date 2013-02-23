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
 * \file   map_node.h
 * \date   Nov 19, 2011
 * \author Matt Richard
 */
#ifndef CONTROL_PANEL_MAP_NODE_H
#define CONTROL_PANEL_MAP_NODE_H

#include <QObject>
#include <QImage>
#include <QRgb>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <string>
#include "control_panel/globals.h"

/**
 * \class MapNode
 * \brief ROS node that receives a nav_msgs::Map and converts it to a QImage
 */
class MapNode : public QObject
{
	Q_OBJECT

	public:
		MapNode(ros::NodeHandle *nh_ptr);
		void subscribe();
		void unsubscribe();
		void mapCallback(const nav_msgs::OccupancyGridConstPtr &msg);
		void setTopic(const std::string &topic) { topic_name = topic; }
		std::string getTopic() const { return topic_name; }

	signals:
		void mapReceived(const QImage &buffer, double origin_x, double origin_y, float res);

	private:
		std::string topic_name;
		ros::NodeHandle *nh;
		ros::Subscriber map_sub;

		QRgb white;
		QRgb black;
		QRgb grey;
};

#endif // CONTROL_PANEL_MAP_NODE_H
