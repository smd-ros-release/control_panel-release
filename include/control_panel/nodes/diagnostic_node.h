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
 * \file   diagnostic_node.h
 * \date   Sept 8, 211
 * \author Matt Richard, Scott K Logan
 */
#ifndef CONTROL_PANEL_DIAGNOSTIC_NODE_H
#define CONTROL_PANEL_DIAGNOSTIC_NODE_H

#include <QObject>
#include "ros/ros.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include <stdio.h>
#include <string>
#include "control_panel/globals.h"

/**
 * \class DiagnosticNode
 * \brief ROS node for receiving diagnostic data.
 */
class DiagnosticNode : public QObject
{
	Q_OBJECT

	public:
		DiagnosticNode(ros::NodeHandle *nh_ptr);
		void subscribe();
		void unsubscribe();
		void diagnosticCallback(const diagnostic_msgs::DiagnosticArrayConstPtr &msg);
		void setTopic(const std::string &topic) { topic_name = topic; }
		std::string getTopic() const { return topic_name; }

	signals:
		void diagnosticDataReceived(const QString &key, const QString &val);

	private:
		std::string topic_name;
		ros::NodeHandle *nh;
		ros::Subscriber diagnostic_sub;
		char ns[255];
};

#endif // CONTROL_PANEL_DIAGNOSTIC_NODE_H
