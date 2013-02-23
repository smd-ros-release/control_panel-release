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
 * \file   diagnostic_node.cpp
 * \date   Sept 8, 2011
 * \author Matt Richard, Scott K Logan
 * \brief  ROS node for received robot diagnostic data.
 */
#include "control_panel/nodes/diagnostic_node.h"


DiagnosticNode::DiagnosticNode(ros::NodeHandle *nh_ptr)
{
	topic_name = Globals::DEFAULT_DIAGNOSTIC_TOPIC;

	nh = nh_ptr;

	strncpy(ns, &nh->getNamespace().c_str()[1], nh->getNamespace().length());
}

void DiagnosticNode::subscribe()
{
	diagnostic_sub = nh->subscribe(topic_name, 1,
		&DiagnosticNode::diagnosticCallback, this, ros::TransportHints().unreliable().tcpNoDelay());
	std::cout << "Diagnostics: Subscribed to " << topic_name << std::endl;
}

void DiagnosticNode::unsubscribe()
{
	diagnostic_sub.shutdown();
}

void DiagnosticNode::diagnosticCallback(
	const diagnostic_msgs::DiagnosticArrayConstPtr &msg)
{
	for(unsigned int i = 0; i < msg->status.size(); i++)
	{
		// Process each diagnostic in the message
		// Only process messages from our bot
		if(!strncmp(msg->status[i].name.c_str(), ns, strlen(ns)))
		{
			//std::cout << "Diagnostics: Got: " << msg->status[i].name << " - " << msg->status[i].message << " - " << msg->status[i].hardware_id << std::endl;
			// Process each value in the diagnostic
			for(unsigned int j = 0; j < msg->status[i].values.size(); j++)
			{
				emit diagnosticDataReceived(msg->status[i].values[j].key.c_str(), msg->status[i].values[j].value.c_str());
				//std::cout << "Diagnostics: Recieved " << msg->status[i].values[j].key << " at " << msg->status[i].values[j].value << std::endl;
			}
		}
	}
}
