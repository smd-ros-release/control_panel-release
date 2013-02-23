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
 * \file   range_node.cpp
 * \date   Jan, 2012
 * \author Matt Richard
 */
#include "control_panel/nodes/range_node.h"

RangeNode::RangeNode(ros::NodeHandle *nh_ptr)
{
	topic_name = Globals::DEFAULT_RANGE_TOPIC;

	nh = nh_ptr;
}

void RangeNode::subscribe()
{
	range_sub = nh->subscribe(topic_name, 1, &RangeNode::rangeCallback, this,
		ros::TransportHints().unreliable().tcpNoDelay());
}

void RangeNode::rangeCallback(const sensor_msgs::RangeConstPtr &msg)
{
	bool valid = true;

	if(msg->range != msg->range)
	{
		ROS_ERROR("NaN detected in Range message");
		return;
	}

	// Check if the range value is valid
	if(msg->range < msg->min_range || msg->range > msg->max_range)
		valid = false;

	emit rangeReceived(msg->range, valid);
}

void RangeNode::unsubscribe()
{
	range_sub.shutdown();
}
