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
 * \file   map_node.cpp
 * \date   Nov 19, 2011
 * \author Matt Richard
 */
#include "control_panel/nodes/map_node.h"

MapNode::MapNode(ros::NodeHandle *nh_ptr)
{
	topic_name = Globals::DEFAULT_MAP_TOPIC;

	nh = nh_ptr;

	white = qRgb(255, 255, 255);
	black = qRgb(0, 0, 0);
	grey = qRgb(50, 50, 50);
}

void MapNode::subscribe()
{
	map_sub = nh->subscribe(topic_name, 1, &MapNode::mapCallback, this);
}

void MapNode::unsubscribe()
{
	map_sub.shutdown();
}

void MapNode::mapCallback(const nav_msgs::OccupancyGridConstPtr &msg)
{
	unsigned int i = 0;

	QImage buffer(msg->info.width, msg->info.height, QImage::Format_RGB888);
	buffer.fill(grey);

	for(unsigned int y = 0; y < msg->info.height; y++)
	{
		for(unsigned int x = 0; x < msg->info.width; x++)
		{
			// Find (x,y) index into map data array
			i = x + (msg->info.height - y - 1) * msg->info.width;

			if(msg->data[i] == 0) // occupancy [0, 0.1)
				buffer.setPixel(x, y, white);
			else if(msg->data[i] == +100) // occupancy (0.65, 1]
				buffer.setPixel(x, y, black);
			//else // occupancy [0.1, 0.65]
			//    buffer.setPixel(x, y, grey);
		}
	}

	emit mapReceived(buffer, msg->info.origin.position.x,
		msg->info.origin.position.y, msg->info.resolution);
}
