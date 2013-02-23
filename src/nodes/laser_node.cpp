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
 * \file   laser_node.cpp
 * \date   Nov 16, 2011
 * \author Matt Richard, Scott K Logan
 */
#include "control_panel/nodes/laser_node.h"

LaserNode::LaserNode(ros::NodeHandle *nh_ptr)
{
	topic_name = Globals::DEFAULT_LASER_TOPIC;

	nh = nh_ptr;

	white = qRgb(255, 255, 255);
	red = qRgb(255, 0, 0);
}

void LaserNode::subscribe()
{
	laser_sub = nh->subscribe(topic_name, 1, &LaserNode::laserCallback, this,
		ros::TransportHints().unreliable().tcpNoDelay());
}

void LaserNode::unsubscribe()
{
	laser_sub.shutdown();
}

void LaserNode::laserCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
	int scale = 40; // scale size of image
	double x, y;

	int robot_pos = msg->range_max * scale;
	//int height = robot_pos * (1.0 - cos(msg->angle_max)) + 10;
	int height = robot_pos * 2.0;

	QImage buffer(2.0 * robot_pos, height, QImage::Format_RGB888);
	buffer.fill(Qt::black);

 	double curr_angle = msg->angle_min + Globals::PI_OVER_TWO;

	// Convert from polar to x,y coordinates and create the image
	for(unsigned int i = 0; i < msg->ranges.size(); i++)
	{
		// Make sure the range is valid
		if(msg->ranges[i] > msg->range_min && msg->ranges[i] < (msg->range_max - 0.1))
		{
			/* x = r * cos(theta) */
			x = msg->ranges[i] * cos(curr_angle);
			/* y = r * sin(theta) */
			y = msg->ranges[i] * sin(curr_angle);

			// draw the laser value on the image
			buffer.setPixel(x * scale + (robot_pos - 1), (robot_pos - 1) - y * scale, white);
		}

		curr_angle += msg->angle_increment;
	}


	// Draw the robots position on the image
	for(int i = robot_pos - 1; i > robot_pos - 6; i--)
		for(int j = robot_pos - 3; j < robot_pos + 3; j++)
			buffer.setPixel(j, i, red);

	emit laserScanReceived(buffer, scale);
}
