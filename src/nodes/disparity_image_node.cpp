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
 * \file   disparity_image_node.cpp
 * \date   Feb 5, 2012
 * \author Matt Richard
 */
#include "control_panel/nodes/disparity_image_node.h"

DisparityImageNode::DisparityImageNode(ros::NodeHandle *nh_ptr)
{
	topic_name = "stereo/disparity";

	nh = nh_ptr;
}

void DisparityImageNode::subscribe()
{
	disparity_sub = nh->subscribe<stereo_msgs::DisparityImage>(topic_name,
		1, &DisparityImageNode::disparityCallback, this);
}

void DisparityImageNode::unsubscribe()
{
	disparity_sub.shutdown();
}

void DisparityImageNode::disparityCallback(const stereo_msgs::DisparityImageConstPtr &msg)
{
	// Verify the image encoding is a disparity image encoding
	if(msg->image.encoding != enc::TYPE_32FC1)
	{
		ROS_ERROR("Unusable disparity image encoding '%s'", msg->image.encoding.c_str());
		return;
	}

	float mult = 255.0 / (msg->max_disparity - msg->min_disparity);

	const cv::Mat_<float> d_mat(msg->image.height, msg->image.width,
		(float *)&msg->image.data[0], msg->image.step);

	cv::Mat_<cv::Vec3b> disp_color;
	disp_color.create(msg->image.height, msg->image.width);

	// Create the disparity color image
	for(int i = 0; i < disp_color.rows; i++)
	{
		const float *d = d_mat[i];

		for(int j = 0; j < disp_color.cols; j++)
		{
			int index = (d[j] - msg->min_disparity) * mult + 0.5;
			index = std::min(255, std::max(0, index));

			disp_color(i, j)[2] = color_table[3 * index];
			disp_color(i, j)[1] = color_table[1 + (3 * index)];
			disp_color(i, j)[0] = color_table[2 + (3 * index)];
		}
	}

	// Create the QImage from the disparity color image
	QImage buffer((unsigned char *)disp_color.data, disp_color.cols,
		disp_color.rows, QImage::Format_RGB888);

	emit disparityImageReceived(buffer.rgbSwapped());
}
