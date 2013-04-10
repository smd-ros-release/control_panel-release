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
 * \file   image_node.cpp
 * \date   Sept 8, 2011
 * \author Matt Richard
 */
#include "control_panel/nodes/image_node.h"

ImageNode::ImageNode(ros::NodeHandle *nh_ptr)
{
	topic_name = Globals::DEFAULT_CAMERA_TOPIC;

	nh = nh_ptr;

	it = new image_transport::ImageTransport(*nh);
}

ImageNode::~ImageNode()
{
	unsubscribe();

	delete it;
}

void ImageNode::subscribe()
{
	image_sub = it->subscribe(topic_name, 1, &ImageNode::imageCallback, this);//,
		//image_transport::TransportHints("raw", ros::TransportHints().unreliable()));
}

void ImageNode::unsubscribe()
{
	image_sub.shutdown();
}

void ImageNode::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
	cv_bridge::CvImageConstPtr cv_ptr;

	// Convert the ROS image into a CvMat image
	if(enc::isColor(msg->encoding) || enc::isMono(msg->encoding))
	{
		cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);

		QImage buffer((unsigned char *)cv_ptr->image.data, cv_ptr->image.cols,
			cv_ptr->image.rows, QImage::Format_RGB888);

		emit frameReceived(buffer.rgbSwapped());
	}
/*
	else if(msg->encoding == enc::MONO16)
	{
		cv_ptr = cv_bridge::toCvShare(msg, enc::MONO16);

		cv::Mat depth_mat;
		cv::Mat depth_f;

		cv_ptr->image.convertTo(depth_mat, CV_8UC1, 255.0/2048.0);
		cv::cvtColor(depth_mat, depth_f, CV_GRAY2BGR);

		QImage buffer((unsigned char *)depth_f.data, depth_f.cols,
			depth_f.rows, QImage::Format_RGB888);

		emit frameReceived(buffer.rgbSwapped());
	}
*/
	else if(msg->encoding == enc::TYPE_32FC1)
	{
		cv_ptr = cv_bridge::toCvShare(msg, enc::TYPE_32FC1);

		cv::Mat depth_img;// = cv::Mat(cv_ptr->image.size(), CV_8UC3, cv::Scalar(0));//, 0, 255));
		cv::Mat temp_img;// = cv::Mat(cv_ptr->image.size(), CV_8UC1);

		cv_ptr->image.convertTo(temp_img, CV_8UC1, 40.0);//255.0/2048.0);
		cv::cvtColor(temp_img, depth_img, CV_GRAY2BGR);

		QImage buffer((unsigned char *)depth_img.data, depth_img.cols,
			depth_img.rows, QImage::Format_RGB888);

 		emit frameReceived(buffer.rgbSwapped());
	}
	else
		ROS_WARN("Unrecognized image encoding: %s\n", msg->encoding.c_str());
}
