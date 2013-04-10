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
 * \file   joystick_node.cpp
 * \date   Oct, 2011
 * \author Scott K Logan, Matt Richard
 */
#include <vector>
#include "control_panel/nodes/joystick_node.h"

JoystickNode::JoystickNode(ros::NodeHandle *nh_ptr) : nh(nh_ptr)
{
	joy_topic_name = Globals::DEFAULT_JOYSTICK_TOPIC;
}

void JoystickNode::joyCallback(const sensor_msgs::JoyConstPtr &msg)
{
	unsigned int i;

	static std::vector<double> axes(msg->axes.size(), 0);
	static std::vector<bool> buttons(msg->buttons.size(), false);

	if((unsigned)axes.size() != msg->axes.size())
        axes.resize(msg->axes.size(), 0);
	if((unsigned)buttons.size() != msg->buttons.size())
        buttons.resize(msg->buttons.size(), false);

	for(i = 0; i < msg->axes.size(); i++)
	{
		if(msg->axes[i] != axes[i])
		{
			axes[i] = msg->axes[i];
//			std::cout << "Axis " << i << ": " << msg->axes[i] << std::endl;
			emit axis_event(i, msg->axes[i]);
		}
	}

	for(i = 0; i < msg->buttons.size(); i++)
	{
		if(msg->buttons[i] != buttons[i])
		{
			buttons[i] = msg->buttons[i];
//			std::cout << "Buttons " << i << ": " << buttons[i] << std::endl;
			emit button_event(i, msg->buttons[i]);
		}
	}
}

void JoystickNode::subscribe()
{
	joy_sub = nh->subscribe(joy_topic_name, 1, &JoystickNode::joyCallback, this,
		ros::TransportHints().unreliable().tcpNoDelay());
}

void JoystickNode::unsubscribe()
{
	joy_sub.shutdown();
}
