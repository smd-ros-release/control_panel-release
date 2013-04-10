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
 * \file   qt_node.cpp
 * \date   Feb 3, 2012
 * \author Matt Richard
 */
#include "control_panel/nodes/qt_node.h"
#include "ros/network.h"
#include <string.h>

QtNode::QtNode(int argc, char **argv, const std::string &name)
    : service_node(NULL), arg_count(argc), arg_vec(argv), node_name(name), nh(NULL)
{ }

QtNode::~QtNode()
{
    stop();

    // Destroy node handle
    if(nh)
    {
        nh->shutdown();
        delete nh;
    }
}

bool QtNode::init()
{
    if(ros::isStarted())
    {
        ROS_ERROR("%s node is already running\n", node_name.c_str());
        return false;
    }

    ROS_INFO("Initializing ROS node");
    ros::init(arg_count, arg_vec, node_name, ros::init_options::NoSigintHandler);

    // Verify we have connection with the master
    if(!ros::master::check())
    {
        ROS_ERROR("Could not connect to master");
        return false;
    }

    nh = new ros::NodeHandle();

    start(); // start thread

    return true;
}

void QtNode::run()
{
    service_node = new CommandNode(nh);

    ros::spin();

    delete service_node;
}

void QtNode::stop()
{
    exit();
    wait();
}

void QtNode::setNodeName(const std::string &name)
{
    if(name != "")
        node_name = name;
}
