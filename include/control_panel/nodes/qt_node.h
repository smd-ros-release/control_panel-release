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
 * \file   qt_node.h
 * \date   Feb 3, 2012
 * \author Matt Richard
 */
#ifndef CONTROL_PANEL_QT_NODE_H
#define CONTROL_PANEL_QT_NODE_H

#include <QThread>
#include "ros/ros.h"
#include <string>
#include "command_node.h"
//#include "joystick_node.h"
//#include "diagnostics_node.h"


/**
 * \class QtNode
 * \brief Initilizes and shutsdown a ROS node. This also handles the global joystick input, diagnostics, and services
 */
class QtNode : public QThread
{
    Q_OBJECT

    public:
        QtNode(int argc, char **argv, const std::string &name = "qt_node");
        ~QtNode();

        /**
         * \brief Returns the name of the node
         */
        std::string getNodeName() const { return node_name; }

        /**
         * \brief
         */
        bool init();

        /**
         * \brief
         */
        void run();
        void stop();
        void setNodeName(const std::string &name);

        CommandNode *service_node;

    private:
        int arg_count;
        char **arg_vec;
        std::string node_name;
        ros::NodeHandle *nh;
};

#endif // CONTROL_PANEL_QT_NODE_H
