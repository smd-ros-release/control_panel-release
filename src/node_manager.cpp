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
 * \file   node_manager.cpp
 * \date   Aug 4, 2011
 * \author Matt Richard, Scott K Logan
 */
#include "control_panel/node_manager.h"
#include <stdio.h>

NodeManager::NodeManager(struct RobotConfig *new_robot_config) :
	camera_node(NULL), image_node(NULL), control_node(NULL), command_node(NULL),
	diagnostic_node(NULL), disparity_image_node(NULL), gps_node(NULL), imu_node(NULL),
	joint_state_node(NULL), joint_trajectory_node(NULL), laser_node(NULL),
	map_node(NULL), odometry_node(NULL), pose_node(NULL), range_node(NULL)
{
	connected = false;
	control_node_enabled = false;
	robot_config = new_robot_config;

	gps_node_list = new QList<GpsNode *>;
	imu_node_list = new QList<ImuNode *>;
	odom_node_list = new QList<OdometryNode *>;
	pose_node_list = new QList<PoseNode *>;

	// Create the node handle and set the local callback queue
	nh_ptr = new ros::NodeHandle(robot_config->nameSpace.toStdString());
	nh_ptr->setCallbackQueue(&robot_callback_queue);

	// Inititalize ROS nodes
	if(!robot_config->sensors.cameras.empty())
		camera_node = new ImageNode(nh_ptr);
	if(!robot_config->processedData.maps.empty())
		map_node = new MapNode(nh_ptr);
	if(!robot_config->processedData.images.empty())
		image_node = new ImageNode(nh_ptr);
	if(!robot_config->processedData.disparity_images.empty())
		disparity_image_node = new DisparityImageNode(nh_ptr);
	if(robot_config->controls.used)
	{
		control_node = new ControlNode(nh_ptr);
		control_node->setTopic(robot_config->controls.drive[0].topicName.toStdString());

		pub_timer = new QTimer(this);
		connect(pub_timer, SIGNAL(timeout()), control_node, SLOT(publish()));

		joystick_node = new JoystickNode(nh_ptr);
		connect(joystick_node, SIGNAL(axis_event(int, double)),
			this, SLOT(joystickAxisChanged(int, double)));
		connect(joystick_node, SIGNAL(button_event(int, bool)),
			this, SLOT(joystickButtonChanged(int, bool)));
	}
	if(robot_config->commands.used)
		command_node = new CommandNode(nh_ptr);
	if(robot_config->diagnostics.used)
		diagnostic_node = new DiagnosticNode(nh_ptr);
	if(robot_config->joint_states.used)
		joint_state_node = new JointStateNode(nh_ptr);
	if(!robot_config->sensors.lasers.empty())
		laser_node = new LaserNode(nh_ptr);
	/* TODO: Hack */
	if(!strncmp(robot_config->robotName.toStdString().c_str(), "Armbot", 6))
	{
		joint_trajectory_node = new JointTrajectoryNode(nh_ptr);
		joint_trajectory_node->setTopic("head_traj_controller/command");
		joint_trajectory_node->addJoint("head_pan_joint");
		joint_trajectory_node->advertise();
	}
}

/******************************************************************************
 * Function:    run
 * Author:      Matt Richard, Scott Logan
 * Parameters:  None
 * Returns:     void
 * Description: The thread.
 *****************************************************************************/
void NodeManager::run()
{
	int i;

	emit connectionStatusChanged(Globals::Connecting);

	// Start necessary ROS nodes
	if(diagnostic_node)
	{
		diagnostic_node->setTopic(robot_config->diagnostics.topicName.toStdString());
		diagnostic_node->subscribe();
	}
	for(i = 0; i < gps_node_list->count(); i++)
		((GpsNode *)gps_node_list->at(i))->subscribe();
	for(i = 0; i < imu_node_list->count(); i++)
		((ImuNode *)imu_node_list->at(i))->subscribe();
	for(i = 0; i < odom_node_list->count(); i++)
		((OdometryNode *)odom_node_list->at(i))->subscribe();
	for(i = 0; i < pose_node_list->count(); i++)
		((PoseNode *)pose_node_list->at(i))->subscribe();
	if(joint_state_node)
		joint_state_node->subscribe();
	if(odometry_node)
		odometry_node->subscribe();
	if(range_node)
		range_node->subscribe();

	connected = true;
	emit connectionStatusChanged(Globals::Connected);

	printf("Connected to %s\n", robot_config->getRobotName().toStdString().c_str());

	// Loop until disconnected
	while(connected)
		robot_callback_queue.callAvailable(ros::WallDuration(0.01));

	printf("Disconnected from %s\n", robot_config->getRobotName().toStdString().c_str());
}

/******************************************************************************
 * Function:    stop
 * Author:      Matt Richard, Scott Logan
 * Parameters:  None
 * Returns:     void
 * Description: Stops the connection thread.
 *****************************************************************************/
void NodeManager::stop()
{
	if(connected)
	{
		enableControlNode(false);

		connected = false;

		printf("Shutting down %s's node handle\n",
			robot_config->getRobotName().toStdString().c_str());

		// Shutdown node handle
		if(camera_node)
			camera_node->unsubscribe();
		if(image_node)
			image_node->unsubscribe();
		nh_ptr->shutdown();

		// Kill thread
		exit();
		wait();

		// Verify the thread was shutdown
		if(isRunning())
			printf("ERROR -- %s's thread failed to stop\n",
				robot_config->robotName.toStdString().c_str());

		emit connectionStatusChanged(Globals::Disconnected);
	}
}

/******************************************************************************
 * Function:    isConnected
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     bool - true if the robot is connected, otherwise false.
 * Description: Returns the connection state of the Control Panel with the
 *              robot.
 *****************************************************************************/
bool NodeManager::isConnected() const
{
	return connected;
}

/******************************************************************************
 * Function:    changeRawDataSource
 * Author:      Scott Logan
 * Parameters:  const std::string &source
 * Returns:     void
 * Description:
 *****************************************************************************/
void NodeManager::changeRawDataSource(const std::string &source)
{
	if(camera_node)
		camera_node->unsubscribe();
	if(laser_node)
		laser_node->unsubscribe();

	for(unsigned int i = 0; i < robot_config->sensors.cameras.size(); i++)
	{
		if(source == robot_config->sensors.cameras[i].name.toStdString())
		{
			camera_node->setTopic(robot_config->sensors.cameras[i].topicName.toStdString());
			camera_node->subscribe();
			return;
		}
	}

	for(unsigned int i = 0; i < robot_config->sensors.lasers.size(); i++)
	{
		if(source == robot_config->sensors.lasers[i].name.toStdString())
		{
			laser_node->setTopic(robot_config->sensors.lasers[i].topicName.toStdString());
			laser_node->subscribe();
			return;
		}
	}
}

void NodeManager::changeProcessedDataSource(const std::string &source)
{
	if(odometry_node)
		emit mapSubscribed(false);

	if(map_node)
		map_node->unsubscribe();
	if(image_node)
		image_node->unsubscribe();
	if(disparity_image_node)
		disparity_image_node->unsubscribe();

	for(unsigned int i = 0; i < robot_config->processedData.images.size(); i++)
	{
		if(source == robot_config->processedData.images[i].name.toStdString())
		{
			image_node->setTopic(robot_config->processedData.images[i].topicName.toStdString());
			image_node->subscribe();
			return;
		}
	}

	for(unsigned int i = 0; i < robot_config->processedData.maps.size(); i++)
	{
		if(source == robot_config->processedData.maps[i].name.toStdString())
		{
			map_node->setTopic(robot_config->processedData.maps[i].topicName.toStdString());
			map_node->subscribe();

			if(odometry_node)
				emit mapSubscribed(true);

			return;
		}
	}

	for(unsigned int i = 0; i < robot_config->processedData.disparity_images.size(); i++)
	{
		if(source == robot_config->processedData.disparity_images[i].name.toStdString())
		{
			disparity_image_node->setTopic(robot_config->processedData.disparity_images[i].topicName.toStdString());
			disparity_image_node->subscribe();
			return;
		}
	}
}

GpsNode *NodeManager::addGpsNode(const std::string &topic)
{
	gps_node = new GpsNode(nh_ptr);
	gps_node->setTopic(topic);

	gps_node_list->append(gps_node);

	return gps_node;
}

ImuNode *NodeManager::addImuNode(const std::string &topic)
{
	imu_node = new ImuNode(nh_ptr);
	imu_node->setTopic(topic);

	imu_node_list->append(imu_node);

	return imu_node;
}


OdometryNode *NodeManager::addOdometryNode(const std::string &topic)
{
	odometry_node = new OdometryNode(nh_ptr);
	odometry_node->setTopic(topic);

	odom_node_list->append(odometry_node);

	return odometry_node;
}

PoseNode *NodeManager::addPoseNode(const std::string &topic, bool isStamped, bool hasCovariance)
{
	pose_node = new PoseNode(nh_ptr, isStamped, hasCovariance);
	pose_node->setTopic(topic);

	pose_node_list->append(pose_node);

	return pose_node;
}

RangeNode *NodeManager::addRangeNode(const std::string &topic)
{
    range_node = new RangeNode(nh_ptr);
    range_node->setTopic(topic);

    // @todo
    // Store pointer.

    return range_node;
}

void NodeManager::joystickAxisChanged(int axis, double value)
{
	/* TODO: Hack */
	static double l_shoulder = 0;
	static double r_shoulder = 0;

	if(connected)
	{
		// Joystick mapping here

		// cmd_vel
		if(axis == 0 && control_node)
			control_node->setLinearY(value);
		else if(axis == 1 && control_node)
			control_node->setLinearX(value);
		else if(axis == 2 && control_node)
			control_node->setAngularZ(value);
		else if(axis == 3 && control_node)
			control_node->setLinearZ(value);

		// joints
		else if((axis == 12 || axis == 13) && joint_trajectory_node)
		{
			value = ( value - 1 ) / 2.0;
			if(axis == 13)
				l_shoulder = value;
			else
				r_shoulder = value;
			std::cout << "Sending joint trajectory: " << (l_shoulder - r_shoulder) * M_PI / 2 << std::endl;
			joint_trajectory_node->setPosition("head_pan_joint", (l_shoulder - r_shoulder) * M_PI / 2);
			joint_trajectory_node->publish();
		}
	}
}

void NodeManager::joystickButtonChanged(int button, bool state)
{
	if(connected && state)
	{
		// Joystick mapping here
		if(button == 4)
		{
			// Flip Forward Message
			for(unsigned int i = 0; i < robot_config->commands.custom.size(); i++)
				if(robot_config->commands.custom[i].name == "Flip Forward")
				    command_node->callEmpty(robot_config->commands.custom[i].topicName);
		}
		else if(button == 5)
		{
			// Flip Right Message
			for(unsigned int i = 0; i < robot_config->commands.custom.size(); i++)
				if(robot_config->commands.custom[i].name == "Flip Right")
				    command_node->callEmpty(robot_config->commands.custom[i].topicName);
		}
		else if(button == 6)
		{
			// Flip Backward Message
			for(unsigned int i = 0; i < robot_config->commands.custom.size(); i++)
				if(robot_config->commands.custom[i].name == "Flip Backward")
				    command_node->callEmpty(robot_config->commands.custom[i].topicName);
		}
		else if(button == 7)
		{
			// Flip Left Message
			for(unsigned int i = 0; i < robot_config->commands.custom.size(); i++)
				if(robot_config->commands.custom[i].name == "Flip Left")
				    command_node->callEmpty(robot_config->commands.custom[i].topicName);
		}
		else if(button == 14)
		{
			// Takeoff Message
			for(unsigned int i = 0; i < robot_config->commands.custom.size(); i++)
				if(robot_config->commands.custom[i].name == "takeoff")
				    command_node->callEmpty(robot_config->commands.custom[i].topicName);
		}
		else if(button == 13)
		{
			// Land Message
			for(unsigned int i = 0; i < robot_config->commands.custom.size(); i++)
				if(robot_config->commands.custom[i].name == "land")
				    command_node->callEmpty(robot_config->commands.custom[i].topicName);
		}
		else if(button == 12)
		{
			// Reset Message
			for(unsigned int i = 0; i < robot_config->commands.custom.size(); i++)
				if(robot_config->commands.custom[i].name == "Reset")
				    command_node->callEmpty(robot_config->commands.custom[i].topicName);
		}
		else if(button == 15)
		{
			// Land Message
			for(unsigned int i = 0; i < robot_config->commands.custom.size(); i++)
				if(robot_config->commands.custom[i].name == "Camera Toggle")
				    command_node->callEmpty(robot_config->commands.custom[i].topicName);
		}
	}
}

void NodeManager::enableControlNode(bool enable)
{
	if(connected && control_node)
	{
		if(enable)
		{
			control_node->advertise();
			pub_timer->start(30);
			control_node_enabled = true;
		}
		else
		{
			pub_timer->stop();
			control_node->unadvertise();
			control_node_enabled = false;
		}
	}
}
