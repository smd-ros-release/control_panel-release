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
 * \file   node_manager.h
 * \date   Aug. 4, 2011
 * \author Matt Richard, Scott K Logan
 */
#ifndef CONTROL_PANEL_NODE_MANAGER_H
#define CONTROL_PANEL_NODE_MANAGER_H

#include <QThread>
#include <QTimer>
#include <QList>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include <string>
#include "nodes/control_node.h"
#include "nodes/command_node.h"
#include "nodes/diagnostic_node.h"
#include "nodes/disparity_image_node.h"
#include "nodes/gps_node.h"
#include "nodes/image_node.h"
#include "nodes/imu_node.h"
#include "nodes/joint_state_node.h"
#include "nodes/joint_trajectory_node.h"
#include "nodes/joystick_node.h"
#include "nodes/laser_node.h"
#include "nodes/map_node.h"
#include "nodes/odometry_node.h"
#include "nodes/pose_node.h"
#include "nodes/range_node.h"
#include "globals.h"
#include "robot_config.h"


/**
 * \class NodeManager
 * \brief Manages all ROS nodes for communicating with a robot
 */
class NodeManager : public QThread
{
	Q_OBJECT

	public:
		NodeManager(struct RobotConfig *);
		bool controlNodeEnabled() const { return control_node_enabled; }
		void enableControlNode(bool enable);
		void run();
		void stop();
		bool isConnected() const;
		GpsNode *addGpsNode(const std::string &topic = Globals::DEFAULT_GPS_TOPIC);
		ImuNode *addImuNode(const std::string &topic = Globals::DEFAULT_IMU_TOPIC);
		OdometryNode *addOdometryNode(const std::string &topic = Globals::DEFAULT_ODOMETRY_TOPIC);
		PoseNode *addPoseNode(const std::string &topic = Globals::DEFAULT_POSE_TOPIC, bool isStamped = false, bool hasCovariance = false);
		RangeNode *addRangeNode(const std::string &topic = Globals::DEFAULT_RANGE_TOPIC);

		// ROS Nodes
		ImageNode *camera_node; // For raw images (e.g., camera/depth image feeds)
		ImageNode *image_node;  // For processed images (e.g., rectified images)
		ControlNode *control_node;
		CommandNode *command_node;
		DiagnosticNode *diagnostic_node;
		DisparityImageNode *disparity_image_node;
		GpsNode *gps_node;
		ImuNode *imu_node;
		JointStateNode *joint_state_node;
		JointTrajectoryNode *joint_trajectory_node;
		JoystickNode *joystick_node;
		LaserNode *laser_node;
		MapNode *map_node;
		OdometryNode *odometry_node;
		PoseNode *pose_node;
		RangeNode *range_node;

	public slots:
		void changeRawDataSource(const std::string &source);
		void changeProcessedDataSource(const std::string &source);
		void joystickAxisChanged(int axis, double value);
		void joystickButtonChanged(int button, bool state);

	signals:
		void connectionStatusChanged(int status);
		void mapSubscribed(bool sub);

	private:
		ros::NodeHandle *nh_ptr;
		ros::CallbackQueue robot_callback_queue;
		struct RobotConfig *robot_config;
		bool connected;
		bool control_node_enabled;
		QTimer *pub_timer;

		QList<ImuNode *> *imu_node_list;
		QList<GpsNode *> *gps_node_list;
		QList<OdometryNode *> *odom_node_list;
		QList<PoseNode *> *pose_node_list;
};

#endif // CONTROL_PANEL_NODE_MANAGER_H
