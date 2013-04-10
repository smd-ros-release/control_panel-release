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
 * \file   robot_tab.cpp
 * \date   Aug 6, 2011
 * \author Matt Richard, Scott K Logan
 */
#include <QtGui>
#include <QMetaType>
#include "control_panel/robot_tab.h"
#include <stdio.h>

RobotTab::RobotTab(RobotConfig *new_robot_config, QWidget *parent) :
	QWidget(parent), raw_data_display(NULL), processed_data_display(NULL)
{
	robot_config = new_robot_config;
	use_keyboard = false;

    raw_data_display = new DisplayPane(this);
    raw_data_display->setTitle("Raw Data");

    for(unsigned int i = 0; i < robot_config->sensors.cameras.size(); i++)
        raw_data_display->addSource(robot_config->sensors.cameras[i].name);

    for(unsigned int i = 0; i < robot_config->sensors.lasers.size(); i++)
        raw_data_display->addSource(robot_config->sensors.lasers[i].name);

    // Create processed display pane if the config file has processed data
    if(!robot_config->processedData.maps.empty() || !robot_config->processedData.images.empty() ||
       !robot_config->processedData.disparity_images.empty())
    {
        processed_data_display = new DisplayPane(this);
        processed_data_display->setTitle("Processed Data");

        for(unsigned int i = 0; i < robot_config->processedData.maps.size(); i++)
            processed_data_display->addSource(robot_config->processedData.maps[i].name);

        for(unsigned int i = 0; i < robot_config->processedData.images.size(); i++)
            processed_data_display->addSource(robot_config->processedData.images[i].name);

        for(unsigned int i = 0; i < robot_config->processedData.disparity_images.size(); i++)
            processed_data_display->addSource(robot_config->processedData.disparity_images[i].name);
    }


	node_manager = new NodeManager(robot_config);

    setupDataPane();

	/**
     * Connections
     **/
	connect(raw_data_display, SIGNAL(changeSource(const std::string &)),
		node_manager, SLOT(changeRawDataSource(const std::string &)));
    if(processed_data_display)
        connect(processed_data_display, SIGNAL(changeSource(const std::string &)),
            node_manager, SLOT(changeProcessedDataSource(const std::string &)));
	connect(node_manager, SIGNAL(connectionStatusChanged(int)),
		this, SLOT(updateConnectionStatus(int)));
	if(node_manager->camera_node)
		connect(node_manager->camera_node, SIGNAL(frameReceived(const QImage &)),
			raw_data_display, SLOT(setImage(const QImage &)));
    if(node_manager->laser_node)
        connect(node_manager->laser_node, SIGNAL(laserScanReceived(const QImage &, int)),
            raw_data_display, SLOT(setImage(const QImage &, int)));
    if(node_manager->image_node)
        connect(node_manager->image_node, SIGNAL(frameReceived(const QImage &)),
            processed_data_display, SLOT(setImage(const QImage &)));
	if(node_manager->disparity_image_node)
        connect(node_manager->disparity_image_node, SIGNAL(disparityImageReceived(const QImage &)),
            processed_data_display, SLOT(setImage(const QImage &)));
    if(node_manager->map_node)
        connect(node_manager->map_node, SIGNAL(mapReceived(const QImage &, double, double, float)),
            processed_data_display, SLOT(setMap(const QImage &, double, double, float)));
    if(node_manager->diagnostic_node && !robot_config->diagnostics.batteryLevel.empty())
		connect(node_manager->diagnostic_node, SIGNAL(diagnosticDataReceived(const QString &, const QString &)),
			this, SLOT(processDiagnostic(const QString &, const QString &)));
    if(!robot_config->diagnostics.batteryLevel.empty())
        data_pane->showBatteryDisplay(true);


	createLayout();
	setLayout(display_layout);
    setFocusPolicy(Qt::ClickFocus);
}

/******************************************************************************
 * Function:    ~RobotTab
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     None
 * Description: Deconstructor:
 *****************************************************************************/
RobotTab::~RobotTab()
{
	delete data_pane;

    if(raw_data_display)
	    delete raw_data_display;

    if(processed_data_display)
        delete processed_data_display;
	
    delete node_manager;
}

/******************************************************************************
 * Function:    robotConnected
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     bool - true if the robot is connected, otherwise false.
 * Description: Returns the connection state of the Control Panel with the
 *              loaded robot.
 *****************************************************************************/
bool RobotTab::robotConnected()
{
	return node_manager->isRunning();
}

/******************************************************************************
 * Function:    disconnectRobot
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     void
 * Description: Stops the connection thread with the robot.
 *****************************************************************************/
void RobotTab::disconnectRobot()
{
	node_manager->stop();
}

/******************************************************************************
 * Function:    connectToRobot
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     void
 * Description: Starts the connection thread with the robot.
 *****************************************************************************/
void RobotTab::connectToRobot()
{
	node_manager->start();//QThread::NormalPriority);
}

/******************************************************************************
 * Function:    getConfig
 * Author:      Scott Logan
 * Parameters:  None
 * Returns:     struct RobotConfig * -
 * Description: Returns the loaded robot's configuration file.
 *****************************************************************************/
struct RobotConfig * RobotTab::getConfig()
{
	return robot_config;
}

/******************************************************************************
 * Function:    createLayout
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     void
 * Description: Creates the display panes layout for the tab.
 *****************************************************************************/
void RobotTab::createLayout()
{
	QFrame *horizontal_separator = new QFrame;
	horizontal_separator->setFrameShape(QFrame::HLine);
	horizontal_separator->setFrameShadow(QFrame::Sunken);

	QFrame *vertical_separator = new QFrame;
	vertical_separator->setFrameShape(QFrame::VLine);
	vertical_separator->setFrameShadow(QFrame::Sunken);

	QHBoxLayout *video_map_layout = new QHBoxLayout;
	video_map_layout->addWidget(raw_data_display);
    // @todo
    // If there is processed items in config file, change below
    if(processed_data_display)
    {
	    video_map_layout->addWidget(vertical_separator);
	    video_map_layout->addWidget(processed_data_display);
    }

	display_layout = new QVBoxLayout;
    //display_layout->setSpacing(0);
	display_layout->addWidget(data_pane);
	display_layout->addWidget(horizontal_separator);
	display_layout->addLayout(video_map_layout);
}

/******************************************************************************
 * Function:    updateConnectionStatus
 * Author:      Matt Richard
 * Parameters:  int status - 
 * Returns:     void
 * Description: 
 *****************************************************************************/
void RobotTab::updateConnectionStatus(int status)
{
    data_pane->connectionStatusChanged(status);

    if(raw_data_display)
        raw_data_display->connectionStatusChanged(status);

    if(processed_data_display)
	    processed_data_display->connectionStatusChanged(status);

	emit connectionStatusChanged(status, robot_config->getRobotName());
}

/******************************************************************************
 * Function:    keyPressEvent
 * Author:      Matt Richard
 * Parameters:  QKeyEvent *event - 
 * Returns:     void
 * Description: Overloaded function.
 *****************************************************************************/
void RobotTab::keyPressEvent(QKeyEvent *event)
{
	// Ignore the event if the robot is not connect or the key is auto repeat
	if(!use_keyboard || !node_manager->isConnected() || event->isAutoRepeat()
        || !node_manager->control_node)
	{
		event->ignore();
		return;
	}

	switch(event->key())
	{
		case Qt::Key_W: // Move forward
			node_manager->control_node->setLinearX(1.0);
			break;

		case Qt::Key_S: // Move backward
			node_manager->control_node->setLinearX(-1.0);
			break;

		case Qt::Key_A: // Turn left
			node_manager->control_node->setAngularZ(1.0);
			break;

		case Qt::Key_D: // Turn right
			node_manager->control_node->setAngularZ(-1.0);
			break;

        case Qt::Key_Q: // Stride left
            node_manager->control_node->setLinearY(1.0);
            break;

        case Qt::Key_E: // Stride right
            node_manager->control_node->setLinearY(-1.0);

		default:
			event->ignore();
			return;
	}
}

/******************************************************************************
 * Function:    keyReleaseEvent
 * Author:      Matt Richard
 * Parameters:  QKeyEvent *event - 
 * Returns:     void
 * Description: Overloaded function.
 *****************************************************************************/
void RobotTab::keyReleaseEvent(QKeyEvent *event)
{
	// Ignore the event if the robot is not connect or the key is auto repeat
	if(!use_keyboard || !node_manager->isConnected() || event->isAutoRepeat()
        || !node_manager->control_node)
	{
		event->ignore();
		return;
	}

	switch(event->key())
	{
		case Qt::Key_W: // Stop forward or backward movement
		case Qt::Key_S:
			node_manager->control_node->setLinearX(0.0);
			break;

		case Qt::Key_A: // Stop rotational movement
		case Qt::Key_D:
			node_manager->control_node->setAngularZ(0.0);
			break;

        case Qt::Key_Q:
        case Qt::Key_E:
            node_manager->control_node->setLinearY(0.0);
            break;

		default:
			event->ignore();
			return;
	}
}

void RobotTab::callService()
{
    unsigned int i = 0;
    bool found_srv = false;

    if(node_manager->isConnected() && node_manager->command_node)
    {
        QString srv_name;
        QStringList services;

        for(i = 0; i < robot_config->commands.custom.size(); i++)
            services << robot_config->commands.custom[i].name;

        CallServiceDialog call_srv_dialog(services);
        if(call_srv_dialog.exec())
        {
            srv_name = call_srv_dialog.getSelectedService();

            for(i = 0; i < robot_config->commands.custom.size() && !found_srv; i++)
                if(robot_config->commands.custom[i].name == srv_name)
                    found_srv = true;

            if(found_srv)
                node_manager->command_node->callEmpty(robot_config->commands.custom[i-1].topicName);
            else
                std::cerr << "Selected service was not found in the robot configuration file" << std::endl;
        }
    }
}

void RobotTab::setRCMode(int rc_mode)
{
    if(rc_mode == Globals::Disabled)
    {
        use_keyboard = false;

        if(node_manager->controlNodeEnabled())
            node_manager->enableControlNode(false);

        if(robot_config->controls.used)
            node_manager->joystick_node->unsubscribe();

        data_pane->setRCModeText("Disabled");
    }
    else if(rc_mode == Globals::Keyboard)
    {
        use_keyboard = true;

        if(!node_manager->controlNodeEnabled())
            node_manager->enableControlNode(true);

        if(robot_config->controls.used)
            node_manager->joystick_node->unsubscribe();

        data_pane->setRCModeText("Keyboard");
    }
    else if(rc_mode == Globals::Joystick)
    {
        use_keyboard = false;

        if(!node_manager->controlNodeEnabled())
            node_manager->enableControlNode(true);

        if(robot_config->controls.used)
            node_manager->joystick_node->subscribe();

        data_pane->setRCModeText("Joystick");
    }
    else
        printf("Unknown RC mode: %d\n", rc_mode);
}

/******************************************************************************
 * Function:    isKeyboardEnabled
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     bool - True is keyboard is enabled, otherwise false
 * Description: Returns the state of keyboard remote control.
 *****************************************************************************/
bool RobotTab::isKeyboardEnabled() const
{
	return use_keyboard;
}

/******************************************************************************
 * Function:    processDiagnostic
 * Author:      Scott K Logan
 * Parameters:  QString key - type of diagnostic
 *              QString val - value reported
 * Returns:     void
 * Description: Processes an incoming diagnostic
 *****************************************************************************/
void RobotTab::processDiagnostic(const QString &key, const QString &val)
{
	for(unsigned int i = 0; i < robot_config->diagnostics.batteryLevel.size(); i++)
		if(robot_config->diagnostics.batteryLevel[i].name == key)
			data_pane->updateBatteryData(val.toFloat());

	for(unsigned int i = 0; i < robot_config->diagnostics.voltage.size(); i++)
		if(robot_config->diagnostics.voltage[i].name == key)
		{
			// Do Something
		}
}


void RobotTab::setupDataPane()
{
    data_pane = new DataPane(this);

    for(unsigned int i = 0; i < robot_config->processedData.odometry.size(); i++)
    {
        OdometryNode *odom = node_manager->addOdometryNode(robot_config->processedData.odometry[i].topicName.toStdString());
        connect(
            odom,
            SIGNAL(odometryDataReceived(const QVector3D &, const QQuaternion &,
                                        const QVector3D &, const QVector3D &)),
            data_pane->addOdometryDisplay(robot_config->processedData.odometry[i].name, robot_config->processedData.odometry[i].position,
                                          robot_config->processedData.odometry[i].orientation, robot_config->processedData.odometry[i].linearVelocity,
                                          robot_config->processedData.odometry[i].angularVelocity, !robot_config->processedData.odometry[i].hideAttitude,
                                          !robot_config->processedData.odometry[i].hideHeading),
            SLOT(updateOdometryDisplay(const QVector3D &, const QQuaternion &,
                                       const QVector3D &, const QVector3D &))
            );

        if(robot_config->processedData.maps.size() && robot_config->processedData.odometry[i].updateMap)
        {
            connect(odom, SIGNAL(odometryDataReceived(const QVector3D &, const QQuaternion &, const QVector3D &, const QVector3D &)),
                processed_data_display, SLOT(setPosition(const QVector3D &)));
        
            connect(node_manager, SIGNAL(mapSubscribed(bool)),
                processed_data_display, SLOT(showPosition(bool)));
        }
    }

    for(unsigned int i = 0; i < robot_config->processedData.pose.size(); i++)
    {
        PoseNode *pose = node_manager->addPoseNode(robot_config->processedData.pose[i].topicName.toStdString(), robot_config->processedData.pose[i].isStamped, robot_config->processedData.pose[i].hasCovariance );
        connect(
            pose,
            SIGNAL(poseDataReceived(const QVector3D &, const QQuaternion &,
                                        const QVector3D &, const QVector3D &)),
            data_pane->addOdometryDisplay(robot_config->processedData.pose[i].name, robot_config->processedData.pose[i].position,
                                          robot_config->processedData.pose[i].orientation, false,
                                          false, !robot_config->processedData.pose[i].hideAttitude,
                                          !robot_config->processedData.pose[i].hideHeading),
            SLOT(updateOdometryDisplay(const QVector3D &, const QQuaternion &,
                                       const QVector3D &, const QVector3D &))
            );

        if(robot_config->processedData.maps.size() && robot_config->processedData.pose[i].updateMap)
        {
            connect(pose, SIGNAL(poseDataReceived(const QVector3D &, const QQuaternion &, const QVector3D &, const QVector3D &)),
                processed_data_display, SLOT(setPosition(const QVector3D &)));
        
            connect(node_manager, SIGNAL(mapSubscribed(bool)),
                processed_data_display, SLOT(showPosition(bool)));
        }
    }

    // Create nodes and widgets for all imu sensors
    for(unsigned int i = 0; i < robot_config->sensors.imu.size(); i++)
    {
        connect(
            node_manager->addImuNode(robot_config->sensors.imu[i].topicName.toStdString()),
            SIGNAL(imuDataReceived(const QQuaternion &, const QVector3D &,
                                   const QVector3D &)),
            data_pane->addImuDisplay(robot_config->sensors.imu[i].name, robot_config->sensors.imu[i].roll, robot_config->sensors.imu[i].pitch,
                                     robot_config->sensors.imu[i].yaw, robot_config->sensors.imu[i].angularVelocity, robot_config->sensors.imu[i].linearAcceleration,
                                     !robot_config->sensors.imu[i].hideAttitude, !robot_config->sensors.imu[i].hideHeading),
            SLOT(updateImuDisplay(const QQuaternion &, const QVector3D &,
                                  const QVector3D &))
            );
    }


    qRegisterMetaType< std::vector<double> >("std::vector<double>");
    if(robot_config->joint_states.used)
    {
        // Group all joints and their display names in string lists
        QStringList name_list;
        QStringList disp_name_list;
        for(unsigned int i = 0; i < robot_config->joint_states.joints.size(); i++)
        {
            name_list << robot_config->joint_states.joints[i].name;
            disp_name_list << robot_config->joint_states.joints[i].displayName;
        }

        node_manager->joint_state_node->setTopic(robot_config->joint_states.topicName.toStdString());
        connect(
            node_manager->joint_state_node,
            SIGNAL(jointDataReceived(const QStringList &, const std::vector<double> &,
                                     const std::vector<double> &, const std::vector<double> &)),
            data_pane->addJointStateDisplay("Joints", name_list, disp_name_list,
                                            robot_config->joint_states.position,
                                            robot_config->joint_states.velocity,
                                            robot_config->joint_states.effort),
            SLOT(updateJointStateDisplay(const QStringList &, const std::vector<double> &,
                                         const std::vector<double> &, const std::vector<double> &))
            );
    }

    // Create nodes and widgets for all gps sensors
    for(unsigned int i = 0; i < robot_config->sensors.gps.size(); i++)
    {
        connect(node_manager->addGpsNode(robot_config->sensors.gps[i].topicName.toStdString()),
                SIGNAL(gpsDataReceived(double, double, double)),
                data_pane->addGpsDisplay(robot_config->sensors.gps[i].name, robot_config->sensors.gps[i].latitude,
                    robot_config->sensors.gps[i].longitude, robot_config->sensors.gps[i].altitude),
                SLOT(updateGpsDisplay(double, double, double)));
    }

    if( robot_config->sensors.range.size( ) > 0 )
    {
        connect(node_manager->addRangeNode(robot_config->sensors.range[0].topicName.toStdString()), SIGNAL(rangeReceived(float, bool)),
            data_pane, SLOT(updateRange(float)));
        data_pane->showRangeLabel(true);
    }
}
