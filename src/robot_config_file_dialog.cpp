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
 * \file   robot_config_file_dialog.cpp
 * \date   Dec 6, 2011
 * \author Matt Richard, Scott Logan
 */
#include <QtGui>
#include <iostream>
#include "control_panel/robot_config_file_dialog.h"
#include "stdio.h"


Qt::CheckState boolToCheckState(bool checked)
{
	if(checked)
		return Qt::Checked;
	return Qt::Unchecked;
}

bool checkStateToBool(Qt::CheckState state)
{
	if(state == Qt::Checked)
		return true;
	return false;
}


////////////////////////// General Tab ///////////////////////////////

GeneralTab::GeneralTab(struct RobotConfig *robot_config, QWidget *parent)
	: QWidget(parent)
{
	QLabel *robot_name_label = new QLabel(tr("Robot's Name"));
	robot_name_lineedit = new QLineEdit(robot_config->robotName);

	QLabel *system_label = new QLabel(tr("System"));
	QStringList system_list;
	system_list << "UGV"// (Unmanned Ground Vehicle)"
		<< "UAV"// (Unmanned Aerial Vehicle)"
		<< "AUV"// (Autonomous Underwater Vehicle)"
		<< "USV"// (Unmanned Surface Vehicle)"
		<< "Humanoid";
	system_combobox = new QComboBox;
	system_combobox->addItems(system_list);

	int index = system_list.indexOf(robot_config->system);
	if(index != -1)
		system_combobox->setCurrentIndex(index);

	QLabel *drive_system_label = new QLabel(tr("Drive System"));
	drive_system_lineedit = new QLineEdit(robot_config->driveSystem);

	QLabel *image_file_label = new QLabel(tr("Image File"));
	image_file_lineedit = new QLineEdit(robot_config->imageFilePath);

	QPushButton *browse_button = new QPushButton(tr("Browse"));
	connect(browse_button, SIGNAL(clicked()), this, SLOT(findImageFile()));

	QLabel *namespace_label = new QLabel(tr("Namespace"));
	namespace_lineedit = new QLineEdit(robot_config->nameSpace);

	// General tab layout
	QHBoxLayout *image_file_hlayout = new QHBoxLayout;
	image_file_hlayout->addWidget(image_file_lineedit);
	image_file_hlayout->addWidget(browse_button);

	QGridLayout *general_tab_layout = new QGridLayout;
	general_tab_layout->addWidget(robot_name_label, 0, 0);
	general_tab_layout->addWidget(robot_name_lineedit, 0, 1);
	general_tab_layout->addWidget(system_label, 1, 0);
	general_tab_layout->addWidget(system_combobox, 1, 1);
	general_tab_layout->addWidget(drive_system_label, 2, 0);
	general_tab_layout->addWidget(drive_system_lineedit, 2, 1);
	general_tab_layout->addWidget(image_file_label, 3, 0);
	general_tab_layout->addLayout(image_file_hlayout, 3, 1);
	general_tab_layout->addWidget(namespace_label, 4, 0);
	general_tab_layout->addWidget(namespace_lineedit, 4, 1);
	setLayout(general_tab_layout);
}

void GeneralTab::findImageFile()
{
	QString path;
	if(!image_file_lineedit->text().isEmpty())
	{
		QFileInfo file(image_file_lineedit->text());
		if(file.exists())
			path = file.absoluteFilePath();
		else
			path = QDir::homePath();
	}
	else
		path = QDir::homePath();

	QString file_name = QFileDialog::getOpenFileName(this, tr("Find Image"),
		path, tr("Images (*.bmp *.gif *.jpg *.jpeg *.png *.pbm *.pgm *.ppm *.tiff *.xbm *.xpm)"));

	image_file_lineedit->setText(file_name);
}

void GeneralTab::storeToConfig(struct RobotConfig *robot_config)
{
	std::cout << "Storing general info to robot configuration struct" << std::endl;

	robot_config->robotName = robot_name_lineedit->text();
	robot_config->system = system_combobox->currentText();
	robot_config->driveSystem = drive_system_lineedit->text();
	robot_config->imageFilePath = image_file_lineedit->text();
	robot_config->nameSpace = namespace_lineedit->text();
}

////////////////////////////// Sensors Tab ///////////////////////////////

SensorsTab::SensorsTab(struct RobotSensors *robot_sensors, QWidget *parent)
    : QWidget(parent)
{
    QLabel *sensors_label = new QLabel(tr("Sensors"));
    QStringList sensor_list;
    sensor_list << "Camera (Image.msg)"
                << "Compass (Imu.msg)"
                << "GPS (NavSatFix.msg)"
                << "IMU (Imu.msg)"
                << "Laser Rangefinder (LaserScan.msg)"
                << "Sonar/1D-Infrared (Range.msg)";
    sensors_combobox = new QComboBox;
    sensors_combobox->addItems(sensor_list);

    QPushButton *add_sensor_button = new QPushButton(tr("Add"));
    connect(add_sensor_button, SIGNAL(clicked()), this, SLOT(addSensor()));

    QPushButton *edit_button = new QPushButton(tr("Edit"));
    connect(edit_button, SIGNAL(clicked()), this, SLOT(editSensor()));

    QPushButton *remove_button = new QPushButton(tr("Remove"));
    connect(remove_button, SIGNAL(clicked()), this, SLOT(removeSensor()));

    QList<QTreeWidgetItem *> item_list;
    QTreeWidgetItem *child_item;

    /* Add Camera config data to item list */
    for(unsigned int i = 0; i < robot_sensors->cameras.size(); i++)
    {
        QTreeWidgetItem *camera_item = new QTreeWidgetItem(Camera);
        camera_item->setText(0, tr("Camera"));
        camera_item->setText(1, robot_sensors->cameras[i].name);

        camera_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Name") << robot_sensors->cameras[i].name)));
        camera_item->addChild(new QTreeWidgetItem((QTreeWidget *)0, 
            (QStringList() << tr("Topic Name") << robot_sensors->cameras[i].topicName)));

        item_list.append(camera_item);
    }

    /* @todo Add compass' */


    /* Add GPS's config data to item list */
    for(unsigned int i = 0; i < robot_sensors->gps.size(); i++)
    {
        QTreeWidgetItem *gps_item = new QTreeWidgetItem(Gps);
        gps_item->setText(0, tr("GPS"));
        gps_item->setText(1, robot_sensors->gps[i].name);

        gps_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Name") << robot_sensors->gps[i].name)));
        gps_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Topic Name") << robot_sensors->gps[i].topicName)));

        child_item = new QTreeWidgetItem(QStringList(tr("Latitude")));
        child_item->setCheckState(1, boolToCheckState(robot_sensors->gps[i].latitude));
        gps_item->addChild(child_item);

        child_item = new QTreeWidgetItem(QStringList(tr("Longitude")));
        child_item->setCheckState(1, boolToCheckState(robot_sensors->gps[i].longitude));
        gps_item->addChild(child_item);

        child_item = new QTreeWidgetItem(QStringList(tr("Altitude")));
        child_item->setCheckState(1, boolToCheckState(robot_sensors->gps[i].altitude));
        gps_item->addChild(child_item);

        item_list.append(gps_item);
    }

    /* Add IMU config data to item list */
    for(unsigned int i = 0; i < robot_sensors->imu.size(); i++)
    {
        QTreeWidgetItem *imu_item = new QTreeWidgetItem(Imu);
        imu_item->setText(0, tr("IMU"));
        imu_item->setText(1, robot_sensors->imu[i].name);

        imu_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Name") << robot_sensors->imu[i].name)));
        imu_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Topic Name") << robot_sensors->imu[i].topicName)));
        
        child_item = new QTreeWidgetItem(QStringList(tr("Roll")));
        child_item->setCheckState(1, boolToCheckState(robot_sensors->imu[i].roll));
        imu_item->addChild(child_item);

        child_item = new QTreeWidgetItem(QStringList(tr("Pitch")));
        child_item->setCheckState(1, boolToCheckState(robot_sensors->imu[i].pitch));
        imu_item->addChild(child_item);

        child_item = new QTreeWidgetItem(QStringList(tr("Yaw")));
        child_item->setCheckState(1, boolToCheckState(robot_sensors->imu[i].yaw));
        imu_item->addChild(child_item);

        child_item = new QTreeWidgetItem(QStringList(tr("Angular Velocity")));
        child_item->setCheckState(1, boolToCheckState(robot_sensors->imu[i].angularVelocity));
        imu_item->addChild(child_item);

        child_item = new QTreeWidgetItem(QStringList(tr("Linear Acceleration")));
        child_item->setCheckState(1, boolToCheckState(robot_sensors->imu[i].linearAcceleration));
        imu_item->addChild(child_item);

        child_item = new QTreeWidgetItem(QStringList(tr("Show Attitude Indicator")));
        child_item->setCheckState(1, boolToCheckState(!robot_sensors->imu[i].hideAttitude));
        imu_item->addChild(child_item);

        child_item = new QTreeWidgetItem(QStringList(tr("Show Heading Indicator")));
        child_item->setCheckState(1, boolToCheckState(!robot_sensors->imu[i].hideHeading));
        imu_item->addChild(child_item);

        child_item = new QTreeWidgetItem(QStringList(tr("Show Labels")));
        child_item->setCheckState(1, boolToCheckState(!robot_sensors->imu[i].hideLabels));
        imu_item->addChild(child_item);

        item_list.append(imu_item);
    }

    /* Add Laser config data to item list */
    for(unsigned int i = 0; i < robot_sensors->lasers.size(); i++)
    {
        QTreeWidgetItem *laser_item = new QTreeWidgetItem(Laser);
        laser_item->setText(0, tr("Laser Rangefinder"));
        laser_item->setText(1, robot_sensors->lasers[i].name);

        laser_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Name") << robot_sensors->lasers[i].name)));
        laser_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Topic Name") << robot_sensors->lasers[i].topicName)));

        item_list.append(laser_item);
    }

    /* Add Range config data to item list */
    for(unsigned int i = 0; i < robot_sensors->range.size(); i++)
    {
        QTreeWidgetItem *range_item = new QTreeWidgetItem(Range);
        range_item->setText(0, tr("Sonar/1D-Infrared"));
        range_item->setText(1, robot_sensors->range[i].name);

        range_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Name") << robot_sensors->range[i].name)));
        range_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Topic Name") << robot_sensors->range[i].topicName)));

        item_list.append(range_item);
    }

    QStringList column_list;
    column_list << "Sensors" << "Value";

    /* Create Tree Widget */
    sensors_treewidget = new QTreeWidget;
    sensors_treewidget->setColumnCount(2);
    sensors_treewidget->setHeaderLabels(column_list);
    sensors_treewidget->addTopLevelItems(item_list);
    sensors_treewidget->resizeColumnToContents(0);
    connect(sensors_treewidget, SIGNAL(itemDoubleClicked(QTreeWidgetItem *, int)),
        this, SLOT(editSensor(QTreeWidgetItem *)));

    /* Create layout */
    QHBoxLayout *sensors_hlayout = new QHBoxLayout;
    sensors_hlayout->addWidget(sensors_label, 0, Qt::AlignLeft);
    sensors_hlayout->addStretch();
    sensors_hlayout->addWidget(sensors_combobox, 0, Qt::AlignRight);
    sensors_hlayout->addWidget(add_sensor_button, 0, Qt::AlignRight);

    QHBoxLayout *button_hlayout = new QHBoxLayout;
    button_hlayout->addWidget(remove_button, 0, Qt::AlignLeft);
    button_hlayout->addStretch();
    button_hlayout->addWidget(edit_button, 0, Qt::AlignRight);

    QVBoxLayout *sensors_tab_layout = new QVBoxLayout;
    sensors_tab_layout->addLayout(sensors_hlayout);
    sensors_tab_layout->addWidget(sensors_treewidget);
    sensors_tab_layout->addLayout(button_hlayout);
    setLayout(sensors_tab_layout);
}

void SensorsTab::addSensor()
{
    QTreeWidgetItem *item, *child_item;
    // Get selected sensor from combobox
    SensorType type = SensorType(sensors_combobox->currentIndex() + Camera);

    QString type_str;
    if(type == Camera)
        type_str = "Camera";
    else if(type == Compass)
        type_str = "Compass";
    else if(type == Gps)
        type_str = "GPS";
    else if(type == Imu)
        type_str = "IMU";
    else if(type == Laser)
        type_str = "Laser Rangefinder";
    else if(type == Range)
        type_str = "Sonar/1D-Infrared";
    else
    {
        std::cerr << "ERROR -- Unknown SensorType '" << type << "' encountered"
                  << " while adding sensor to configuration file" << std::endl;
        return;
    }

    if(type == Camera || type == Laser || type == Range) // Others require different dialog
    {
        ComponentDialog component_dialog(this);
        component_dialog.setWindowTitle(QString("Add %1").arg(type_str));

        if(component_dialog.exec())
        {
            item = new QTreeWidgetItem(type);
            item->setText(0, type_str);
            item->setText(1, component_dialog.getName());

            item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
                (QStringList() << tr("Name") << component_dialog.getName())));
            item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
                (QStringList() << tr("Topic Name") << component_dialog.getTopicName())));

            sensors_treewidget->addTopLevelItem(item);
        }
    }
    else if(type == Compass)
    {
        CompassDialog compass_dialog(this);
        compass_dialog.setWindowTitle(QString("Add %1").arg(type_str));
        
        if(compass_dialog.exec())
        {
            item = new QTreeWidgetItem(type);
            item->setText(0, type_str);
            item->setText(1, compass_dialog.getName());

            item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
                (QStringList() << tr("Name") << compass_dialog.getName())));
            item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
                (QStringList() << tr("Topic Name") << compass_dialog.getTopicName())));

            child_item = new QTreeWidgetItem(QStringList(tr("Show Heading Indicator")));
            child_item->setCheckState(1, boolToCheckState(compass_dialog.isShowHeadingChecked()));
            item->addChild(child_item);
            
            child_item = new QTreeWidgetItem(QStringList(tr("Show Heading Indiciator")));
            child_item->setCheckState(1, boolToCheckState(compass_dialog.isShowHeadingChecked()));
            item->addChild(child_item);

            child_item = new QTreeWidgetItem(QStringList(tr("Show Label")));
            child_item->setCheckState(1, boolToCheckState(compass_dialog.isShowLabelChecked()));
            item->addChild(child_item);

            sensors_treewidget->addTopLevelItem(item);
        }
    }
    else if(type == Gps)
    {
        GpsDialog gps_dialog(this);
        gps_dialog.setWindowTitle(QString("Add %1").arg(type_str));

        if(gps_dialog.exec())
        {
            item = new QTreeWidgetItem(type);
            item->setText(0, type_str);
            item->setText(1, gps_dialog.getName());

            item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
                (QStringList() << tr("Name") << gps_dialog.getName())));
            item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
                (QStringList() << tr("Topic Name") << gps_dialog.getTopicName())));

            // Add latitude and its check state
            child_item = new QTreeWidgetItem(QStringList(tr("Latitude")));
            child_item->setCheckState(1, boolToCheckState(gps_dialog.isLatitudeChecked()));
            item->addChild(child_item);

            // Add longitude and its check state
            child_item = new QTreeWidgetItem(QStringList(tr("Longitude")));
            child_item->setCheckState(1, boolToCheckState(gps_dialog.isLongitudeChecked()));
            item->addChild(child_item);

            // Add altitude and its check state
            child_item = new QTreeWidgetItem(QStringList(tr("Altitude")));
            child_item->setCheckState(1, boolToCheckState(gps_dialog.isAltitudeChecked()));
            item->addChild(child_item);

            sensors_treewidget->addTopLevelItem(item);
        }
    }
    else if(type == Imu)
    {
        ImuDialog imu_dialog(this);
        imu_dialog.setWindowTitle(QString("Add %1").arg(type_str));

        if(imu_dialog.exec())
        {
            item = new QTreeWidgetItem(type);
            item->setText(0, type_str);
            item->setText(1, imu_dialog.getName());

            item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
                (QStringList() << tr("Name") << imu_dialog.getName())));
            item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
                (QStringList() << tr("Topic Name") << imu_dialog.getTopicName())));

            child_item = new QTreeWidgetItem(QStringList(tr("Roll")));
            child_item->setCheckState(1, boolToCheckState(imu_dialog.isRollChecked()));
            item->addChild(child_item);

            child_item = new QTreeWidgetItem(QStringList(tr("Pitch")));
            child_item->setCheckState(1, boolToCheckState(imu_dialog.isPitchChecked()));
            item->addChild(child_item);

            child_item = new QTreeWidgetItem(QStringList(tr("Yaw")));
            child_item->setCheckState(1, boolToCheckState(imu_dialog.isYawChecked()));
            item->addChild(child_item);

            child_item = new QTreeWidgetItem(QStringList(tr("Angular Velocity")));
            child_item->setCheckState(1, boolToCheckState(imu_dialog.isAngularVelocityChecked()));
            item->addChild(child_item);

            child_item = new QTreeWidgetItem(QStringList(tr("Linear Acceleration")));
            child_item->setCheckState(1, boolToCheckState(imu_dialog.isLinearAccelerationChecked()));
            item->addChild(child_item);

            child_item = new QTreeWidgetItem(QStringList(tr("Show Attitude Indicator")));
            child_item->setCheckState(1, boolToCheckState(imu_dialog.isShowAttitudeChecked()));
            item->addChild(child_item);

            child_item = new QTreeWidgetItem(QStringList(tr("Show Heading Indicator")));
            child_item->setCheckState(1, boolToCheckState(imu_dialog.isShowHeadingChecked()));
            item->addChild(child_item);

            child_item = new QTreeWidgetItem(QStringList(tr("Show Labels")));
            child_item->setCheckState(1, boolToCheckState(imu_dialog.isShowLabelsChecked()));
            item->addChild(child_item);

            sensors_treewidget->addTopLevelItem(item);
        }
    }
}

void SensorsTab::editSensor(QTreeWidgetItem *item)
{
    QTreeWidgetItem *top_item = item;

    if(item == 0) // Item is unknown so grab current item
    {
        top_item = sensors_treewidget->currentItem();

        if(top_item == 0) // No item is selected
            return;
    }
    else if(sensors_treewidget->indexOfTopLevelItem(item) != -1) // Ignore double click on a parent item
        return;

    // Get parent item
    if(top_item->parent() != 0)
        top_item = top_item->parent();

    // Edit Camera, Laser, or Range item
    if(top_item->type() == Camera || top_item->type() == Laser || top_item->type() == Range)
    {
        // Create dialog and populate with item's values
        ComponentDialog component_dialog(this);
        component_dialog.setWindowTitle(QString("Edit %1").arg(top_item->text(0)));
        component_dialog.setName(top_item->child(0)->text(1));
        component_dialog.setTopicName(top_item->child(1)->text(1));

        // Execute dialog and update item
        if(component_dialog.exec())
        {
            top_item->setText(1, component_dialog.getName());
            top_item->child(0)->setText(1, component_dialog.getName());
            top_item->child(1)->setText(1, component_dialog.getTopicName());
        }
    }
    else if(top_item->type() == Compass) // Edit compass item
    {
        // Create compass dialog and populate with item's values
        CompassDialog compass_dialog(this);
        compass_dialog.setWindowTitle(QString("Edit %1").arg(top_item->text(0)));
        compass_dialog.setName(top_item->child(0)->text(1));
        compass_dialog.setTopicName(top_item->child(1)->text(1));
        compass_dialog.setShowHeadingChecked(checkStateToBool(top_item->child(2)->checkState(1)));
        compass_dialog.setShowLabelChecked(checkStateToBool(top_item->child(3)->checkState(1)));

        // Execute dialog and update item
        if(compass_dialog.exec())
        {
            top_item->setText(1, compass_dialog.getName());
            top_item->child(0)->setText(1, compass_dialog.getName());
            top_item->child(1)->setText(1, compass_dialog.getTopicName());
            top_item->child(2)->setCheckState(1, boolToCheckState(compass_dialog.isShowHeadingChecked()));
            top_item->child(3)->setCheckState(1, boolToCheckState(compass_dialog.isShowLabelChecked()));
        }
    }
    else if(top_item->type() == Gps) // Edit GPS item
    {
        // Create gps dialog and populate with item's values
        GpsDialog gps_dialog(this);
        gps_dialog.setWindowTitle(QString("Edit %1").arg(top_item->text(0)));
        gps_dialog.setName(top_item->child(0)->text(1));
        gps_dialog.setTopicName(top_item->child(1)->text(1));
        gps_dialog.setLatitudeChecked(checkStateToBool(top_item->child(2)->checkState(1)));
        gps_dialog.setLongitudeChecked(checkStateToBool(top_item->child(3)->checkState(1)));
        gps_dialog.setAltitudeChecked(checkStateToBool(top_item->child(4)->checkState(1)));

        // Execute dialog and update item
        if(gps_dialog.exec())
        {
            top_item->setText(1, gps_dialog.getName());
            top_item->child(0)->setText(1, gps_dialog.getName());
            top_item->child(1)->setText(1, gps_dialog.getTopicName());
            top_item->child(2)->setCheckState(1, boolToCheckState(gps_dialog.isLatitudeChecked()));
            top_item->child(3)->setCheckState(1, boolToCheckState(gps_dialog.isLongitudeChecked()));
            top_item->child(4)->setCheckState(1, boolToCheckState(gps_dialog.isAltitudeChecked()));
        }
    }
    else if(top_item->type() == Imu) // Edit IMU item
    {
        // Create IMU dialog and populate with item's values
        ImuDialog imu_dialog(this);
        imu_dialog.setWindowTitle(QString("Edit %1").arg(top_item->text(0)));
        imu_dialog.setName(top_item->child(0)->text(1));
        imu_dialog.setTopicName(top_item->child(1)->text(1));
        imu_dialog.setRollChecked(checkStateToBool(top_item->child(2)->checkState(1)));
        imu_dialog.setPitchChecked(checkStateToBool(top_item->child(3)->checkState(1)));
        imu_dialog.setYawChecked(checkStateToBool(top_item->child(4)->checkState(1)));
        imu_dialog.setAngularVelocityChecked(checkStateToBool(top_item->child(5)->checkState(1)));
        imu_dialog.setLinearAccelerationChecked(checkStateToBool(top_item->child(6)->checkState(1)));
        imu_dialog.setShowAttitudeChecked(checkStateToBool(top_item->child(7)->checkState(1)));
        imu_dialog.setShowHeadingChecked(checkStateToBool(top_item->child(8)->checkState(1)));
        imu_dialog.setShowLabelsChecked(checkStateToBool(top_item->child(9)->checkState(1)));

        // Execute dialog and update item
        if(imu_dialog.exec())
        {
            top_item->setText(1, imu_dialog.getName());
            top_item->child(0)->setText(1, imu_dialog.getName());
            top_item->child(1)->setText(1, imu_dialog.getTopicName());
            top_item->child(2)->setCheckState(1, boolToCheckState(imu_dialog.isRollChecked()));
            top_item->child(3)->setCheckState(1, boolToCheckState(imu_dialog.isPitchChecked()));
            top_item->child(4)->setCheckState(1, boolToCheckState(imu_dialog.isYawChecked()));
            top_item->child(5)->setCheckState(1, boolToCheckState(imu_dialog.isAngularVelocityChecked()));
            top_item->child(6)->setCheckState(1, boolToCheckState(imu_dialog.isLinearAccelerationChecked()));
            top_item->child(7)->setCheckState(1, boolToCheckState(imu_dialog.isShowAttitudeChecked()));
            top_item->child(8)->setCheckState(1, boolToCheckState(imu_dialog.isShowHeadingChecked()));
            top_item->child(9)->setCheckState(1, boolToCheckState(imu_dialog.isShowLabelsChecked()));
        }
    }
    else
    {
        std::cerr << "Unknown sensor type '" << top_item->type()
                  << "' encountered while editing sensor." << std::endl;
    }
}

void SensorsTab::removeSensor()
{
	// Get current item
	QTreeWidgetItem *item = sensors_treewidget->currentItem();

	if(item == 0) // No item is selected
		return;

	if(item->parent() != 0) // Get top level item in the tree widget
		item = item->parent();

	delete item;
}

void SensorsTab::storeToConfig(struct RobotSensors *robot_sensors)
{
    std::cout << "Storing sensors to robot configuration struct" << std::endl;

    QTreeWidgetItem *item;
    /* Loop through all top level items */
    for(int i = 0; i < sensors_treewidget->topLevelItemCount(); i++)
    {
        item = sensors_treewidget->topLevelItem(i);

        if(item->type() == Camera)
        {
            struct RobotCamera temp_camera;
            temp_camera.name = item->child(0)->text(1);
            temp_camera.topicName = item->child(1)->text(1);

            robot_sensors->cameras.push_back(temp_camera);
        }
        else if(item->type() == Compass)
        {
            /* @todo Store compass to configuration file */
        }
        else if(item->type() == Gps)
        {
            struct RobotGPS temp_gps;
            temp_gps.name = item->child(0)->text(1);
            temp_gps.topicName = item->child(1)->text(1);
            temp_gps.latitude = checkStateToBool(item->child(2)->checkState(1));
            temp_gps.longitude = checkStateToBool(item->child(3)->checkState(1));
            temp_gps.altitude = checkStateToBool(item->child(4)->checkState(1));

            robot_sensors->gps.push_back(temp_gps);
        }
        else if(item->type() == Imu)
        {
            struct RobotIMU temp_imu;
            temp_imu.name = item->child(0)->text(1);
            temp_imu.topicName = item->child(1)->text(1);
            temp_imu.roll = checkStateToBool(item->child(2)->checkState(1));
            temp_imu.pitch = checkStateToBool(item->child(3)->checkState(1));
            temp_imu.yaw = checkStateToBool(item->child(4)->checkState(1));
            temp_imu.angularVelocity = checkStateToBool(item->child(5)->checkState(1));
            temp_imu.linearAcceleration = checkStateToBool(item->child(6)->checkState(1));
            temp_imu.hideAttitude = !checkStateToBool(item->child(7)->checkState(1));
            temp_imu.hideHeading = !checkStateToBool(item->child(8)->checkState(1));
            temp_imu.hideLabels = !checkStateToBool(item->child(9)->checkState(1));

            robot_sensors->imu.push_back(temp_imu);
        }
        else if(item->type() == Laser)
        {
            struct RobotLaser temp_laser;
            temp_laser.name = item->child(0)->text(1);
            temp_laser.topicName = item->child(1)->text(1);

            robot_sensors->lasers.push_back(temp_laser);
        }
        else if(item->type() == Range)
        {
            struct RobotRange temp_range;
            temp_range.name = item->child(0)->text(1);
            temp_range.topicName = item->child(1)->text(1);

            robot_sensors->range.push_back(temp_range);
        }
        else
        {
            std::cerr << "ERROR -- Unknown sensor type '" << item->type()
                      << "' encountered when storing sensors to config file." << std::endl;
        }
    }
}

////////////////////////////// Processed Data Tab //////////////////////////

ProcessedDataTab::ProcessedDataTab(struct RobotProcessedData *robot_processed_data,
    QWidget *parent) : QWidget(parent)
{
    QLabel *processed_data_label = new QLabel(tr("Processed Data"));
    QStringList processed_data_list;
    processed_data_list << "Disparity Image (DisparityImage.msg)"
                        << "Map (Map.msg)"
                        << "Odometry (Odometry.msg)"
                        << "Pose (Pose.msg)"
                        << "Processed Image (Image.msg)";
    processed_data_combobox = new QComboBox;
    processed_data_combobox->addItems(processed_data_list);

    QPushButton *add_button = new QPushButton(tr("Add"));
    connect(add_button, SIGNAL(clicked()), this, SLOT(addProcessedData()));

    QPushButton *edit_button = new QPushButton(tr("Edit"));
    connect(edit_button, SIGNAL(clicked()), this, SLOT(editProcessedData()));

    QPushButton *remove_button = new QPushButton(tr("Remove"));
    connect(remove_button, SIGNAL(clicked()), this, SLOT(removeProcessedData()));

    QList<QTreeWidgetItem *> item_list;
    QTreeWidgetItem *child_item;

    /* Add disparity images from robot configuration file */
    for(unsigned int i = 0; i < robot_processed_data->disparity_images.size(); i++)
    {
        QTreeWidgetItem *disparity_image_item = new QTreeWidgetItem(DisparityImage);
        disparity_image_item->setText(0, tr("Disparity Image"));
        disparity_image_item->setText(1, robot_processed_data->disparity_images[i].name);

        disparity_image_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Name") << robot_processed_data->disparity_images[i].name)));
        disparity_image_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Topic Name") << robot_processed_data->disparity_images[i].topicName)));

        item_list.append(disparity_image_item);
    }

    /* Add odometry from robot configuration file */
    for(unsigned int i = 0; i < robot_processed_data->odometry.size(); i++)
    {
        QTreeWidgetItem *odometry_item = new QTreeWidgetItem(Odometry);
        odometry_item->setText(0, tr("Odometry"));
        odometry_item->setText(1, robot_processed_data->odometry[i].name);

        odometry_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Name") << robot_processed_data->odometry[i].name)));
        odometry_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Topic Name") << robot_processed_data->odometry[i].topicName)));

        child_item = new QTreeWidgetItem(QStringList(tr("Position")));
        child_item->setCheckState(1, boolToCheckState(robot_processed_data->odometry[i].position));
        odometry_item->addChild(child_item);

        child_item = new QTreeWidgetItem(QStringList(tr("Orientation")));
        child_item->setCheckState(1, boolToCheckState(robot_processed_data->odometry[i].orientation));
        odometry_item->addChild(child_item);

        child_item = new QTreeWidgetItem(QStringList(tr("Linear Velocity")));
        child_item->setCheckState(1, boolToCheckState(robot_processed_data->odometry[i].linearVelocity));
        odometry_item->addChild(child_item);

        child_item = new QTreeWidgetItem(QStringList(tr("Angular Velocity")));
        child_item->setCheckState(1, boolToCheckState(robot_processed_data->odometry[i].angularVelocity));
        odometry_item->addChild(child_item);

        child_item = new QTreeWidgetItem(QStringList(tr("Show AttitudeIndicator")));
        child_item->setCheckState(1, boolToCheckState(!robot_processed_data->odometry[i].hideAttitude));
        odometry_item->addChild(child_item);

        child_item = new QTreeWidgetItem(QStringList(tr("Show Heading Indicator")));
        child_item->setCheckState(1, boolToCheckState(!robot_processed_data->odometry[i].hideHeading));
        odometry_item->addChild(child_item);

        child_item = new QTreeWidgetItem(QStringList(tr("Show Labels")));
        child_item->setCheckState(1, boolToCheckState(!robot_processed_data->odometry[i].hideLabels));
        odometry_item->addChild(child_item);

        child_item = new QTreeWidgetItem(QStringList(tr("Update Map")));
        child_item->setCheckState(1, boolToCheckState(robot_processed_data->odometry[i].updateMap));
        odometry_item->addChild(child_item);

        item_list.append(odometry_item);
    }

    /* Add pose from robot configuration file */
    for(unsigned int i = 0; i < robot_processed_data->pose.size(); i++)
    {
        QTreeWidgetItem *pose_item = new QTreeWidgetItem(Pose);
        pose_item->setText(0, tr("Pose"));
        pose_item->setText(1, robot_processed_data->pose[i].name);

        pose_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Name") << robot_processed_data->pose[i].name)));
        pose_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Topic Name") << robot_processed_data->pose[i].topicName)));

        child_item = new QTreeWidgetItem(QStringList(tr("Position")));
        child_item->setCheckState(1, boolToCheckState(robot_processed_data->pose[i].position));
        pose_item->addChild(child_item);

        child_item = new QTreeWidgetItem(QStringList(tr("Orientation")));
        child_item->setCheckState(1, boolToCheckState(robot_processed_data->pose[i].orientation));
        pose_item->addChild(child_item);

        child_item = new QTreeWidgetItem(QStringList(tr("Show AttitudeIndicator")));
        child_item->setCheckState(1, boolToCheckState(!robot_processed_data->pose[i].hideAttitude));
        pose_item->addChild(child_item);

        child_item = new QTreeWidgetItem(QStringList(tr("Show Heading Indicator")));
        child_item->setCheckState(1, boolToCheckState(!robot_processed_data->pose[i].hideHeading));
        pose_item->addChild(child_item);

        child_item = new QTreeWidgetItem(QStringList(tr("Show Labels")));
        child_item->setCheckState(1, boolToCheckState(!robot_processed_data->pose[i].hideLabels));
        pose_item->addChild(child_item);

        child_item = new QTreeWidgetItem(QStringList(tr("Pose Is Stamped")));
        child_item->setCheckState(1, boolToCheckState(robot_processed_data->pose[i].isStamped || robot_processed_data->pose[i].hasCovariance));
        pose_item->addChild(child_item);

        child_item = new QTreeWidgetItem(QStringList(tr("Pose Has Covariance")));
        child_item->setCheckState(1, boolToCheckState(robot_processed_data->pose[i].hasCovariance));
        pose_item->addChild(child_item);

        child_item = new QTreeWidgetItem(QStringList(tr("Update Map")));
        child_item->setCheckState(1, boolToCheckState(robot_processed_data->pose[i].updateMap));
        pose_item->addChild(child_item);

        item_list.append(pose_item);
    }

    /* Add maps from robot configuration file */
    for(unsigned int i = 0; i < robot_processed_data->maps.size(); i++)
    {
        QTreeWidgetItem *map_item = new QTreeWidgetItem(Map);
        map_item->setText(0, tr("Map"));
        map_item->setText(1, robot_processed_data->maps[i].name);

        map_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Name") << robot_processed_data->maps[i].name)));
        map_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Topic Name") << robot_processed_data->maps[i].topicName)));

        item_list.append(map_item);
    }

    /* Add processed images from robot configuation file */
    for(unsigned int i = 0; i < robot_processed_data->images.size(); i++)
    {
        QTreeWidgetItem *processed_image_item = new QTreeWidgetItem(ProcessedImage);
        processed_image_item->setText(0, tr("Processed Image"));
        processed_image_item->setText(1, robot_processed_data->images[i].name);

        processed_image_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Name") << robot_processed_data->images[i].name)));
        processed_image_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Topic Name") << robot_processed_data->images[i].topicName)));

        item_list.append(processed_image_item);
    }

    QStringList column_list;
    column_list << "Processed Data" << "Values";

    /* Create tree widget */
    processed_data_treewidget = new QTreeWidget;
    processed_data_treewidget->setHeaderLabels(column_list);
    processed_data_treewidget->addTopLevelItems(item_list);
    processed_data_treewidget->resizeColumnToContents(0);
    connect(processed_data_treewidget, SIGNAL(itemDoubleClicked(QTreeWidgetItem *, int)),
            this, SLOT(editProcessedData(QTreeWidgetItem *)));

    /* Create layout */
    QHBoxLayout *processed_data_hlayout = new QHBoxLayout;
    processed_data_hlayout->addWidget(processed_data_label, 0, Qt::AlignLeft);
    processed_data_hlayout->addStretch();
    processed_data_hlayout->addWidget(processed_data_combobox, 0, Qt::AlignRight);
    processed_data_hlayout->addWidget(add_button, 0, Qt::AlignRight);

    QHBoxLayout *button_hlayout = new QHBoxLayout;
    button_hlayout->addWidget(remove_button, 0, Qt::AlignLeft);
    button_hlayout->addStretch();
    button_hlayout->addWidget(edit_button, 0, Qt::AlignRight);

    QVBoxLayout *processed_data_layout = new QVBoxLayout;
    processed_data_layout->addLayout(processed_data_hlayout);
    processed_data_layout->addWidget(processed_data_treewidget);
    processed_data_layout->addLayout(button_hlayout);
    setLayout(processed_data_layout);
}

void ProcessedDataTab::addProcessedData()
{
    QTreeWidgetItem *item, *child_item;
    /* Get the type of component */
    ProcessedDataType add_type = ProcessedDataType(processed_data_combobox->currentIndex() + DisparityImage);
    QString type_str;

    if(add_type == DisparityImage)
        type_str = "Disparity Image";
    else if(add_type == Map)
        type_str = "Map";
    else if(add_type == Odometry)
        type_str = "Odometry";
    else if(add_type == Pose)
        type_str = "Pose";
    else if(add_type == ProcessedImage)
        type_str = "Processed Image";
    else
    {
        std::cerr << "ERROR -- Unknown Type encountered '" << add_type
                  << "' while adding processed data component" << std::endl;
        return;
    }

    if(add_type == Odometry) // Odometry needs a different dialog than ComponentDialog
    {
        OdometryDialog odom_dialog(this);
        odom_dialog.setWindowTitle(QString("Add %1").arg(type_str));

        if(odom_dialog.exec())
        {
            item = new QTreeWidgetItem(add_type);
            item->setText(0, type_str);
            item->setText(1, odom_dialog.getName());

            item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
                (QStringList() << tr("Name") << odom_dialog.getName())));
            item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
                (QStringList() << tr("Topic Name") << odom_dialog.getTopicName())));

            // Set and add position state
            child_item = new QTreeWidgetItem(QStringList(tr("Position")));
            child_item->setCheckState(1, boolToCheckState(odom_dialog.isPositionChecked()));
            item->addChild(child_item);

            // Set and add orientation state
            child_item = new QTreeWidgetItem(QStringList(tr("Orientation")));
            child_item->setCheckState(1, boolToCheckState(odom_dialog.isOrientationChecked()));
            item->addChild(child_item);

            // Set and add linear velocity state
            child_item = new QTreeWidgetItem(QStringList(tr("Linear Velocity")));
            child_item->setCheckState(1, boolToCheckState(odom_dialog.isLinearVelocityChecked()));
            item->addChild(child_item);

            // Set and add angular velocity state
            child_item = new QTreeWidgetItem(QStringList(tr("Angular Velocity")));
            child_item->setCheckState(1, boolToCheckState(odom_dialog.isAngularVelocityChecked()));
            item->addChild(child_item);

            // Set and add attitude indicator state
            child_item = new QTreeWidgetItem(QStringList(tr("Show Attitude Indicator")));
            child_item->setCheckState(1, boolToCheckState(odom_dialog.isShowAttitudeChecked()));
            item->addChild(child_item);

            // Set and add heading indicator state
            child_item = new QTreeWidgetItem(QStringList(tr("Show Heading Indicator")));
            child_item->setCheckState(1, boolToCheckState(odom_dialog.isShowHeadingChecked()));
            item->addChild(child_item);

            // Set and add labels state
            child_item = new QTreeWidgetItem(QStringList(tr("Show Labels")));
            child_item->setCheckState(1, boolToCheckState(odom_dialog.isShowLabelsChecked()));
            item->addChild(child_item);

            // Set and add update map state
            child_item = new QTreeWidgetItem(QStringList(tr("Update Map")));
            child_item->setCheckState(1, boolToCheckState(odom_dialog.isUpdateMapChecked()));
            item->addChild(child_item);

            processed_data_treewidget->addTopLevelItem(item);
        }
    }
    else if(add_type == Pose) // Pose needs a different dialog than ComponentDialog
    {
        PoseDialog pose_dialog(this);
        pose_dialog.setWindowTitle(QString("Add %1").arg(type_str));

        if(pose_dialog.exec())
        {
            item = new QTreeWidgetItem(add_type);
            item->setText(0, type_str);
            item->setText(1, pose_dialog.getName());

            item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
                (QStringList() << tr("Name") << pose_dialog.getName())));
            item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
                (QStringList() << tr("Topic Name") << pose_dialog.getTopicName())));

            // Set and add position state
            child_item = new QTreeWidgetItem(QStringList(tr("Position")));
            child_item->setCheckState(1, boolToCheckState(pose_dialog.isPositionChecked()));
            item->addChild(child_item);

            // Set and add orientation state
            child_item = new QTreeWidgetItem(QStringList(tr("Orientation")));
            child_item->setCheckState(1, boolToCheckState(pose_dialog.isOrientationChecked()));
            item->addChild(child_item);

            // Set and add attitude indicator state
            child_item = new QTreeWidgetItem(QStringList(tr("Show Attitude Indicator")));
            child_item->setCheckState(1, boolToCheckState(pose_dialog.isShowAttitudeChecked()));
            item->addChild(child_item);

            // Set and add heading indicator state
            child_item = new QTreeWidgetItem(QStringList(tr("Show Heading Indicator")));
            child_item->setCheckState(1, boolToCheckState(pose_dialog.isShowHeadingChecked()));
            item->addChild(child_item);

            // Set and add labels state
            child_item = new QTreeWidgetItem(QStringList(tr("Show Labels")));
            child_item->setCheckState(1, boolToCheckState(pose_dialog.isShowLabelsChecked()));
            item->addChild(child_item);

            // Set and add stamped state
            child_item = new QTreeWidgetItem(QStringList(tr("Pose Is Stamped")));
            child_item->setCheckState(1, boolToCheckState(pose_dialog.isIsStampedChecked() || pose_dialog.isHasCovarianceChecked()));
            item->addChild(child_item);

            // Set and add covariance state
            child_item = new QTreeWidgetItem(QStringList(tr("Pose Has Covariance")));
            child_item->setCheckState(1, boolToCheckState(pose_dialog.isHasCovarianceChecked()));
            item->addChild(child_item);

            // Set and add update map state
            child_item = new QTreeWidgetItem(QStringList(tr("Update Map")));
            child_item->setCheckState(1, boolToCheckState(pose_dialog.isUpdateMapChecked()));
            item->addChild(child_item);

            processed_data_treewidget->addTopLevelItem(item);
        }
    }
    else
    {
        ComponentDialog component_dialog(this);
        component_dialog.setWindowTitle(QString("Add %1").arg(type_str));

        if(component_dialog.exec())
        {
            item = new QTreeWidgetItem(add_type);
            item->setText(0, type_str);
            item->setText(1, component_dialog.getName());

            item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
                (QStringList() << tr("Name") << component_dialog.getName())));
            item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
                (QStringList() << tr("Topic Name") << component_dialog.getTopicName())));

            processed_data_treewidget->addTopLevelItem(item);
        }
    }
}

void ProcessedDataTab::editProcessedData(QTreeWidgetItem *item)
{
    QTreeWidgetItem *top_item = item;

    if(item == 0) // Item is unknown so grab current item
    {
        top_item = processed_data_treewidget->currentItem();

        if(top_item == 0) // No item is selected
            return;
    }
    else if(processed_data_treewidget->indexOfTopLevelItem(item) != -1)
        return;

    // Get parent item
    if(top_item->parent() != 0)
        top_item = top_item->parent();

    if(top_item->type() == DisparityImage || top_item->type() == Map ||
       top_item->type() == ProcessedImage)
    {
        ComponentDialog component_dialog(this);
        component_dialog.setWindowTitle(QString("Edit %1").arg(top_item->text(0)));
        component_dialog.setName(top_item->child(0)->text(1));
        component_dialog.setTopicName(top_item->child(1)->text(1));
        
        if(component_dialog.exec())
        {
            top_item->setText(1, component_dialog.getName());
            top_item->child(0)->setText(1, component_dialog.getName());
            top_item->child(1)->setText(1, component_dialog.getTopicName());
        }
    }
    else if(top_item->type() == Odometry)
    {
        OdometryDialog odom_dialog(this);
        odom_dialog.setWindowTitle(QString("Edit %1").arg(top_item->type()));
        odom_dialog.setName(top_item->child(0)->text(1));
        odom_dialog.setTopicName(top_item->child(1)->text(1));
        odom_dialog.setPositionChecked(checkStateToBool(top_item->child(2)->checkState(1)));
        odom_dialog.setOrientationChecked(checkStateToBool(top_item->child(3)->checkState(1)));
        odom_dialog.setLinearVelocityChecked(checkStateToBool(top_item->child(4)->checkState(1)));
        odom_dialog.setAngularVelocityChecked(checkStateToBool(top_item->child(5)->checkState(1)));
        odom_dialog.setShowAttitudeChecked(checkStateToBool(top_item->child(6)->checkState(1)));
        odom_dialog.setShowHeadingChecked(checkStateToBool(top_item->child(7)->checkState(1)));
        odom_dialog.setShowLabelsChecked(checkStateToBool(top_item->child(8)->checkState(1)));
        odom_dialog.setUpdateMapChecked(checkStateToBool(top_item->child(9)->checkState(1)));

        if(odom_dialog.exec())
        {
            top_item->setText(1, odom_dialog.getName());
            top_item->child(0)->setText(1, odom_dialog.getName());
            top_item->child(1)->setText(1, odom_dialog.getTopicName());
            top_item->child(2)->setCheckState(1, boolToCheckState(odom_dialog.isPositionChecked()));
            top_item->child(3)->setCheckState(1, boolToCheckState(odom_dialog.isOrientationChecked()));
            top_item->child(4)->setCheckState(1, boolToCheckState(odom_dialog.isLinearVelocityChecked()));
            top_item->child(5)->setCheckState(1, boolToCheckState(odom_dialog.isAngularVelocityChecked()));
            top_item->child(6)->setCheckState(1, boolToCheckState(odom_dialog.isShowAttitudeChecked()));
            top_item->child(7)->setCheckState(1, boolToCheckState(odom_dialog.isShowHeadingChecked()));
            top_item->child(8)->setCheckState(1, boolToCheckState(odom_dialog.isShowLabelsChecked()));
            top_item->child(9)->setCheckState(1, boolToCheckState(odom_dialog.isUpdateMapChecked()));
        }
    }
    else if(top_item->type() == Pose)
    {
        PoseDialog pose_dialog(this);
        pose_dialog.setWindowTitle(QString("Edit %1").arg(top_item->type()));
        pose_dialog.setName(top_item->child(0)->text(1));
        pose_dialog.setTopicName(top_item->child(1)->text(1));
        pose_dialog.setPositionChecked(checkStateToBool(top_item->child(2)->checkState(1)));
        pose_dialog.setOrientationChecked(checkStateToBool(top_item->child(3)->checkState(1)));
        pose_dialog.setShowAttitudeChecked(checkStateToBool(top_item->child(4)->checkState(1)));
        pose_dialog.setShowHeadingChecked(checkStateToBool(top_item->child(5)->checkState(1)));
        pose_dialog.setShowLabelsChecked(checkStateToBool(top_item->child(6)->checkState(1)));
        pose_dialog.setIsStampedChecked(checkStateToBool(top_item->child(7)->checkState(1)));
        pose_dialog.setHasCovarianceChecked(checkStateToBool(top_item->child(8)->checkState(1)));
        pose_dialog.setUpdateMapChecked(checkStateToBool(top_item->child(9)->checkState(1)));

        if(pose_dialog.exec())
        {
            top_item->setText(1, pose_dialog.getName());
            top_item->child(0)->setText(1, pose_dialog.getName());
            top_item->child(1)->setText(1, pose_dialog.getTopicName());
            top_item->child(2)->setCheckState(1, boolToCheckState(pose_dialog.isPositionChecked()));
            top_item->child(3)->setCheckState(1, boolToCheckState(pose_dialog.isOrientationChecked()));
            top_item->child(4)->setCheckState(1, boolToCheckState(pose_dialog.isShowAttitudeChecked()));
            top_item->child(5)->setCheckState(1, boolToCheckState(pose_dialog.isShowHeadingChecked()));
            top_item->child(6)->setCheckState(1, boolToCheckState(pose_dialog.isShowLabelsChecked()));
            top_item->child(7)->setCheckState(1, boolToCheckState(pose_dialog.isIsStampedChecked()));
            top_item->child(8)->setCheckState(1, boolToCheckState(pose_dialog.isHasCovarianceChecked()));
            top_item->child(9)->setCheckState(1, boolToCheckState(pose_dialog.isUpdateMapChecked()));
        }
    }
    else
    {
        std::cerr << "Unknown processed data type '" << top_item->type()
                  << "' encountered while editing." << std::endl;
    }
}

void ProcessedDataTab::removeProcessedData()
{
	// Get current item
	QTreeWidgetItem *item = processed_data_treewidget->currentItem();

	if(item == 0) // No item is selected
		return;

	if(item->parent() != 0) // Get top level item in the tree widget
		item = item->parent();

	delete item;
}

void ProcessedDataTab::storeToConfig(struct RobotProcessedData *robot_processed_data)
{
    std::cout << "Storing processed data to robot configuration struct" << std::endl;

    QTreeWidgetItem *item;
    /* Loop through top level items and store to config struct */
    for(int i = 0; i < processed_data_treewidget->topLevelItemCount(); i++)
    {
        item = processed_data_treewidget->topLevelItem(i);

        if(item->type() == DisparityImage)
        {
            struct RobotDisparityImage temp_disp_image;
            temp_disp_image.name = item->child(0)->text(1);
            temp_disp_image.topicName = item->child(1)->text(1);

            robot_processed_data->disparity_images.push_back(temp_disp_image);
        }
        else if(item->type() == Map)
        {
            struct RobotMap temp_map;
            temp_map.name = item->child(0)->text(1);
            temp_map.topicName = item->child(1)->text(1);

            robot_processed_data->maps.push_back(temp_map);
        }
        else if(item->type() == Odometry)
        {
            struct RobotOdometry temp_odom;
            temp_odom.name = item->child(0)->text(1);
            temp_odom.topicName = item->child(1)->text(1);
            temp_odom.position = checkStateToBool(item->child(2)->checkState(1));
            temp_odom.orientation = checkStateToBool(item->child(3)->checkState(1));
            temp_odom.linearVelocity = checkStateToBool(item->child(4)->checkState(1));
            temp_odom.angularVelocity = checkStateToBool(item->child(5)->checkState(1));
            temp_odom.hideAttitude = !checkStateToBool(item->child(6)->checkState(1));
            temp_odom.hideHeading = !checkStateToBool(item->child(7)->checkState(1));
            temp_odom.hideLabels = !checkStateToBool(item->child(8)->checkState(1));
            temp_odom.updateMap = checkStateToBool(item->child(9)->checkState(1));

            robot_processed_data->odometry.push_back(temp_odom);
        }
        else if(item->type() == Pose)
        {
            struct RobotPose temp_pose;
            temp_pose.name = item->child(0)->text(1);
            temp_pose.topicName = item->child(1)->text(1);
            temp_pose.position = checkStateToBool(item->child(2)->checkState(1));
            temp_pose.orientation = checkStateToBool(item->child(3)->checkState(1));
            temp_pose.hideAttitude = !checkStateToBool(item->child(4)->checkState(1));
            temp_pose.hideHeading = !checkStateToBool(item->child(5)->checkState(1));
            temp_pose.hideLabels = !checkStateToBool(item->child(6)->checkState(1));
            temp_pose.isStamped = checkStateToBool(item->child(7)->checkState(1)) || checkStateToBool(item->child(8)->checkState(1));
            temp_pose.hasCovariance = checkStateToBool(item->child(8)->checkState(1));
            temp_pose.updateMap = checkStateToBool(item->child(9)->checkState(1));

            robot_processed_data->pose.push_back(temp_pose);
        }
        else if(item->type() == ProcessedImage)
        {
            struct RobotCamera temp_image;
            temp_image.name = item->child(0)->text(1);
            temp_image.topicName = item->child(1)->text(1);

            robot_processed_data->images.push_back(temp_image);
        }
        else
        {
            std::cerr << "ERROR -- Unknown processed data type '" << item->type()
                      << "' encountered when storing sensors to config file." << std::endl;
        }
    }
}

///////////////////////// Joints Tab ////////////////////////////

JointsTab::JointsTab(struct RobotJoints *robot_joints, 
    QWidget *parent) : QWidget(parent)
{
    QLabel *topic_name_label = new QLabel(tr("Topic Name"));
    topic_name_lineedit = new QLineEdit(robot_joints->topicName);

    position_checkbox = new QCheckBox(tr("Position"));
    position_checkbox->setChecked(robot_joints->position);
    velocity_checkbox = new QCheckBox(tr("Velocity"));
    velocity_checkbox->setChecked(robot_joints->velocity);
    effort_checkbox = new QCheckBox(tr("Effort"));
    effort_checkbox->setChecked(robot_joints->effort);

    //QLabel *joint_states_label = new QLabel(tr("Joints"));
    QPushButton *add_button = new QPushButton(tr("Add"));
    connect(add_button, SIGNAL(clicked()), this, SLOT(addJoint()));

    QPushButton *edit_button = new QPushButton(tr("Edit"));
    connect(edit_button, SIGNAL(clicked()), this, SLOT(editJoint()));

    QPushButton *remove_button = new QPushButton(tr("Remove"));
    connect(remove_button, SIGNAL(clicked()), this, SLOT(removeJoint()));

    /* Add all joints from the robot configuration file */
    QList<QTreeWidgetItem *> joint_itemlist;
    for(unsigned int i = 0; i < robot_joints->joints.size(); i++)
        joint_itemlist.append(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << robot_joints->joints[i].name << robot_joints->joints[i].displayName)));

    QStringList column_names;
    column_names << "Joint Name" << "Display Name";

    /* Create tree widget */
    joints_treewidget = new QTreeWidget;
    joints_treewidget->setHeaderLabels(column_names);
    joints_treewidget->addTopLevelItems(joint_itemlist);
    joints_treewidget->resizeColumnToContents(0);
    joints_treewidget->setSortingEnabled(true);
    connect(joints_treewidget, SIGNAL(itemDoubleClicked(QTreeWidgetItem *, int)),
            this, SLOT(editJoint(QTreeWidgetItem *)));


    // Create joints tab layout
    QGridLayout *joints_gridlayout = new QGridLayout;
    joints_gridlayout->addWidget(topic_name_label, 0, 0);
    joints_gridlayout->addWidget(topic_name_lineedit, 0, 1);
    joints_gridlayout->addWidget(position_checkbox, 1, 0);
    joints_gridlayout->addWidget(velocity_checkbox, 2, 0);
    joints_gridlayout->addWidget(effort_checkbox, 3, 0);
    //joints_gridlayout->addWidget(joint_states_label, 4, 0);
    //joints_gridlayout->addWidget(add_button, 4, 1, Qt::AlignRight);

    QHBoxLayout *button_hlayout = new QHBoxLayout;
    button_hlayout->addWidget(add_button, 0, Qt::AlignLeft);
    button_hlayout->addWidget(edit_button, 0, Qt::AlignLeft);
    button_hlayout->addStretch();
    button_hlayout->addWidget(remove_button, 0, Qt::AlignRight);

    QVBoxLayout *joints_layout = new QVBoxLayout;
    joints_layout->addLayout(joints_gridlayout);
    joints_layout->addLayout(button_hlayout);
    joints_layout->addWidget(joints_treewidget);
    setLayout(joints_layout);
}

void JointsTab::addJoint()
{
	ComponentDialog add_joint_dialog(this);
	add_joint_dialog.setNameLabelText(tr("Joint Name"));
	add_joint_dialog.setTopicNameLabelText(tr("Display Name"));
	add_joint_dialog.setWindowTitle("Add Joint");

	if(add_joint_dialog.exec())
	{
		joints_treewidget->addTopLevelItem(new QTreeWidgetItem((QTreeWidget *)0,
			(QStringList() << add_joint_dialog.getName() << add_joint_dialog.getTopicName())));
	}
}

void JointsTab::editJoint(QTreeWidgetItem *item)
{
	QTreeWidgetItem *joint_item = item;

	if(joint_item == 0) // Item is unknown
	{
		joint_item = joints_treewidget->currentItem();

		if(joint_item == 0) // No item is selected
			return;
	}

	ComponentDialog edit_joint_dialog(this);
	edit_joint_dialog.setNameLabelText(tr("Joint Name"));
	edit_joint_dialog.setName(joint_item->text(0));
	edit_joint_dialog.setTopicNameLabelText(tr("Display Name"));
	edit_joint_dialog.setTopicName(joint_item->text(1));
	edit_joint_dialog.setWindowTitle(tr("Edit Joint"));

	if(edit_joint_dialog.exec())
	{
		joint_item->setText(0, edit_joint_dialog.getName());
		joint_item->setText(1, edit_joint_dialog.getTopicName());
	}
}

void JointsTab::removeJoint()
{
	// Get currently selected item to remove
	QTreeWidgetItem *item = joints_treewidget->currentItem();

	if(item == 0) // No item was selected
		return;

	delete item;
}

void JointsTab::storeToConfig(struct RobotJoints *robot_joints)
{
	std::cout << "Storing joints to robot configuration struct" << std::endl;

	robot_joints->topicName = topic_name_lineedit->text();
	robot_joints->position = position_checkbox->isChecked();
	robot_joints->velocity = velocity_checkbox->isChecked();
	robot_joints->effort = effort_checkbox->isChecked();
	robot_joints->used = false;

	QTreeWidgetItemIterator it(joints_treewidget);
	while(*it)
	{
		struct RobotJoint temp_joint;
		temp_joint.name = (*it)->text(0);
		temp_joint.displayName = (*it)->text(1);

		robot_joints->joints.push_back(temp_joint);
		robot_joints->used = true;

		it++;
	}
}

////////////////////////////// Controls Tab ////////////////////////////

ControlsTab::ControlsTab(struct RobotControls *robot_controls,
	QWidget *parent) : QWidget(parent)
{
	QLabel *controls_label = new QLabel(tr("Controls"));

	QStringList controls_list;
	controls_list << "Teleoperation (Twist.msg)";

	controls_combobox = new QComboBox;
	controls_combobox->addItems(controls_list);

	QPushButton *add_button = new QPushButton(tr("Add"));
	connect(add_button, SIGNAL(clicked()), this, SLOT(addControl()));

	QPushButton *edit_button = new QPushButton(tr("Edit"));
	connect(edit_button, SIGNAL(clicked()), this, SLOT(editControl()));

	QPushButton *remove_button = new QPushButton(tr("Remove"));
	connect(remove_button, SIGNAL(clicked()), this, SLOT(removeControl()));

	QStringList column_list;
	column_list << tr("Control") << tr("Values");


	// Add all teleoperation from robot config file
	QList<QTreeWidgetItem *> controls_itemlist;
	for(unsigned int i = 0; i < robot_controls->drive.size(); i++)
		controls_itemlist.append(new QTreeWidgetItem((QTreeWidget *)0,
			(QStringList() << tr("Teleoperation") << robot_controls->drive[i].topicName)));


	// Create tree widget
	controls_treewidget = new QTreeWidget;
	controls_treewidget->setHeaderLabels(column_list);
	controls_treewidget->addTopLevelItems(controls_itemlist);
	connect(controls_treewidget, SIGNAL(itemDoubleClicked(QTreeWidgetItem *, int)),
		this, SLOT(editControl(QTreeWidgetItem *)));


	// Dialog layout
	QHBoxLayout *button_hlayout = new QHBoxLayout;
	button_hlayout->addStretch();
	button_hlayout->addWidget(remove_button, 0, Qt::AlignRight);
	button_hlayout->addSpacing(10);
	button_hlayout->addWidget(edit_button, 0, Qt::AlignRight);

	QHBoxLayout *controls_hlayout = new QHBoxLayout;
	controls_hlayout->addWidget(controls_label, 0, Qt::AlignLeft);
	controls_hlayout->addStretch();
	controls_hlayout->addWidget(controls_combobox, 0, Qt::AlignRight);
	controls_hlayout->addWidget(add_button, 0, Qt::AlignRight);

	QVBoxLayout *controls_layout = new QVBoxLayout;
	controls_layout->addLayout(controls_hlayout);
	controls_layout->addWidget(controls_treewidget);
	controls_layout->addLayout(button_hlayout);
	setLayout(controls_layout);
}

void ControlsTab::addControl()
{
	QTreeWidgetItem *item;
	ControlType type = ControlType(controls_combobox->currentIndex() + 1001);

	QString type_str;
	if(type == Teleop)
		type_str = "Teleoperation";
	else
	{
		std::cerr << "ERROR -- Unknown control type '" << type
			<< "' encountered while adding control." << std::endl;
		return;
	}

	if(type == Teleop)
	{
		bool ok;
		QString topic_name = QInputDialog::getText(this, QString("Add %1").arg(type_str),
			tr("Topic Name"), QLineEdit::Normal, QString(), &ok);

		if(ok && !topic_name.isEmpty())
		{
			item = new QTreeWidgetItem(QStringList() << type_str << topic_name);
			controls_treewidget->addTopLevelItem(item);
		}
	}
}

void ControlsTab::editControl(QTreeWidgetItem *item)
{
	QTreeWidgetItem *control_item = item;

        if(control_item == 0) // Item is unknown so grab current item
        {
                control_item = controls_treewidget->currentItem();

                if(control_item == 0) // No item is selected
                        return;
        }

	bool ok;
	QString topic_name = QInputDialog::getText(this, QString("Edit Teleoperation"),
		tr("Topic Name"), QLineEdit::Normal, control_item->text(1), &ok);

	if(ok)
		control_item->setText(1, topic_name);
}

void ControlsTab::removeControl()
{
	// Get current selected item
	QTreeWidgetItem *item = controls_treewidget->currentItem();

	if(item == 0) // No item is selected
		return;

	delete item;
}

void ControlsTab::storeToConfig(struct RobotControls *robot_controls)
{
	std::cout << "Storing controls to robot configuration struct" << std::endl;

        robot_controls->used = false;

        QTreeWidgetItemIterator it(controls_treewidget);
        while(*it)
        {
                struct RobotDrive temp_drive;
                temp_drive.topicName = (*it)->text(1);

                robot_controls->drive.push_back(temp_drive);
                robot_controls->used = true;

                it++;
        }
}

//////////////////////////// Services Tab ///////////////////////////////

ServicesTab::ServicesTab(struct RobotCommands *robot_services, 
	QWidget *parent) : QWidget(parent)
{
	QLabel *services_label = new QLabel(tr("Services"));
	QStringList services_list;
	services_list << "Custom (Empty.srv)"
		<< "Land (Empty.srv)"
 		<< "Takeoff (Empty.srv)";
	services_combobox = new QComboBox;
	services_combobox->addItems(services_list);

	QPushButton *add_button = new QPushButton(tr("Add"));
	connect(add_button, SIGNAL(clicked()), this, SLOT(addService()));

	QPushButton *edit_button = new QPushButton(tr("Edit"));
	connect(edit_button, SIGNAL(clicked()), this, SLOT(editService()));

	QPushButton *remove_button = new QPushButton(tr("Remove"));
	connect(remove_button, SIGNAL(clicked()), this, SLOT(removeService()));

    
	/* Add all services from robot configuration file */
	QList<QTreeWidgetItem *> services_itemlist;
	for(unsigned int i = 0; i < robot_services->custom.size(); i++)
		services_itemlist.append(new QTreeWidgetItem((QTreeWidget *)0,
			(QStringList() << robot_services->custom[i].name << robot_services->custom[i].topicName)));


	QStringList column_list;
	column_list << "Service Name" << "Topic Name";

	/* Create tree widget */
	services_treewidget = new QTreeWidget;
	services_treewidget->setHeaderLabels(column_list);
	services_treewidget->addTopLevelItems(services_itemlist);
	connect(services_treewidget, SIGNAL(itemDoubleClicked(QTreeWidgetItem *, int)),
		this, SLOT(editService(QTreeWidgetItem *)));


	/* Create services tab layout */
	QHBoxLayout *button_hlayout = new QHBoxLayout;
	button_hlayout->addStretch();
	button_hlayout->addWidget(remove_button, 0, Qt::AlignRight);
	button_hlayout->addSpacing(10);
	button_hlayout->addWidget(edit_button, 0, Qt::AlignRight);

	QHBoxLayout *services_hlayout = new QHBoxLayout;
	services_hlayout->addWidget(services_label, 0, Qt::AlignLeft);
	services_hlayout->addStretch();
	services_hlayout->addWidget(services_combobox, 0, Qt::AlignRight);
	services_hlayout->addWidget(add_button, 0, Qt::AlignRight);

	QVBoxLayout *services_layout = new QVBoxLayout;
	services_layout->addLayout(services_hlayout);
	services_layout->addWidget(services_treewidget);
	services_layout->addLayout(button_hlayout);
	setLayout(services_layout);
}

void ServicesTab::addService()
{
	ComponentDialog add_service_dialog(this);
	add_service_dialog.setNameLabelText(tr("Service Name"));
	add_service_dialog.setWindowTitle("Add Service");

	if(services_combobox->currentText().startsWith(QString("Takeoff")))
		add_service_dialog.setName(tr("Takeoff"));
	else if(services_combobox->currentText().startsWith(QString("Land")))
		add_service_dialog.setName(tr("Land"));

	if(add_service_dialog.exec())
		services_treewidget->addTopLevelItem(new QTreeWidgetItem((QTreeWidget *)0,
			(QStringList() << add_service_dialog.getName() << add_service_dialog.getTopicName())));
}

void ServicesTab::editService(QTreeWidgetItem *item)
{
	QTreeWidgetItem *service_item = item;

	if(service_item == 0) // Item is unknown so get currently selected item
	{
		service_item = services_treewidget->currentItem();

		if(service_item == 0) // No item is selected
			return;
	}

	ComponentDialog edit_service_dialog(this);
	edit_service_dialog.setName(service_item->text(0));
	edit_service_dialog.setNameLabelText(tr("Service Name"));
	edit_service_dialog.setTopicName(service_item->text(1));
	edit_service_dialog.setWindowTitle(tr("Edit Service"));

	if(edit_service_dialog.exec())
	{
		service_item->setText(0, edit_service_dialog.getName());
		service_item->setText(1, edit_service_dialog.getTopicName());
	}
}

void ServicesTab::removeService()
{
	QTreeWidgetItem *item = services_treewidget->currentItem();

	if(item == 0) // No item is selected
		return;

	delete item;
}

void ServicesTab::storeToConfig(struct RobotCommands *robot_commands)
{
	std::cout << "Storing commands to robot configuration struct" << std::endl;

	robot_commands->used = false;

	QTreeWidgetItemIterator it(services_treewidget);
        while(*it)
        {
                struct RobotCommandCustom temp_custom;
		temp_custom.name = (*it)->text(0);
                temp_custom.topicName = (*it)->text(1);

                robot_commands->custom.push_back(temp_custom);
                robot_commands->used = true;

                it++;
        }
}

////////////////////////// Diagnostics Tab ////////////////////////////////

DiagnosticsTab::DiagnosticsTab(QWidget *parent) : QWidget(parent)
{
	QLabel *topic_name_label = new QLabel(tr("Topic Name"));
	QLineEdit *topic_name_lineedit = new QLineEdit;

	QLabel *path_label = new QLabel(tr("Path"));
	QLineEdit *path_lineedit = new QLineEdit;

	QLabel *diagnostics_label = new QLabel(tr("Diagnostics"));
	QStringList diagnostics_list;
	diagnostics_list << "Battery Level" << "Diagnostic";
	QComboBox *diagnostics_combobox = new QComboBox;
	diagnostics_combobox->addItems(diagnostics_list);
	QPushButton *add_button = new QPushButton(tr("Add"));

	QScrollArea *diagnostics_scrollarea = new QScrollArea;

	QHBoxLayout *add_diagnostic_hlayout = new QHBoxLayout;
	add_diagnostic_hlayout->addWidget(diagnostics_combobox);
	add_diagnostic_hlayout->addWidget(add_button);

	QGridLayout *diagnostics_gridlayout = new QGridLayout;
	diagnostics_gridlayout->addWidget(topic_name_label, 0, 0);
	diagnostics_gridlayout->addWidget(topic_name_lineedit, 0, 1);
	diagnostics_gridlayout->addWidget(path_label, 1, 0);
	diagnostics_gridlayout->addWidget(path_lineedit, 1, 1);
	diagnostics_gridlayout->addWidget(diagnostics_label, 2, 0);
	diagnostics_gridlayout->addLayout(add_diagnostic_hlayout, 2, 1);
    
	QVBoxLayout *diagnosticstab_layout = new QVBoxLayout;
	diagnosticstab_layout->addLayout(diagnostics_gridlayout);
	diagnosticstab_layout->addWidget(diagnostics_scrollarea);
	setLayout(diagnosticstab_layout);
}


/////////////////////// Robot Config File Dialog //////////////////////////
RobotConfigFileDialog::RobotConfigFileDialog(
	struct RobotConfig *new_robot_config, QWidget *parent) : QDialog(parent)
{
	robot_config = new_robot_config;

	general_tab = new GeneralTab(robot_config);
	sensors_tab = new SensorsTab(&robot_config->sensors);
	processed_data_tab = new ProcessedDataTab(&robot_config->processedData);
	joints_tab = new JointsTab(&robot_config->joint_states);
	controls_tab = new ControlsTab(&robot_config->controls);
	services_tab = new ServicesTab(&robot_config->commands);
	diagnostics_tab = new DiagnosticsTab;

	// Create tabs
	tab_widget = new QTabWidget;
	tab_widget->addTab(general_tab, tr("General"));
	tab_widget->addTab(sensors_tab, tr("Sensors"));
	tab_widget->addTab(processed_data_tab, tr("Processed Data"));
	tab_widget->addTab(joints_tab, tr("Joints"));
	tab_widget->addTab(controls_tab, tr("Controls"));
	tab_widget->addTab(services_tab, tr("Services"));
	tab_widget->addTab(diagnostics_tab, tr("Diagnostics"));

	button_box = new QDialogButtonBox(QDialogButtonBox::Cancel |
		QDialogButtonBox::Save);
	connect(button_box, SIGNAL(accepted()), this, SLOT(saveConfig()));
	connect(button_box, SIGNAL(rejected()), this, SLOT(reject()));

	// Create dialog layout
	QVBoxLayout *dialog_layout = new QVBoxLayout;
	dialog_layout->setSizeConstraint(QLayout::SetNoConstraint);
	dialog_layout->addWidget(tab_widget);
	dialog_layout->addWidget(button_box);
	setLayout(dialog_layout);
}

void RobotConfigFileDialog::saveConfig()
{
	QString file_name = QFileDialog::getSaveFileName(this, tr("Save Robot Configuration File"),
		QDir::homePath(), tr("XML files (*.xml)"));

	if(file_name.isNull())
		return;

	if(!file_name.endsWith(QString(".xml")))
		file_name.append(".xml");

	robot_config->defaults(); // Clear current robot config

	// Save all data to robot configuration file
	robot_config->configFilePath = file_name;
	general_tab->storeToConfig(robot_config);
	sensors_tab->storeToConfig(&robot_config->sensors);
	processed_data_tab->storeToConfig(&robot_config->processedData);
	joints_tab->storeToConfig(&robot_config->joint_states);
	controls_tab->storeToConfig(&robot_config->controls);
	services_tab->storeToConfig(&robot_config->commands);
	//diagnostics_tab->storeToConfig(&robot_config->diagnostics);

	QFile file(robot_config->configFilePath);
	if (!file.open(QIODevice::WriteOnly))
	{
		std::cerr << "ERROR: Could not open " << robot_config->configFilePath.toStdString( ) << " for writing!" << std::endl;
		return;
	}
	else
		robot_config->exportData(&file);
	file.close();

	accept();
}
