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
 * \file   robot_widget.cpp
 * \date   Oct 17, 2011
 * \author Matt Richard
 * \brief  Displays details of a robot to be listed in the main tab.
 *
 * RobotWidget is the widget displayed in the scroll area in the
 * MainTab for one robot in the robot list. This reads every
 * robot's configuration file and displays the basic detailes of
 * the robot.
 */
#include <QtGui>
#include "control_panel/robot_widget.h"

RobotWidget::RobotWidget(QWidget *parent) : QFrame(parent)
{
	createWidgets();
	createLayout();

	default_background_palette = palette();
	selected_background_palette.setColor(QPalette::Background,
		QColor(50, 150, 255, 25));

	setLayout(robot_widget_layout);
	setFixedHeight(100);
	setFrameStyle(QFrame::Box | QFrame::Sunken);
	setAutoFillBackground(true);
}

void RobotWidget::mousePressEvent(QMouseEvent *event)
{
	bool state = false;
	
	if(event->button() == Qt::LeftButton)
	{
		state = select_checkbox->isChecked();
		select_checkbox->setChecked(!state);
	}
}

void RobotWidget::mouseDoubleClickEvent(QMouseEvent *event)
{
	QStringList robot_list;
	robot_list << configFilePath;

	if(event->button() == Qt::LeftButton)
		emit singleRobotSelected(robot_list, true);

	setSelected(false);
}

void RobotWidget::createWidgets()
{
	robot_picture_label = new QLabel;
	robot_picture_label->setFixedSize(80, 80);

	QFont font;
	font.setPointSize(15);

	robot_name_label = new QLabel;
	robot_name_label->setFont(font);

	system_label = new QLabel(tr("System: "));

	drive_system_label = new QLabel(tr("Drive System: "));

    robot_namespace_label =new QLabel(tr("Namespace: "));

	select_checkbox = new QCheckBox(tr("Select"));
	connect(select_checkbox, SIGNAL(stateChanged(int)),
		this, SLOT(selectCheckboxChanged(int)));
}

void RobotWidget::createLayout()
{
    QLabel *sys = new QLabel("System: ");
    QLabel *d_sys = new QLabel("Drive System: ");
    QLabel *ns = new QLabel("Namespace: ");

    QGridLayout *robot_info_gridlayout = new QGridLayout;
    robot_info_gridlayout->setVerticalSpacing(0);
    robot_info_gridlayout->addWidget(robot_name_label, 0, 0, 1, 2, Qt::AlignLeft);
    robot_info_gridlayout->addWidget(sys, 1, 0, Qt::AlignLeft);
    robot_info_gridlayout->addWidget(system_label, 1, 1, Qt::AlignLeft);
    robot_info_gridlayout->addWidget(d_sys, 2, 0, Qt::AlignLeft);
    robot_info_gridlayout->addWidget(drive_system_label, 2, 1, Qt::AlignLeft);
    robot_info_gridlayout->addWidget(ns, 3, 0, Qt::AlignLeft);
    robot_info_gridlayout->addWidget(robot_namespace_label, 3, 1, Qt::AlignLeft);

	robot_widget_layout = new QHBoxLayout;
	robot_widget_layout->addWidget(robot_picture_label, 0, Qt::AlignLeft);
    robot_widget_layout->addLayout(robot_info_gridlayout);
    robot_widget_layout->addStretch();
	robot_widget_layout->addWidget(select_checkbox, 0, Qt::AlignRight);
}

void RobotWidget::setRobotPicture(const QImage &robot_image)
{
	robot_picture_label->setPixmap(QPixmap::fromImage(robot_image.scaled(
		robot_picture_label->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation)));
}

void RobotWidget::setRobotName(const std::string &name)
{
	robot_name = name.c_str();

	robot_name_label->setText(getRobotName());
}

void RobotWidget::setRobotNamespace(const std::string &ns)
{
    if(ns == "")
        robot_namespace_label->setText(tr("/"));
    else
        robot_namespace_label->setText(ns.c_str());
}

QString RobotWidget::getRobotName() const
{
	if(robot_name.isEmpty())
		return QString("(unnamed robot)");
	return robot_name;
}

QString RobotWidget::getConfigPath() const
{
	return configFilePath;
}

void RobotWidget::setSystem(const std::string &robot_system)
{
	system_label->setText(tr(robot_system.c_str()));
}

void RobotWidget::setDriveSystem(const std::string &robot_drive_system)
{
	drive_system_label->setText(tr(robot_drive_system.c_str()));
}

void RobotWidget::setConfigPath(const std::string &robot_config_path)
{
	configFilePath = robot_config_path.c_str();
}

void RobotWidget::setSelected(bool selected)
{
	select_checkbox->setChecked(selected);
}

bool RobotWidget::isSelected() const
{
	return select_checkbox->isChecked();
}

void RobotWidget::selectCheckboxChanged(int state)
{
	if(state == Qt::Checked)
		setPalette(selected_background_palette);
	else
		setPalette(default_background_palette);
}

void RobotWidget::setRobot(RobotConfig *rbt)
{
	setConfigPath(rbt->configFilePath.toStdString());
	setRobotName(rbt->robotName.toStdString());
	setSystem(rbt->system.toStdString());
	setDriveSystem(rbt->driveSystem.toStdString());
	setRobotPicture(rbt->image);
    setRobotNamespace(rbt->nameSpace.toStdString());
}

