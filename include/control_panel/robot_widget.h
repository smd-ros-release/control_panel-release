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
 * \file   robot_widget.h
 * \date   Oct 17, 2011
 * \author Matt Richard
 * \brief  @todo Write brief description
 * 
 * RobotWidget is the widget displayed in the scroll area in the
 * MainTab for one robot in the robot list. This reads every
 * robot's configuration file and displays the basic detailes of
 * the robot.
 */
#ifndef CONTROL_PANEL_ROBOT_WIDGET_H
#define CONTROL_PANEL_ROBOT_WIDGET_H

#include <QFrame>

QT_BEGIN_NAMESPACE
class QHBoxLayout;
class QLabel;
class QCheckBox;
class QImage;
class QPalette;
class QMouseEvent;
class QString;
QT_END_NAMESPACE

#include "robot_config.h"
#include <string>

/**
 * \class RobotWidget
 * \brief @todo Write brief class description
 */
class RobotWidget : public QFrame
{
	Q_OBJECT

	public:
		RobotWidget(QWidget *parent = 0);
		void setRobotPicture(const QImage &robot_image);
		void setRobotName(const std::string &name);
        void setRobotNamespace(const std::string &ns);
		QString getRobotName() const;
		QString getConfigPath() const;
		void setSystem(const std::string &robot_system);
		void setDriveSystem(const std::string &robot_drive_system);
		void setConfigPath(const std::string &robot_config_path);
		void setSelected(bool selected);
		bool isSelected() const;
		void setRobot(RobotConfig *rbt);

	signals:
		void singleRobotSelected(const QStringList &robot, bool auto_connect);

	protected:
		void mousePressEvent(QMouseEvent *event);
		void mouseDoubleClickEvent(QMouseEvent *event);

	private slots:
		void selectCheckboxChanged(int state);

	private:
		void createWidgets();
		void createLayout();

		QString robot_name;
		QString configFilePath;

		QHBoxLayout *robot_widget_layout;
		QPalette default_background_palette;
		QPalette selected_background_palette;

		QLabel *robot_picture_label;
		QLabel *robot_name_label;
		QLabel *system_label;
		QLabel *drive_system_label;
        QLabel *robot_namespace_label;

		QCheckBox *select_checkbox;
};

#endif // CONTROL_PANEL_ROBOT_WIDGET_H
