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
 * \file   robot_tab.h
 * \date   Aug 6, 2011
 * \author Matt Richard, Scott K Logan
 * \brief  This creates the tab layout and handles the ROS communication for a single robot.
 */
#ifndef CONTROL_PANEL_ROBOT_TAB_H
#define CONTROL_PANEL_ROBOT_TAB_H

#include <QWidget>

QT_BEGIN_NAMESPACE
class QVBoxLayout;
QT_END_NAMESPACE

#include <string>
#include "data_pane.h"
#include "display_pane.h"
#include "node_manager.h"
#include "globals.h"
#include "robot_config.h"
#include "call_service_dialog.h"


/**
 * \class RobotTab
 * \brief
 */
class RobotTab : public QWidget
{
	Q_OBJECT

	public:
		RobotTab(struct RobotConfig *robot_config, QWidget *parent = 0);
		~RobotTab();
        void callService();
		void connectToRobot();
		void disconnectRobot();
		struct RobotConfig * getConfig();
		bool isKeyboardEnabled() const;
		bool robotConnected();
        void setRCMode(int rc_mode);

		NodeManager *node_manager;
        ControlNode *node; /* <-- Why is this here? */

	protected:
		void keyPressEvent(QKeyEvent *event);
		void keyReleaseEvent(QKeyEvent *event);

	signals:
		void connectionStatusChanged(int status, const QString &robot_name);

	private slots:
		void updateConnectionStatus(int status);
		void processDiagnostic(const QString &, const QString &);

	private:
		void createLayout();
        void setupDataPane();

		struct RobotConfig *robot_config;
		bool use_keyboard;

		QVBoxLayout *display_layout;

		// GUI Panes
		DataPane *data_pane;
        DisplayPane *raw_data_display;
        DisplayPane *processed_data_display;
};

#endif // CONTROL_PANEL_TAB_H
