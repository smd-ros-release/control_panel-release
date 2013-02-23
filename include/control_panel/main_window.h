/*
 * Copyright (c) 2011, 2012 SDSM&T CSR.
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
 * \file   main_window.h
 * \date   June 2011
 * \author Matt Richard, Scott K Logan
 */
#ifndef CONTROL_PANEL_MAIN_WINDOW_H
#define CONTROL_PANEL_MAIN_WINDOW_H

#include <QMainWindow>
#include <QIcon>

QT_BEGIN_NAMESPACE
class QAction;
class QActionGroup;
class QKeyEvent;
class QMenu;
class QMessageBox;
class QTabWidget;
class QString;
class QStringList;
QT_END_NAMESPACE

#include "nodes/qt_node.h"
#include "main_tab.h"
#include "robot_tab.h"
#include "robot_config_file_dialog.h"
#include "velocity_scale_dialog.h"

/**
 * \class MainWindow
 * \brief SRS Control Panel main window. MainWindow manages the menus, main tab, and all robot tabs.
 */
class MainWindow : public QMainWindow
{
	Q_OBJECT

	public:
		/**
		 * \brief Constructor. Initializes QtNode and sets up the menus and tab widget.
		 *
		 * \param argc Command line argument count (to be passed into QtNode).
		 * \param argv Command line argument vector (to be passed into QtNode).
		 */
		MainWindow(int argc, char **argv);

		/**
		 * \brief Uses QSettings to read the Control Panel's saved settings.
		 */
		void readSettings();

		/**
		 * \brief Uses QSettings to write the Control Panel's settings just before exiting.
		 */
		void writeSettings();

	protected:
		/**
		 * \brief Overloaded from QMainWindow.
		 *
		 * Checks if any robots are currently connected to the Control Panel.
		 * If there are any robots connect, the user is warned about the connected
		 * robots and is prompted to verify that they want to disconnect from the
		 * robots and close the program.
		 */
		void closeEvent(QCloseEvent *event);

		/**
		 * \brief Overloaded from QMainWindow.
		 *
		 * Creates a popup menu when the user right clicks inside the window.
		 */
		void contextMenuEvent(QContextMenuEvent *event);

	private slots:
		void about();
		void callRobotService();
		void callService();
		void closeTab(int index);
		void editRobotConfigFile();
		void removeRobotConfigFiles();
		void setMaxVelocity();
		void fullScreenChanged(bool checked);
		void help();
		void loadSelectedRobots(const QStringList &robot_list, bool auto_connect);
		void newRobotConfigFile();
		void openTabInWindow();
		void openRobotConfig();
		//void openWidgetInWindow();
		//void updateJoystickAxis(int axis, double value);
		//void updateJoystickButton(int axis, bool state);
		void startConnection();
		void stopConnection();
		void tabChanged(int index);
		void toggleRC(QAction *action);
		void updateTabIcon(int status, const QString &robot_name);
		void startRuntimeMonitor();
		void startRviz();
		void startRxconsole();
		void startRxgraph();
		void startDynamicReconfigure();

	private:
		void createActions();
		void createMenus();
		void createTab();

		QtNode *qt_node;

		QTabWidget *tab_widget;
		MainTab *main_tab;

		QString robot_directory;

		QStringList robot_config_list;

		/* Icons to display on a robot's tab for indicating connection status */
		QIcon robot_disconnected_icon;
		QIcon robot_connecting_icon;
		QIcon robot_connected_icon;

		// Menu items
		QMenu *file_menu;
		QMenu *edit_menu;
		QMenu *view_menu;
		QMenu *connections_menu;
		QMenu *tools_menu;
		QMenu *help_menu;

		// Menu action items
		QAction *new_robot_action;
		QAction *open_config_action;
		QAction *exit_action;
		QAction *configuration_file_action;
		QAction *remove_config_action;
		QAction *set_velocity_action;
		QAction *gestures_action;
		QAction *show_menu_bar_action;
		QAction *full_screen_action;
		QAction *tab_in_window_action;
		QAction *widget_in_window_action;
		QAction *system_diagnostics_action;
		QAction *connect_action;
		QAction *disconnect_action;
		QAction *manual_mode_action;
		QAction *semiautonomous_mode_action;
		QAction *autonomous_mode_action;
		QAction *disable_rc_action;
		QAction *keyboard_rc_action;
		QAction *joystick_rc_action;
		QAction *call_robot_service_action;
		QAction *call_service_action;
		QAction *runtime_monitor_action;
		QAction *rviz_action;
		QAction *rxconsole_action;
		QAction *rxgraph_action;
		QAction *dynamic_reconfigure_action;
		QAction *help_action;
		QAction *about_action;

		QActionGroup *robot_mode_actiongroup;
		QActionGroup *robot_rc_actiongroup;
};

#endif // CONTROL_PANEL_MAIN_WINDOW_H
