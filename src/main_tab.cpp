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
 * \file   main_tab.cpp
 * \date   Oct 10, 2011
 * \author Matt Richard, Scott K Logan
 * \brief  Displays the list of all robots
 *
 * The main tab displayed when the Control Panel is first
 * executed. This tab displays the list of known robots and allows
 * the user to select one or more robots to connect to and can
 * specify if the robot should be automatically connected to or
 * requires manual connection.
 */
#include <QtGui>
#include "control_panel/main_tab.h"
#include <iostream>

MainTab::MainTab(const QString &robots, QWidget *parent)
	: QWidget(parent)
{
	robot_directory = robots;

	createWidgets();
	createLayout();

	setMinimumWidth(800);
	setMinimumHeight(400);
	setLayout(main_tab_layout);
}


void MainTab::createWidgets()
{
	QFont font;
	font.setPointSize(20);

	robot_list_label = new QLabel(tr("Robot List"));
	robot_list_label->setFont(font);

	robot_list_scrollarea = new QScrollArea;
	robot_list_scrollarea->setWidgetResizable(true);
	robot_list_widget = new QWidget;

	deselect_all_button = new QPushButton(tr("Deselect All"));
	connect(deselect_all_button, SIGNAL(clicked()), this, SLOT(deselectAllRobots()));

	auto_connect_checkbox = new QCheckBox(tr("Connect Automatically"));
	auto_connect_checkbox->setChecked(true);

	load_button = new QPushButton(tr("Load"));
	connect(load_button, SIGNAL(clicked()), this, SLOT(loadButtonPressed()));
}


void MainTab::createLayout()
{
	QHBoxLayout *button_hlayout = new QHBoxLayout;
	button_hlayout->addWidget(deselect_all_button, 0, Qt::AlignLeft);
	button_hlayout->addStretch();
	button_hlayout->addWidget(auto_connect_checkbox, 0, Qt::AlignRight);
	button_hlayout->addWidget(load_button, 0, Qt::AlignRight);

	robot_list_layout = new QVBoxLayout;
	loadRobots();
	robot_list_layout->addStretch();

	robot_list_widget->setLayout(robot_list_layout);
	robot_list_scrollarea->setWidget(robot_list_widget);

	main_tab_layout = new QVBoxLayout;
	main_tab_layout->addWidget(robot_list_label, 0, Qt::AlignLeft);
	main_tab_layout->addWidget(robot_list_scrollarea);
	main_tab_layout->addLayout(button_hlayout);
}

QString MainTab::getFirstSelectedRobot()
{
	QString robot_config_path;
	bool robot_found = false;

	/* Search for the first selected robot */
	for(int i = 0; i < robot_list_layout->count() - 1 && !robot_found; i++)
	{
		robot_widget = (RobotWidget *)robot_list_layout->itemAt(i)->widget();
		if(robot_widget->isSelected())
		{
			robot_config_path = robot_widget->getConfigPath();
			robot_found = true;
		}
	}

    return robot_config_path;
}

QStringList MainTab::getSelectedRobots()
{
	QStringList robot_list;

	/* Add all selected robots' names to the string list */
	for(int i = 0; i < robot_list_layout->count() - 1; i++)
	{
		robot_widget = (RobotWidget *)robot_list_layout->itemAt(i)->widget();
		if(robot_widget->isSelected())
			robot_list << robot_widget->getConfigPath();
	}

	return robot_list;
}

void MainTab::insertRobot(struct RobotConfig *config, bool sorted)
{
	bool found = false;
	int i;

	robot_widget = new RobotWidget;
	robot_widget->setRobot(config);
	connect(robot_widget, SIGNAL(singleRobotSelected(const QStringList &, bool)),
		this, SIGNAL(loadRobots(const QStringList &, bool)));

	if(sorted)
	{
		// Find the correct position to insert the robot
		for(i = 0; i < robot_list_layout->count() - 1 && !found; i++)
			if(((RobotWidget *)robot_list_layout->itemAt(i)->widget())->getRobotName() > config->robotName)
				found = true;

		if(found) // Found a position to insert the robot
			robot_list_layout->insertWidget(i - 1, robot_widget);
		else // Insert robot at end of list
			robot_list_layout->insertWidget(i, robot_widget);
	}
	else // Insert robot at end of list
		robot_list_layout->insertWidget(robot_list_layout->count() - 1, robot_widget);
}

int MainTab::numSelected()
{
	int count = 0;

	/* Count the number of selected robots */
	for(int i = 0; i < robot_list_layout->count() - 1; i++)
	{
		robot_widget = (RobotWidget *)robot_list_layout->itemAt(i)->widget();
		if(robot_widget->isSelected())
			count++;
	}

	return count;
}

QStringList MainTab::removeSelectedRobots()
{
        QStringList robot_list;

        /* Find all selected robots and delete them */
        for(int i = 0; i < robot_list_layout->count() - 1; i++)
        {
                robot_widget = (RobotWidget *)robot_list_layout->itemAt(i)->widget();
                if(robot_widget->isSelected())
		{
                        robot_list << robot_widget->getConfigPath();

			robot_widget = (RobotWidget *)robot_list_layout->takeAt(i)->widget();
			delete robot_widget;
			i--;
		}
        }

        return robot_list;
}


void MainTab::loadRobots()
{
	RobotConfig robot_config;

	QDir dir(":/robots");
	QStringList robot_list = dir.entryList(QDir::Files, QDir::Name);

	/* load the robots */
	for(int i = 0; i < robot_list.size(); i++)
	{
		QFile curr_file(dir.filePath(robot_list.at(i)));
		std::cout << "Loading robot from " << dir.filePath(robot_list.at(i)).toStdString() << std::endl;
		if(curr_file.open(QIODevice::ReadOnly) && !robot_config.loadFrom(&curr_file))
		{
			robot_widget = new RobotWidget;
			robot_widget->setRobot(&robot_config);
			connect(robot_widget, SIGNAL(singleRobotSelected(const QStringList &, bool)),
				this, SIGNAL(loadRobots(const QStringList &, bool)));
			robot_list_layout->addWidget(robot_widget);
		}
		curr_file.close();
	}
}

bool MainTab::loadRobot(const QString &path)
{
	RobotConfig robot_config;

	// Load robot and add to list, sorted
	QFile file(path);
	std::cout << "Loading robot from " << path.toStdString() << std::endl;
	if(file.open(QIODevice::ReadOnly) && !robot_config.loadFrom(&file))
	{
		insertRobot(&robot_config);
		return true;
	}
	return false;
}

void MainTab::deselectAllRobots()
{
	QLayoutItem *temp;

	for(int i = 0; i < robot_list_layout->count() - 1; i++)
	{
		temp = robot_list_layout->itemAt(i);

		robot_widget = (RobotWidget *)temp->widget();
		robot_widget->setSelected(false);
	}
}

void MainTab::loadButtonPressed()
{
	QStringList selected_robots;
	QLayoutItem *temp;
	int robot_list_count = robot_list_layout->count();

	if(robot_list_count > 1)
	{
		for(int i = 0; i < robot_list_layout->count() - 1; i++)
		{
			temp = robot_list_layout->itemAt(i);

			robot_widget = (RobotWidget *)temp->widget();
			if(robot_widget->isSelected())
			{
				selected_robots << robot_widget->getConfigPath();

				robot_widget->setSelected(false);
			}
		}

		emit loadRobots(selected_robots, auto_connect_checkbox->isChecked());
	}
}
