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
 * \file   gps_display.cpp
 * \date   Jan 20, 2012
 * \author Matt Richard
 */
#include <QtGui>
#include "control_panel/widgets/gps_display.h"
#include "control_panel/globals.h"

GpsDisplay::GpsDisplay(QWidget *parent) : QWidget(parent)
{
	gps_name = "GPS";

	// Assume use of all labels
	use_lat = true;
	use_long = true;
	use_alt = true;

	createWidget();
}

GpsDisplay::GpsDisplay(const QString &name, bool show_lat, bool show_long,
	bool show_alt, QWidget *parent) : QWidget(parent)
{
	if(name.isEmpty())
		gps_name = "GPS";
	else
		gps_name = name;

	use_lat = show_lat;
	use_long = show_long;
	use_alt = show_alt;

	createWidget();
}

void GpsDisplay::createWidget()
{
	// counter for the grid layout
	int rows = 0;

	// initialize priavte variables
	zeroValues();

	widget_layout = new QHBoxLayout;

	// Grid layout for labels
	QGridLayout *gps_gridlayout = new QGridLayout;

	// Only display the GPS name if any attributes will be displayed
	if(use_lat || use_long || use_alt)
	{
		name_label = new QLabel(gps_name);

		gps_gridlayout->addWidget(name_label, rows, 0, 1, 0);
		rows++;

		gps_gridlayout->setColumnMinimumWidth(1, 8);
	}

	// Create latitude label and add to layout
	if(use_lat)
	{
		latitude_label = new QLabel;

		gps_gridlayout->addWidget(new QLabel(tr("Latitude: ")), rows, 0, Qt::AlignLeft);
		gps_gridlayout->addWidget(latitude_label, rows, 1, Qt::AlignRight);
		rows++;
	}

	// Create longitude label and add to layout
	if(use_long)
	{
		longitude_label = new QLabel;

		gps_gridlayout->addWidget(new QLabel(tr("Longitude: ")), rows, 0, Qt::AlignLeft);
		gps_gridlayout->addWidget(longitude_label, rows, 1, Qt::AlignRight);
		rows++;
	}

	// Create altitude label and add to layout
	if(use_alt)
	{
		altitude_label = new QLabel;

		gps_gridlayout->addWidget(new QLabel(tr("Altitude: ")), rows, 0, Qt::AlignLeft);
		gps_gridlayout->addWidget(altitude_label, rows, 1, Qt::AlignLeft);
		rows++;
	}

	widget_layout->addLayout(gps_gridlayout);
	setLayout(widget_layout);
}

void GpsDisplay::zeroValues()
{
	// Zero private variables
	latitude = 0.0;
	longitude = 0.0;
	altitude = 0.0;
}

void GpsDisplay::updateGpsDisplay(double lat, double lon, double alt)
{
	char direction; // N, S, E, or W

	latitude = lat;
	longitude = lon;
	altitude = alt;

	// Update latitude
	if(use_lat)
	{
		// Check latitude direction
		if(latitude >= 0)
			direction = 'N';
		else
		{
			direction = 'S';
			latitude *= -1; // Set latitude to be positive
		}

		latitude_label->setText(QString("%1 ").arg(latitude, 0, 'f', 3) +
			Globals::DegreesSymbol + direction);
	}


	// Update longitude
	if(use_long)
	{
		// Check longitude direction
		if(longitude >= 0)
			direction = 'E';
		else
		{
			direction = 'W';
			lon *= -1; // Set longitude to be positive
		}

		longitude_label->setText(QString("%1 ").arg(longitude, 0, 'f', 3) +
			Globals::DegreesSymbol + direction);
	}


	// Update altitude
	if(use_alt)
		altitude_label->setText(QString("%1 m").arg(altitude, 0, 'f', 3));
}
