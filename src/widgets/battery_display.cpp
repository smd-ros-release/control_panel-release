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
 * \file   battery_display.cpp
 * \date   July 15, 2011
 * \author Matt Richard
 */
#include <QtGui>
#include "control_panel/widgets/battery_display.h"

BatteryDisplay::BatteryDisplay(QWidget *parent) : QGraphicsView(parent)
{
	// Set graphics view properties
	setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
	setFrameStyle(QFrame::NoFrame | QFrame::Plain);
	setBackgroundRole(QPalette::Window);
	setAutoFillBackground(true);

	// Create graphics items
	battery_background_item = new QGraphicsPixmapItem(
		QPixmap(":/images/battery/battery_background.png"));
	battery_foreground_item = new QGraphicsPixmapItem(
		QPixmap(":/images/battery/battery.png"));

	QBrush brush(QColor(0, 255, 0));
	QPen pen(Qt::NoPen);

	battery_level_item = new QGraphicsRectItem;
	battery_level_item->setBrush(brush);
	battery_level_item->setPen(pen);

	battery_text_item = new QGraphicsTextItem;
	battery_text_item->setDefaultTextColor(Qt::white);

	// Create graphics scene
	scene = new QGraphicsScene(this);
	scene->addItem(battery_background_item);
	scene->addItem(battery_level_item);
	scene->addItem(battery_text_item);
	scene->addItem(battery_foreground_item);
	scene->setSceneRect(scene->itemsBoundingRect());
	setScene(scene);
	setFixedSize(scene->sceneRect().toRect().size());


	// Initialize battery level
	setBatteryLevel(0.0);
}

void BatteryDisplay::setBatteryLevel(float level)
{
	// Set battery level and update display
	battery_level = level;
	battery_level_item->setRect(0, 0, 90 * (battery_level / 100.0) + 5,
		scene->height());

	battery_text_item->setPlainText(QString("%1%").arg(battery_level));
	// Center text
	battery_text_item->setPos((scene->width() / 2.0) -
		(battery_text_item->boundingRect().width() / 2.0),
		(scene->height() - battery_text_item->boundingRect().height()) / 2.0);
}
