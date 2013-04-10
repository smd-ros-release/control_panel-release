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
 * \file   heading_indicator.cpp
 * \date   July 14, 2011
 * \author Matt Richard
 */
#include <QtGui>
#include "control_panel/widgets/heading_indicator.h"

HeadingIndicator::HeadingIndicator(QWidget *parent) : QGraphicsView(parent)
{
	// Create background
	background_item = new QGraphicsPixmapItem(QPixmap(
		":/images/heading_indicator/compass_background.png").scaled(
		100, 100, Qt::KeepAspectRatio, Qt::SmoothTransformation));
	background_item->setTransformOriginPoint(
		background_item->pixmap().width() / 2.0,
		background_item->pixmap().height() / 2.0);

	// Create cover
	cover_item = new QGraphicsPixmapItem(QPixmap(
		":/images/heading_indicator/compass_cover.png").scaled(
		100, 100, Qt::KeepAspectRatio, Qt::SmoothTransformation));

	// Heading indicator
	indicator_item = new QGraphicsPixmapItem(QPixmap(
		":/images/heading_indicator/compass_indicator.png").scaled(
		100, 100, Qt::KeepAspectRatio, Qt::SmoothTransformation));

	// Create the scene
	scene = new QGraphicsScene(this);
	scene->addItem(background_item);
	scene->addItem(indicator_item);
 	scene->addItem(cover_item);

	// Set graphics view's properties
	setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
	setFrameStyle(QFrame::NoFrame | QFrame::Plain);
	setStyleSheet("background-color: transparent;");
	setScene(scene);

	// Initialize heading indicator
	setYaw(0.0);
}

void HeadingIndicator::setYaw(double angle)
{
	yaw = angle;

	// Update the heading indicator
	background_item->setRotation(yaw);
}
