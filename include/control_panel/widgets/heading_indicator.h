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
 * \file   heading_indicator.h
 * \date   July 14, 2011
 * \author Matt Richard
 */
#ifndef CONTROL_PANEL_HEADING_INDICATOR_H
#define CONTROL_PANEL_HEADING_INDICATOR_H

#include <QGraphicsView>

QT_BEGIN_NAMESPACE
class QGraphicsScene;
class QGraphicsPixmapItem;
QT_END_NAMESPACE


/**
 * \class  HeadingIndicator
 * \brief  A graphic to display the yaw of a robot.
 */
class HeadingIndicator : public QGraphicsView
{
	Q_OBJECT

	public:
		/**
		 * \brief Constructor. 
		 *
		 * \param parent The parent widget.
		 */
		HeadingIndicator(QWidget *parent = 0);

		/**
		 * \brief Returns the current yaw value.
		 */
		double getYaw() const { return yaw; }

		/**
		 * \brief Updates the yaw and rotates the heading indicator.
		 *
		 * \param angle New yaw angle in degrees.
		 */
		void setYaw(double angle);

	private:
		QGraphicsScene *scene;
		QGraphicsPixmapItem *background_item;
		QGraphicsPixmapItem *indicator_item;
		QGraphicsPixmapItem *cover_item;

		double yaw; // Robot's yaw in degrees
};

#endif // CONTROL_PANEL_HEADING_INDICATOR_H
