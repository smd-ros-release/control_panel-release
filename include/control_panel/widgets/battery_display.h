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
 * \file   battery_display.h
 * \date   July 15, 2011
 * \author Matt Richard
 */
#ifndef CONTROL_PANEL_BATTERY_DISPLAY_H
#define CONTROL_PANEL_BATTERY_DISPLAY_H

#include <QGraphicsView>

QT_BEGIN_NAMESPACE
class QGraphicsPixmapItem;
class QGraphicsRectItem;
class QGraphicsScene;
class QGraphicsTextItem;
QT_END_NAMESPACE

/**
 * \class BatteryDisplay
 * \brief Graphical battery display for indicating a robots battery level
 */
class BatteryDisplay : public QGraphicsView
{
	Q_OBJECT

	public:
		/**
		 * \brief Contruct. Sets up the display
		 *
		 * \param parent The parent widget
		 */
		BatteryDisplay(QWidget *parent = 0);

		/**
		 * \brief Returns the current battery level
		 */
		int getBatteryLevel() const { return battery_level; }

		/**
		 * \brief Sets the current battery level and updates the graphics display.
		 *
		 * \param level New battery level.
		 */
		void setBatteryLevel(float level);

	private:
		QGraphicsScene *scene;
		QGraphicsRectItem *battery_level_item;
		QGraphicsPixmapItem *battery_background_item;
		QGraphicsPixmapItem *battery_foreground_item;
		QGraphicsTextItem *battery_text_item;
		float battery_level;
};

#endif // CONTROL_PANEL_BATTERY_DISPLAY_H
