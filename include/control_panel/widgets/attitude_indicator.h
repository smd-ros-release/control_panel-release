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
 * \file   attitude_indicator.h
 * \date   July 15, 2011
 * \author Matt Richard
 */
#ifndef CONTROL_PANEL_ATTITUDE_INDICATOR_H
#define CONTROL_PANEL_ATTITUDE_INDICATOR_H

#include <QGraphicsView>

QT_BEGIN_NAMESPACE
class QGraphicsScene;
class QGraphicsPixmapItem;
QT_END_NAMESPACE

#include "artificial_horizon.h"

/**
 * \class AttitudeIndicator
 * \brief A graphical display of a robot's roll and pitch.
 */
class AttitudeIndicator : public QGraphicsView
{
	Q_OBJECT

	public:
		/**
		 * \brief Contructor. Creates the scene and view.
		 *
		 * \param parent The parent widget.
		 */
		AttitudeIndicator(QWidget *parent = 0);

		/**
		 * \brief Returns the current roll
		 */
		double getRoll() const { return roll; }

		/**
		 * \brief Returns the current pitch
		 */
		double getPitch() const { return pitch; }

		/**
		 * \brief Sets the roll and updates the display.
		 *
		 * \param angle New roll angle in degrees.
		 */
		void setRoll(double angle);

		/**
		 * \brief Sets the pitch and updates the display.
		 *
		 * \param angle New pitch angle in degrees.
		 */
		void setPitch(double angle);

		/**
		 * \brief Sets both the roll and pitch, then updates the display with these new values
		 *
		 * \param roll_angle  New roll angle in degrees.
		 * \param pitch_angle New pitch angle in degrees.
		 */
		void setAttitude(double roll_angle, double pitch_angle);

	private:
		/**
		 * \brief Updates the artificial horizon with the current values of roll and pitch.
		 */
		void updateAttitudeIndicator();

		QGraphicsScene *scene;
		QGraphicsPixmapItem *crosshair_item;
		QGraphicsPixmapItem *cover_item;
		ArtificialHorizon *artificial_horizon_item;

		double roll;
		double pitch;
};

#endif // CONTROL_PANEL_ATTITUDE_INDICATOR_H
