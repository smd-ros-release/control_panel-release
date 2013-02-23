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
 * \file   artificial_horizon.h
 * \date   July, 2011
 * \author Matt Richard
 */
#ifndef CONTROL_PANEL_ARTIFICIAL_HORIZON_H
#define CONTROL_PANEL_ARTIFICIAL_HORIZON_H

#include <QGraphicsPixmapItem>

/**
 * \class ArtificialHorizon
 * \brief An artificial horizon graphic that is used in the attitude indicator.
 */
class ArtificialHorizon : public QGraphicsPixmapItem
{
	public:
		/**
		 * \brief Constructor. Loads and sets the pixmap for the graphics item.
		 */
		ArtificialHorizon();

		/**
		 * \brief Overloaded function that returns the item's bounding rectangle.
		 */
		QRectF boundingRect() const;

		/**
		 * \brief Overloaded function that clips the item's shape to a circle.
		 */
		QPainterPath shape() const;

		/**
		 * \brief Updates roll and pitch and then translates and rotates the graphics item accordingly.
		 *
		 * \param roll_angle  The new roll in degrees.
		 * \param pitch_angle The new pitch in degrees.
		 */
		void updateArtificialHorizon(int roll_angle, int pitch_angle);

		/**
		 * \brief Returns the current pitch.
		 */
		int getPitch() const { return pitch; }

		/**
		 * \brief Returns the current roll.
		 */
		int getRoll() const { return roll; }

	private:
		float center_x; // x-axis center of loaded pixmap
		float center_y; // y-axis center of loaded pixmap
		int offset;     // Pixmap offset
		int scene_width;
		int scene_height;

		int roll;
		int pitch;
};

#endif // CONTROL_PANEL_ARTIFICIAL_HORIZON_H
