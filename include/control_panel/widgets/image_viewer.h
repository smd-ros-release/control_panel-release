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
 * \file   image_viewer.h
 * \date   Jan 4, 2012
 * \author Matt Richard
 */
#ifndef CONTROL_PANEL_IMAGE_VIEWER_H
#define CONTROL_PANEL_IMAGE_VIEWER_H

#include <QGraphicsView>
#include <QPixmap>

QT_BEGIN_NAMESPACE
class QGraphicsEllipseItem;
class QGraphicsPixmapItem;
class QGraphicsScene;
class QGraphicsTextItem;
class QPainter;
class QPen;
class QRectF;
class QWheelEvent;
QT_END_NAMESPACE

/**
 * \class ImageViewer
 * \brief A graphics view widget for displaying a robots video, LiDAR, and maps.
 *
 * The image viewer allows for a grip to be overlayed on top of the image,
 * zooming, and translating the image.
 */
class ImageViewer : public QGraphicsView
{
	Q_OBJECT

	public:
		/**
		 * \brief Contructor. Creates the items, scene, and graphics view.
		 *
		 * \param parent The parent widget.
		 */
		ImageViewer(QWidget *parent = 0);

		/**
		 * \brief Returns the current pixmap.
		 */
		QPixmap getImagePixmap() const { return image_pixmap; }

		/**
		 * \brief Returns true if the grid is visible, otherwise returns false
		 */
		bool gridVisible() const { return grid_visible; }

		/**
		 * \brief Sets the interval for which each grid line is drawn.
		 *
		 * \param pixels The number of pixels between each grid line. Default is 20 pixels.
		 */
		void setGridLineInterval(int pixels);

		/**
		 * \brief Sets the pixmap to be drawn, and an optional grid line interval.
		 *
		 * \param pixamp   The new pixmap to display.
		 * \param interval The pixel interval for which to draw each grid line.
		 *                 If interval is less than 0, the previous interval will be used.
		 */
		void setImagePixmap(const QPixmap &pixmap, int interval = -1);

		void setMapPixmap(const QPixmap &pixmap, double origin_x, double origin_y, int interval = -1);

		/**
		 * \brief Sets the view state of the image pixmap.
		 *
		 * \param visible If true, the image pixmap will be visible, otherwise the image will be hidden.
		 */
		void setImageVisible(bool visible);

		/**
		 * \brief Draws a red circle on the graphics scene at (x_pos, y_pos).
		 *
		 * This fuction is used for displaying a robots position on a map.
		 * x_pos and y_pos will be translated into the scenes coordinates.
		 *
		 * \param x_pos The position of the robot on the x-axis
		 * \param y_pos The position of the robot on the y-axis
		 */
		void setRobotPosition(double x_pos, double y_pos);

	public slots:
		/**
		 * \brief SLOT. Sets the scale of the scene. Used for zooming.
		 *
		 * \param factor The scale factor. Must be a value between 25 and 400, inclusive.
		 */
		void setScale(int factor);

		/**
		 * \brief SLOT. Sets whether the grid should be drawn or not.
		 *
		 * \param show If true, the grid will be drawn, otherwise the grid will not be drawn.
		 */
		void showGrid(bool show);

		/**
		 * \brief SLOT. Sets whether the position of a robot should be drawn on the scene or not.
		 *
		 * \param show If true, the postion of the robot will be display, otherwise the position will not be drawn.
		 */
		void showOdometry(bool show);

	signals:
		/**
		 * \brief SIGNAL. Emitted when the scale of the scene is changed.
		 *
		 * \param factor The new scale of the scene.
		 */
		void scaleChanged(int factor);

	protected:
		/**
		 * \brief Reimplemented from QGraphicsView. Draws the grid.
		 *
		 * If grid_visible is true, the grid lines will be drawn at every
		 * grid_interval pixels.
		 *
		 * \param painter The painter to draw the grid.
		 * \param rect    The viewable area of the scene in the viewport.
		 */
		void drawForeground(QPainter *painter, const QRectF &rect);

		/**
		 * \brief Reimplemented from QGraphicsView.
		 *
		 * Scales the scene according to how far the mouse wheel is scrolled.
		 *
		 * \param event The wheel event.
		 */
		void wheelEvent(QWheelEvent *event);

	private:
		QGraphicsScene *scene;
		QGraphicsPixmapItem *image_item;
		QGraphicsTextItem *text_item;
		QGraphicsEllipseItem *robot_pos_item;

		QPixmap image_pixmap;

		bool show_odom;
		bool grid_visible;
		int grid_interval;
		QPen grid_pen;
		float scale_factor;

		QPointF map_origin;
};

#endif // CONTROL_PANEL_IMAGE_VIEWER_H
