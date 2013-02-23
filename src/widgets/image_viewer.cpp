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
 * \file   image_viewer.cpp
 * \date   Jan 4, 2012
 * \author Matt Richard
 * \brief  A graphics view widget for displaying a robots video, LiDAR, and maps.
 */
#include <QtGui>
#include "control_panel/widgets/image_viewer.h"

ImageViewer::ImageViewer(QWidget *parent) : QGraphicsView(parent)
{
	grid_visible = false;
	grid_interval = 20;
	grid_pen.setColor(Qt::blue);
	grid_pen.setWidth(1);
	scale_factor = 1.0;
	show_odom = false;

	image_item = new QGraphicsPixmapItem(image_pixmap);
	image_item->setShapeMode(QGraphicsPixmapItem::BoundingRectShape);

	/* Loading text item */
	QFont font;
	font.setPointSize(18);
	text_item = new QGraphicsTextItem(tr("Loading..."));
	text_item->setFont(font);
	text_item->setDefaultTextColor(Qt::white);
	text_item->setVisible(true);

	/* Position of a robot on the display map */
	robot_pos_item = new QGraphicsEllipseItem(QRectF(0, 0, 6, 6));
	robot_pos_item->setVisible(false);
	robot_pos_item->setBrush(QBrush(Qt::red));
	robot_pos_item->setPen(Qt::NoPen);

	/* Create the scene */
	scene = new QGraphicsScene(this);
	scene->setBackgroundBrush(Qt::black);
	scene->addItem(text_item);
	scene->addItem(image_item);
	scene->addItem(robot_pos_item);

	/* Set graphics view properties */
	setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
	setFrameStyle(QFrame::NoFrame | QFrame::Plain);
	setScene(scene);
	setDragMode(QGraphicsView::ScrollHandDrag);
	setResizeAnchor(QGraphicsView::AnchorViewCenter);
	setTransformationAnchor(QGraphicsView::AnchorViewCenter);
}

void ImageViewer::setGridLineInterval(int pixels)
{
	if(pixels > 0)
		grid_interval = pixels;
}

void ImageViewer::setImagePixmap(const QPixmap &pixmap, int interval)
{
	if(interval > 0)
		grid_interval = interval;

	image_pixmap = pixmap;
	image_item->setPixmap(image_pixmap);

	//scene->setSceneRect(scene->itemsBoundingRect());
	scene->setSceneRect(image_item->boundingRect());

	if(scale_factor == 1.0)
		centerOn(image_item);
}

void ImageViewer::setMapPixmap(const QPixmap &pixmap, double origin_x, double origin_y, int interval)
{
	map_origin = QPointF(origin_x, origin_y);

	setImagePixmap(pixmap, interval);
}

void ImageViewer::setImageVisible(bool visible)
{
	if(visible)
		image_pixmap.fill(Qt::transparent); // Transparent is used initially so the Loading text can be seen
	else
		image_pixmap.fill(Qt::black); // Hides the Loading text

	// Show the Loading text and center the item in the scene
	text_item->setVisible(visible);
	text_item->setPos((scene->width() - text_item->boundingRect().width()) / 2.0,
		(scene->height() - text_item->boundingRect().height()) / 2.0);

	image_item->setPixmap(image_pixmap);
	image_item->setVisible(visible);
}

void ImageViewer::setScale(int factor)
{
	scale_factor = (float)factor / 100.0;

	// this->scale( ) scales the scene from its previous scale, so reset the scale to 1.0
	resetMatrix();

	scale(scale_factor, scale_factor);
}

void ImageViewer::showGrid(bool show)
{
	grid_visible = show;
}

void ImageViewer::showOdometry(bool show)
{
	show_odom = show;
	if( !show )
		robot_pos_item->hide( );
	robot_pos_item->setPos(
		(scene->width() - robot_pos_item->boundingRect().width()) / 2.0,
		(scene->height() - robot_pos_item->boundingRect().height()) / 2.0);
}

void ImageViewer::setRobotPosition(double x_pos, double y_pos)
{
	if(show_odom)
	{
		// Find the center of the image compared to the center of the circle item (the robots position)
		//double centerx = (image_item->boundingRect().width() -
		//	robot_pos_item->boundingRect().width()) / 2.0;
		//double centery = (image_item->boundingRect().height() -
		//	robot_pos_item->boundingRect().height()) / 2.0;

		// Offset the circle item to indicate the robots position
		//robot_pos_item->setPos(centerx + (x_pos / 0.05), centery - (y_pos / 0.05));
		robot_pos_item->setPos(map_origin.x() + (x_pos / 0.05),
			image_item->boundingRect().height() - (map_origin.y() + (y_pos / 0.05)));
		robot_pos_item->show( );
	}
}

void ImageViewer::drawForeground(QPainter *painter, const QRectF &rect)
{
	if(grid_visible)
	{
		// Find the top left postion of the scene.
		double left = (int)rect.left() - ((int)rect.left() % grid_interval);
		double top = (int)rect.top() - ((int)rect.top() % grid_interval) +
			((int)scene->height() % grid_interval);

		// Create the vertical lines
		QVarLengthArray<QLineF, 100> lines_x;
		for(double x = left; x < rect.right(); x += grid_interval)
			lines_x.append(QLineF(x, rect.top(), x, rect.bottom() - 1));

		// Create the horizonal lines
		QVarLengthArray<QLineF, 100> lines_y;
		for(double y = top; y < rect.bottom(); y += grid_interval)
			lines_y.append(QLineF(rect.left(), y, rect.right(), y));

		// Draw lines
		painter->setPen(grid_pen);
		painter->drawLines(lines_x.data(), lines_x.size());
		painter->drawLines(lines_y.data(), lines_y.size());
	}
}

void ImageViewer::wheelEvent(QWheelEvent *event)
{
	scale_factor += (float)event->delta() / 1000.0;

	// Clamp the scale factor
	if(scale_factor < 0.25)
		scale_factor = 0.25;
	else if(scale_factor > 4.0)
		scale_factor = 4.0;

	emit scaleChanged((int)(scale_factor * 100));

	event->accept();
}
