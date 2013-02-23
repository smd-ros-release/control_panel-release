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
 * \file   display_pane.cpp
 * \date   Nov 9, 2011
 * \author Matt Richard
 */
#include <QtGui>
#include "control_panel/display_pane.h"
#include "math.h"


DisplayPane::DisplayPane(QWidget *parent)
    : QWidget(parent)
{
    image_off = true;
	source_selection_list << "Off";

    QFont font;
    font.setPointSize(18);

    title_label = new QLabel;
    title_label->setFont(font);


	image_viewer = new ImageViewer;
	image_viewer->setAlignment(Qt::AlignCenter);
	image_viewer->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
	image_viewer->setFrameStyle(QFrame::Panel | QFrame::Sunken);
	image_viewer->setMinimumSize(0, 240);


    source_label = new QLabel(tr("Source: "));
    source_label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    source_selector = new QComboBox;
	source_selector->setSizeAdjustPolicy(QComboBox::AdjustToContents);

	scale_slider = new QSlider(Qt::Horizontal);
	scale_slider->setRange(25, 400);
//	scale_slider->setSingleStep(25);
	scale_slider->setTickPosition(QSlider::TicksBelow);
	scale_slider->setTickInterval(25);

	grid_checkbox = new QCheckBox(tr("Grid"));


    connect(source_selector, SIGNAL(currentIndexChanged(int)),
		this, SLOT(sourceChanged(int)));
	connect(scale_slider, SIGNAL(valueChanged(int)),
		image_viewer, SLOT(setScale(int)));
	connect(image_viewer, SIGNAL(scaleChanged(int)),
		scale_slider, SLOT(setValue(int)));
    connect(grid_checkbox, SIGNAL(toggled(bool)),
        image_viewer, SLOT(showGrid(bool)));

    // Create video pane layout
    QHBoxLayout *header_hlayout = new QHBoxLayout;
    header_hlayout->addWidget(title_label, 0, Qt::AlignLeft);
    header_hlayout->addStretch();
    header_hlayout->addWidget(source_label, 0, Qt::AlignRight);
    header_hlayout->addWidget(source_selector, 0, Qt::AlignLeft);


	QHBoxLayout *footer_hlayout = new QHBoxLayout;
	footer_hlayout->addWidget(scale_slider);
	footer_hlayout->addWidget(grid_checkbox, 0, Qt::AlignRight);


    display_pane_layout = new QVBoxLayout;
    display_pane_layout->addLayout(header_hlayout);
	display_pane_layout->addWidget(image_viewer);
	display_pane_layout->addLayout(footer_hlayout);

    setLayout(display_pane_layout);
    setFocusPolicy(Qt::ClickFocus);
}

/******************************************************************************
** Function:    addSource
** Author:      Matt Richard
** Parameters:  char *new_source - source string to add
** Returns:     void
** Description:
******************************************************************************/
void DisplayPane::addSource(const QString &new_source)
{
	source_selection_list << new_source;
}

/******************************************************************************
** Function:    addSourceList
** Author:      Matt Richard
** Parameters:  QStringList source_list - list of source strings to add
** Returns:     void
** Description:
******************************************************************************/
void DisplayPane::addSourceList(const QStringList &source_list)
{
	source_selection_list << source_list;
}

/******************************************************************************
** Function:    getTitle
** Author:      Matt Richard
** Parameters:  None
** Returns:     std::string - the display pane's title
** Description:
******************************************************************************/
std::string DisplayPane::getTitle() const
{
	return title.toStdString();
}

/******************************************************************************
** Function:    getImage
** Author:      Matt Richard
** Parameters:  None
** Returns:     QImage - current image
** Description:
******************************************************************************/
QImage DisplayPane::getImage() const
{
	return image;
}

/******************************************************************************
** Function:    getCurrentSource
** Author:      Matt Richard
** Parameters:  None
** Returns:     std::string - the current source string
** Description:
******************************************************************************/
std::string DisplayPane::getCurrentSource() const
{
	return source_selector->currentText().toStdString();
}

/******************************************************************************
** Function:
** Author:
** Parameters:
** Returns:
** Description:
******************************************************************************/
bool DisplayPane::isImageOff() const
{
	return image_off;
}

/******************************************************************************
** Function:    connectionStatusChanged
** Author:      Matt Richard
** Parameters:  int new_status - the new status
** Returns:     void
** Description: 
******************************************************************************/
void DisplayPane::connectionStatusChanged(int new_status)
{
    if(new_status == Globals::Connected)
    {
        source_selector->addItems(source_selection_list);

		image_off = false;
    }
    else if(new_status == Globals::Connecting) {}
    else if(new_status == Globals::Disconnected)
        source_selector->clear(); // clear source combobox
    else
        printf("Display pane received unknown status '%d'.\n", new_status);
}

/******************************************************************************
** Function:    setImage
** Author:      Matt Richard
** Parameters:  QImage new_image - the new image to display
** Returns:     void
** Description:
******************************************************************************/
void DisplayPane::setImage(const QImage &new_image, int grid_interval)
{
	// Check if the image display is off
	if(!image_off)
		image_viewer->setImagePixmap(QPixmap::fromImage(new_image),
			grid_interval);
}

void DisplayPane::setMap(const QImage &new_image, double origin_x,
	double origin_y, float resolution)
{
	if(!image_off)
		image_viewer->setMapPixmap(QPixmap::fromImage(new_image),
			abs(origin_x) / resolution, abs(origin_y) / resolution, (int)(1.0 / resolution));
}

/******************************************************************************
** Function:    setTitle
** Author:      Matt Richard
** Parameters:  char *new_title - the new display pane title
** Returns:     void
** Description:
******************************************************************************/
void DisplayPane::setTitle(const char *new_title)
{
	title = new_title;

	title_label->setText(title);
}

/******************************************************************************
** Function:    updateVideoDisplay
** Author:      Matt Richard
** Parameters:  int new_view - 
** Returns:     void
** Description: 
******************************************************************************/
void DisplayPane::sourceChanged(int index)
{
    if(index <= 0)
	{
        image_off = true;
		scale_slider->setValue(100);
		grid_checkbox->setChecked(false);
	}
    else
		image_off = false;

	image_viewer->setImageVisible(!image_off);
	scale_slider->setEnabled(!image_off);
	grid_checkbox->setEnabled(!image_off);

    emit changeSource(source_selector->currentText().toStdString());
}

void DisplayPane::showPosition(bool show)
{
    image_viewer->showOdometry(show);
}

void DisplayPane::setPosition(const QVector3D &pos)
{
    image_viewer->setRobotPosition(pos.x(), pos.y());
}
