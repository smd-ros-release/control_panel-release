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
 * \file   display_pane.h
 * \date   Nov 9, 2011
 * \author Matt Richard
 */
#ifndef CONTROL_PANEL_DISPLAY_PANE_H
#define CONTROL_PANEL_DISPLAY_PANE_H

#include <QWidget>

QT_BEGIN_NAMESPACE
class QCheckBox;
class QComboBox;
class QImage;
class QLabel;
class QSlider;
class QString;
class QStringList;
class QVBoxLayout;
QT_END_NAMESPACE

#include <string>
#include "widgets/image_viewer.h"
#include "globals.h"


/**
 * \class DisplayPane
 * \brief 
 */
class DisplayPane : public QWidget
{
    Q_OBJECT

    public:
        DisplayPane(QWidget *parent = 0);
		void addSource(const QString &new_source);
		void addSourceList(const QStringList &source_list);
		std::string getTitle() const;
		QImage getImage() const;
		std::string getCurrentSource() const;
		bool isImageOff() const;
		void setTitle(const char *new_title);

    public slots:
        void connectionStatusChanged(int new_status);
		void setImage(const QImage &new_image, int grid_interval = -1);
		void setMap(const QImage &new_image, double origin_x,
			double origin_y, float resolution = -1.0);
        void sourceChanged(int index);
        void showPosition(bool show);
        void setPosition(const QVector3D &pos);

    signals:
        void changeSource(const std::string source);

    private:
		bool image_off;
        QVBoxLayout *display_pane_layout;

        QLabel *title_label;
        QLabel *source_label;

        QComboBox *source_selector;
        QStringList source_selection_list;

		QImage image;
		QString title;

		ImageViewer *image_viewer;

		QSlider *scale_slider;

		QCheckBox *grid_checkbox;
};

#endif // CONTROL_PANEL_DISPLAY_PANE_H
