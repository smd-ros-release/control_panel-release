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
 * \file   odometry_display.h
 * \date   Jan 26,2012
 * \author Matt Richard
 */
#ifndef CONTROL_PANEL_ODOMETRY_DISPLAY_H
#define CONTROL_PANEL_ODOMETRY_DISPLAY_H

#include <QWidget>
#include <QQuaternion>
#include <QVector3D>
#include <QString>

QT_BEGIN_NAMESPACE
class QHBoxLayout;
class QLabel;
QT_END_NAMESPACE

#include "attitude_indicator.h"
#include "heading_indicator.h"
#include "control_panel/globals.h"


/**
 * \class OdometryDisplay
 * \brief Displays odometry information (position, orientation, linear velocity, and angular velocity).
 */
class OdometryDisplay : public QWidget
{
    Q_OBJECT

    public:
        OdometryDisplay(QWidget *parent = 0);
        OdometryDisplay(const QString &name, bool show_pos = true,
            bool show_rpy = true, bool show_lin_vel = true,
            bool show_ang_vel = true, bool show_attitude = true,
            bool show_heading = true, QWidget *parent = 0);
        void zeroValues();

    public slots:
        void updateOdometryDisplay(const QVector3D &position,
            const QQuaternion &orientation, const QVector3D &linear_velocity,
            const QVector3D &angular_velocity);

    private:
        void createWidget();

        QHBoxLayout *widget_layout;

        QString odometry_name;

        bool use_pos;
        bool use_rpy;
        bool use_lin_vel;
        bool use_ang_vel;
        bool use_heading_ind;
        bool use_attitude_ind;

        double roll;
        double pitch;
        double yaw;
        QVector3D pos;
        QVector3D lin_vel;
        QVector3D ang_vel;

        AttitudeIndicator *attitude;
        HeadingIndicator *heading;

        QLabel *name_label;
        QLabel *position_label;
        QLabel *roll_label;
        QLabel *pitch_label;
        QLabel *yaw_label;
        QLabel *lin_vel_label;
        QLabel *ang_vel_label;
};

#endif // CONTROL_PANEL_ODOMETRY_DISIPLAY_H
