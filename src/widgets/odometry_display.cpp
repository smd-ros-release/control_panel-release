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
 * \file   odometry_display.cpp
 * \date   Jan 26, 2012
 * \author Matt Richard
 * \brief  Displays odometry information (position, orientation, linear velocity, and angular velocity).
 */
#include <QtGui>
#include "control_panel/widgets/odometry_display.h"

OdometryDisplay::OdometryDisplay(QWidget *parent) : QWidget(parent)
{
    use_pos = true;
    use_rpy = true;
    use_lin_vel = true;
    use_ang_vel = true;
    use_attitude_ind = true;
    use_heading_ind = true;

    createWidget();
}

OdometryDisplay::OdometryDisplay(const QString &name, bool show_pos, bool show_rpy,
    bool show_lin_vel, bool show_ang_vel, bool show_attitude,
    bool show_heading, QWidget *parent)
    : QWidget(parent)
{
    if(name == "")
        odometry_name = "Odometry";
    else
        odometry_name = name;

    use_pos = show_pos;
    use_rpy = show_rpy;
    use_lin_vel = show_lin_vel;
    use_ang_vel = show_ang_vel;
    use_attitude_ind = show_attitude;
    use_heading_ind = show_heading;

    createWidget();
}

void OdometryDisplay::createWidget()
{
    int curr_row = 0;

    // Initialize private variables
    zeroValues();

    widget_layout = new QHBoxLayout;

    if(use_attitude_ind)
    {
        attitude = new AttitudeIndicator;
        widget_layout->addStretch();
        widget_layout->addWidget(attitude);
    }

    if(use_heading_ind)
    {
        heading = new HeadingIndicator;
        widget_layout->addStretch();
        widget_layout->addWidget(heading);
    }

    // Grid layout for the labels
    QGridLayout *data_gridlayout = new QGridLayout;
    data_gridlayout->setSpacing(0);

    // Check if any labels will be displayed
    if(use_pos || use_rpy || use_lin_vel || use_ang_vel)
    {
        name_label = new QLabel(odometry_name);

        data_gridlayout->addWidget(name_label, curr_row, 0, 1, 0, Qt::AlignHCenter);
        curr_row++;

        data_gridlayout->setColumnMinimumWidth(1, 150);
    }

    // Create postition labels and add to the grid layout
    if(use_pos)
    {
        QLabel *pos_str = new QLabel("Position: ");
        position_label = new QLabel;

        data_gridlayout->addWidget(pos_str, curr_row, 0, Qt::AlignLeft);
        data_gridlayout->addWidget(position_label, curr_row, 1, Qt::AlignRight);
        curr_row++;
    }

    // Create roll, pitch, and yaw labels and add to the grid layout
    if(use_rpy)
    {
        QLabel *roll_str = new QLabel("Roll: ");
        QLabel *pitch_str = new QLabel("Pitch: ");
        QLabel *yaw_str = new QLabel("Yaw: ");
        roll_label = new QLabel;
        pitch_label = new QLabel;
        yaw_label = new QLabel;

        data_gridlayout->addWidget(roll_str, curr_row, 0, Qt::AlignLeft);
        data_gridlayout->addWidget(roll_label, curr_row, 1, Qt::AlignRight);
        curr_row++;
        data_gridlayout->addWidget(pitch_str, curr_row, 0, Qt::AlignLeft);
        data_gridlayout->addWidget(pitch_label, curr_row, 1, Qt::AlignRight);
        curr_row++;
        data_gridlayout->addWidget(yaw_str, curr_row, 0, Qt::AlignLeft);
        data_gridlayout->addWidget(yaw_label, curr_row, 1, Qt::AlignRight);
        curr_row++;
    }

    // Create linear velocity labels and add to grid layout
    if(use_lin_vel)
    {
        QLabel *lin_vel_str = new QLabel("Linear: ");
        lin_vel_label = new QLabel;

        data_gridlayout->addWidget(lin_vel_str, curr_row, 0, Qt::AlignLeft);
        data_gridlayout->addWidget(lin_vel_label, curr_row, 1, Qt::AlignRight);
        curr_row++;
    }

    // Create angular velocity labels and add to grid layout
    if(use_ang_vel)
    {
        QLabel *ang_vel_str = new QLabel("Angular: ");
        ang_vel_label = new QLabel;

        data_gridlayout->addWidget(ang_vel_str, curr_row, 0, Qt::AlignLeft);
        data_gridlayout->addWidget(ang_vel_label, curr_row, 1, Qt::AlignRight);
        curr_row++;
    }

    // Add the grid layout to the final layout
    widget_layout->addStretch();
    widget_layout->addLayout(data_gridlayout);
    widget_layout->addStretch();

    // Set the final layout
    setLayout(widget_layout);
}

void OdometryDisplay::zeroValues()
{
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;

    pos.setX(0);
    pos.setY(0);
    pos.setZ(0);

    lin_vel.setX(0);
    lin_vel.setY(0);
    lin_vel.setZ(0);

    ang_vel.setX(0);
    ang_vel.setY(0);
    ang_vel.setZ(0);
}

void OdometryDisplay::updateOdometryDisplay(const QVector3D &position,
    const QQuaternion &orientation, const QVector3D &linear_velocity,
    const QVector3D &angular_velocity)
{
    // Update and display postition
    pos = position;
    if(use_pos)
        position_label->setText(QString("( %1, %2, %3)").arg(pos.x(), 6, 'f', 1).arg(
            pos.y(), 6, 'f', 1).arg(pos.z(), 6, 'f', 1));


    double x = orientation.x();
    double y = orientation.y();
    double z = orientation.z();
    double w = orientation.scalar();

    // Calculate roll, pitch, and yaw from the quaternion
    roll = atan2(2.0 * (y*z + w*x), w*w - x*x - y*y + z*z) * Globals::RAD_TO_DEG;
    //roll = atan2(2.0 * (w*x + y*z), 1.0 - 2.0 * (x*x + y*y)) * Globals::RAD_TO_DEG;
    pitch = asin(-2.0 * (x*z - w*y)) * Globals::RAD_TO_DEG;
    //pitch = asin(2.0 * (w*y - z*x)) * Globals::RAD_TO_DEG;
    yaw = atan2(2.0 * (x*y + w*z), w*w + x*x - y*y - z*z) * Globals::RAD_TO_DEG;
    //yaw = atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z)) * Globals::RAD_TO_DEG;

    // Display roll, pitch, and yaw in integer value
    if(use_rpy)
    {
        roll_label->setText(QString("%1").arg((int)roll, 4, 10) + Globals::DegreesSymbol);
        pitch_label->setText(QString("%1").arg((int)pitch, 4, 10) + Globals::DegreesSymbol);
        yaw_label->setText(QString("%1").arg((int)yaw, 4, 10) + Globals::DegreesSymbol);
    }

    // Update the attitude indicator
    if(use_attitude_ind)
        attitude->setAttitude(roll, pitch);

    // Update the heading indicator
    if(use_heading_ind)
        heading->setYaw(yaw);


    lin_vel = linear_velocity;
    if(use_lin_vel)
        lin_vel_label->setText(QString("%1").arg(lin_vel.length(), 6, 'f', 1) + " m/s");


    ang_vel = angular_velocity;
    if(use_ang_vel)
        ang_vel_label->setText(QString("%1").arg(ang_vel.length(), 6, 'f', 1) + " rad/s");
}
