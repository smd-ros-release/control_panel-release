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
 * \file   imu_display.cpp
 * \date   Jan 19, 2012
 * \author Matt Richard
 */
#include <QtGui>
#include "control_panel/widgets/imu_display.h"

ImuDisplay::ImuDisplay(QWidget *parent) : QWidget(parent)
{
    imu_name = "IMU";

    use_roll = true;
    use_pitch = true;
    use_yaw = true;
    use_ang_vel = true;
    use_lin_accel = true;
    use_attitude_ind = true;
    use_heading_ind = true;

    createWidget();
}

ImuDisplay::ImuDisplay(const QString &name, bool show_roll, bool show_pitch,
    bool show_yaw, bool show_ang_vel, bool show_lin_accel, bool show_attitude,
    bool show_heading, QWidget *parent) : QWidget(parent)
{
    if(name == "")
        imu_name = "IMU";
    else
        imu_name = name;

    use_roll = show_roll;
    use_pitch = show_pitch;
    use_yaw = show_yaw;
    use_ang_vel = show_ang_vel;
    use_lin_accel = show_lin_accel;
    use_attitude_ind = show_attitude;
    use_heading_ind = show_heading;

    createWidget();
}

void ImuDisplay::createWidget()
{
    int rows = 0;

    // Zero all values
    zeroValues();

    widget_layout = new QHBoxLayout;

    // Create attitude widget
    if(use_attitude_ind)
    {
        attitude = new AttitudeIndicator;
        //widget_layout->addStretch();
        widget_layout->addWidget(attitude);//, 0, Qt::AlignLeft);
    }

    // Create heading widget
    if(use_heading_ind)
    {
        heading = new HeadingIndicator;
        widget_layout->addStretch();
        widget_layout->addWidget(heading);
    }

    // use a grid layout for the labels
    QGridLayout *data_gridlayout = new QGridLayout;
    data_gridlayout->setSpacing(0);

    // Create header if any labels will be displayed
    if(use_roll || use_pitch || use_yaw || use_ang_vel || use_lin_accel)
    {
        name_label = new QLabel(imu_name);
        
        data_gridlayout->addWidget(name_label, 0, 0, 1, 0, Qt::AlignHCenter);
        rows++;

        data_gridlayout->setColumnMinimumWidth(1, 100);
    }

    // Create roll labels
    if(use_roll)
    {
        QLabel *roll_str = new QLabel("Roll: ");
        roll_label = new QLabel;

        data_gridlayout->addWidget(roll_str, rows, 0, Qt::AlignLeft);
        data_gridlayout->addWidget(roll_label, rows, 1, Qt::AlignRight);
        rows++;
    }

    // Create pitch labels
    if(use_pitch)
    {
        QLabel *pitch_str = new QLabel("Pitch: ");
        pitch_label = new QLabel;

        data_gridlayout->addWidget(pitch_str, rows, 0, Qt::AlignLeft);
        data_gridlayout->addWidget(pitch_label, rows, 1, Qt::AlignRight);
        rows++;
    }

    // Create yaw labels
    if(use_yaw)
    {
        QLabel *yaw_str = new QLabel("Yaw: ");
        yaw_label = new QLabel;

        data_gridlayout->addWidget(yaw_str, rows, 0, Qt::AlignLeft);
        data_gridlayout->addWidget(yaw_label, rows, 1, Qt::AlignRight);
        rows++;
    }

    // Create angular velocity labels and add to grid layout
    if(use_ang_vel)
    {
        QLabel *ang_vel_str = new QLabel("Ang Vel: ");
        ang_vel_label = new QLabel;

        data_gridlayout->addWidget(ang_vel_str, rows, 0, Qt::AlignLeft);
        data_gridlayout->addWidget(ang_vel_label, rows, 1, Qt::AlignRight);
        rows++;
    }

    // Create linear acceleration labels and add to grid layout
    if(use_lin_accel)
    {
        QLabel *lin_accel_str = new QLabel("Lin Accel: ");
        lin_accel_label = new QLabel;

        data_gridlayout->addWidget(lin_accel_str, rows, 0, Qt::AlignLeft);
        data_gridlayout->addWidget(lin_accel_label, rows, 1, Qt::AlignRight);
        rows++;
    }

    // Add the grid layout to the final layout
    widget_layout->addStretch();
    widget_layout->addLayout(data_gridlayout);
    widget_layout->addStretch();


    // set the layout
    setLayout(widget_layout);
}

void ImuDisplay::zeroValues()
{
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;

    angular_velocity.setX(0.0);
    angular_velocity.setY(0.0);
    angular_velocity.setZ(0.0);

    linear_acceleration.setX(0.0);
    linear_acceleration.setY(0.0);
    linear_acceleration.setZ(0.0);
}

/*
void ImuDisplay::updateImuDisplay(const QQuaternion &orientation, double ori_covar[9],
    const QVector3D &ang_vel, double ang_vel_covar[9],
    const QVector3D &lin_accel, double lin_accel_covar[9])
{

}
*/

void ImuDisplay::updateImuDisplay(const QQuaternion &orientation)
{
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
    if(use_roll)
        roll_label->setText(QString("%1").arg((int)roll, 4, 10) + Globals::DegreesSymbol);
        
    if(use_pitch)
        pitch_label->setText(QString("%1").arg((int)pitch, 4, 10) + Globals::DegreesSymbol);
    
    if(use_yaw)
        yaw_label->setText(QString("%1").arg((int)yaw, 4, 10) + Globals::DegreesSymbol);


    // Update the attitude indicator
    if(use_attitude_ind)
        attitude->setAttitude(roll, pitch);

    // Update the heading indicator
    if(use_heading_ind)
        heading->setYaw(yaw);
}

void ImuDisplay::updateImuDisplay(const QQuaternion &orientation,
    const QVector3D &ang_vel, const QVector3D &lin_accel)
{
    updateImuDisplay(orientation);

    angular_velocity = ang_vel;
    linear_acceleration = lin_accel;

    // Display angular velocity
    if(use_ang_vel)
        ang_vel_label->setText(QString("%1").arg(ang_vel.length(), 6, 'f', 1) + QString(" rad/s"));

    // Display linear acceleratrion
    if(use_lin_accel)
        lin_accel_label->setText(QString("%1").arg(lin_accel.length(), 6, 'f', 1) + QString(" m/s^2"));
}
