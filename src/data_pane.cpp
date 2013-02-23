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
 * \file   data_pane.cpp
 * \date   June 2011
 * \author Matt Richard
 */
#include <QtGui>
#include "control_panel/data_pane.h"

DataPane::DataPane(QWidget *parent) : QWidget(parent)
{
    gps_list = new QList<GpsDisplay *>;
    imu_list = new QList<ImuDisplay *>;
    joint_state_list = new QList<JointStateDisplay *>;
    odom_list = new QList<OdometryDisplay *>;

    use_range = false;

    createLabels();
    createLayout();

    setLayout(data_pane_layout);
    setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
    setFocusPolicy(Qt::ClickFocus);
}

void DataPane::createLabels()
{
    status_light = new QImage(":/images/status_lights/status_light_grey.png");

    status_light_label = new QLabel;
    status_light_label->setBackgroundRole(QPalette::Base);
    status_light_label->setPixmap(QPixmap::fromImage(status_light->scaled(QSize(30, 30),
                                  Qt::KeepAspectRatio, Qt::SmoothTransformation)));
    status_light_label->setAlignment(Qt::AlignCenter);


    QFont font;
    font.setPointSize(18);

    connection_status_label = new QLabel(tr("Disconnected"));
    connection_status_label->setFont(font);
    connection_status_label->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);


    rc_mode_label = new QLabel("RC Mode: Disabled");

    range_label = new QLabel("");
    //battery_status_label = new QLabel;

/*
    speed_label = new QLabel;
    turn_label = new QLabel;
    slip_label = new QLabel;
    slide_label = new QLabel;

    radiation_label = new QLabel;
    temp_label = new QLabel;
    light_label = new QLabel;
    pressure_label = new QLabel;
*/
}

void DataPane::createLayout()
{
    // Connection column layout
    QHBoxLayout *connection_hlayout = new QHBoxLayout;
    connection_hlayout->addWidget(status_light_label, 0, Qt::AlignLeft);
    connection_hlayout->addWidget(connection_status_label, 0, Qt::AlignLeft);

    //takeoff_land_button = new QPushButton("Takeoff");
    //connect(takeoff_land_button, SIGNAL(clicked()),
    //    this, SLOT(takeoffLandButtonClicked()));

    battery_display = new BatteryDisplay();
    battery_display->setVisible(false);

    QVBoxLayout *connection_vlayout = new QVBoxLayout;
    connection_vlayout->addLayout(connection_hlayout);
    connection_vlayout->addWidget(rc_mode_label);
    connection_vlayout->addStretch();
    connection_vlayout->addWidget(range_label);
//    connection_vlayout->addWidget(takeoff_land_button, 0, Qt::AlignLeft);
    connection_vlayout->addStretch();
    connection_vlayout->addWidget(battery_display, 0, Qt::AlignLeft);
    connection_vlayout->setAlignment(Qt::AlignLeft);

/*
    // Speed, turn, slip, and slide column layout
    QVBoxLayout *speed_vlayout = new QVBoxLayout;
    speed_vlayout->addWidget(speed_label);
    speed_vlayout->addWidget(turn_label);
    speed_vlayout->addWidget(slip_label);
    speed_vlayout->addWidget(slide_label);
    speed_vlayout->addStretch();


    // GPS, radiation, temperature, light, and pressure column layout
    QVBoxLayout *gps_vlayout = new QVBoxLayout;
    gps_vlayout->addWidget(gps_label);
    gps_vlayout->addWidget(radiation_label);
    gps_vlayout->addWidget(temp_label);
    gps_vlayout->addWidget(light_label);
    gps_vlayout->addWidget(pressure_label);
    gps_vlayout->addStretch();
*/

    // Data pane final layout
    data_pane_layout = new QHBoxLayout;
    data_pane_layout->addLayout(connection_vlayout);
    //data_pane_layout->addStretch(1);
}

void DataPane::showBatteryDisplay(bool show)
{
    battery_display->setVisible(show);
}

/******************************************************************************
**                                   SLOTS
******************************************************************************/

void DataPane::takeoffLandButtonClicked()
{
    if(takeoff_land_button->text() == "Takeoff")
    {
        takeoff_land_button->setText("Land");
        emit takeoff();
    }
    else // land
    {
        takeoff_land_button->setText("Takeoff");
        emit land();
    }
}


/******************************************************************************
** Function:    connectionStatusChanged
** Author:      Matt Richard
** Parameters:  int new_status -
** Returns:     void
** Description: 
******************************************************************************/
void DataPane::connectionStatusChanged(int new_status)
{
    if(new_status == Globals::Connected)
    {
        // Load green status light image
        status_light->load(":/images/status_lights/status_light_green.png");
        status_light_label->setPixmap(
            QPixmap::fromImage(status_light->scaled(QSize(30, 30),
            Qt::KeepAspectRatio, Qt::SmoothTransformation)));

        connection_status_label->setText(tr("Connected    "));
    }
    else if(new_status == Globals::Connecting)
    {
        status_light->load(":/image/status_lights/status_light_yellow.png");
        status_light_label->setPixmap(QPixmap::fromImage(status_light->scaled(
            QSize(30, 30), Qt::KeepAspectRatio, Qt::SmoothTransformation)));

        connection_status_label->setText(tr("Connecting  "));
    }
    else if(new_status == Globals::Disconnected)
    {
        // Load grey status light image
        status_light->load(":/images/status_lights/status_light_grey.png");
        status_light_label->setPixmap(
            QPixmap::fromImage(status_light->scaled(QSize(30, 30),
            Qt::KeepAspectRatio, Qt::SmoothTransformation)));

        connection_status_label->setText(tr("Disconnected"));

        // Reset labels
        //initializeLabels();

        // Reset graphical displays
        //battery_display->setBatteryLevel(0);
        //attitude_indicator->setAttitude(0, 0);
        //heading_indicator->setYaw(0);
    }
    else
        printf("ERROR -- DataPane class received an unknown new status '%d'.\n", new_status);
}

void DataPane::updateBatteryData(float battery_data)
{
    battery_display->setBatteryLevel(battery_data);

    //battery_status_label->setText(tr("Battery: %1%").arg(battery_data));
}

GpsDisplay *DataPane::addGpsDisplay(const QString &name, bool lat, bool lon,
    bool alt)
{
    gps_display = new GpsDisplay(name, lat, lon, alt);

    data_pane_layout->addWidget(gps_display);

    gps_list->append(gps_display);

    return gps_display;
}

ImuDisplay *DataPane::addImuDisplay(const QString &name, bool roll,
    bool pitch, bool yaw, bool ang_vel, bool lin_accel, bool attitude_graphic,
    bool heading_graphic)
{
    imu_display = new ImuDisplay(name, roll, pitch, yaw, ang_vel, lin_accel,
        attitude_graphic, heading_graphic);

    data_pane_layout->addWidget(imu_display);

    imu_list->append(imu_display);

    return imu_display;
}

JointStateDisplay *DataPane::addJointStateDisplay(const QString &widget_name,
    const QStringList &names, const QStringList &display_names, bool show_pos,
    bool show_vel, bool show_eff)
{
    joint_state_display = new JointStateDisplay(widget_name, names,
        display_names, show_pos, show_vel, show_eff);

    data_pane_layout->addWidget(joint_state_display);

    joint_state_list->append(joint_state_display);

    return joint_state_display;
}

OdometryDisplay *DataPane::addOdometryDisplay(const QString &name, bool pos,
    bool rpy, bool lin_vel, bool ang_vel, bool attitude_graphic,
    bool heading_graphic)
{
    odom_display = new OdometryDisplay(name, pos, rpy, lin_vel, ang_vel,
        attitude_graphic, heading_graphic);

    data_pane_layout->addWidget(odom_display);

    odom_list->append(odom_display);

    return odom_display;
}


void DataPane::setRCModeText(const QString &mode)
{
    rc_mode_label->setText(QString("RC Mode: ") + mode);
}

void DataPane::showRangeLabel(bool show)
{
    range_label->setText("");

    use_range = show;

    if(use_range)
        range_label->setText("Height:");
}

void DataPane::updateRange(float range)
{
    if(use_range)
        range_label->setText(QString("Height: %1 m").arg(range, 0, 'g', 3));
}
