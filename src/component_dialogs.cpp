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
 * \file   component_dialogs.cpp
 * \date   Jan 9, 2012
 * \author Matt Richard
 */
#include <QtGui>
#include "control_panel/component_dialogs.h"
#include "control_panel/globals.h"

///////////////////////////// Component Dialog ////////////////////////////
ComponentDialog::ComponentDialog(QWidget *parent) : QDialog(parent)
{
    this->createDialog();
}

ComponentDialog::ComponentDialog(const QString &name, const QString &topic_name,
    const QString &name_label_text, const QString &topic_name_label_text,
    QWidget *parent) : QDialog(parent)
{
    this->createDialog();

    name_label->setText(name_label_text);
    name_lineedit->setText(name);
    topic_name_label->setText(topic_name_label_text);
    topic_name_lineedit->setText(topic_name);
}

void ComponentDialog::createDialog()
{
    name_label = new QLabel(tr("Name"));
    name_lineedit = new QLineEdit;

    topic_name_label = new QLabel(tr("Topic Name"));
    topic_name_lineedit = new QLineEdit;

    button_box = new QDialogButtonBox(QDialogButtonBox::Cancel |
                                      QDialogButtonBox::Ok);

    connect(button_box, SIGNAL(accepted()), this, SLOT(accept()));
    connect(button_box, SIGNAL(rejected()), this, SLOT(reject()));

    // Create layout
    QGridLayout *dialog_layout = new QGridLayout;
    dialog_layout->addWidget(name_label, 0, 0);
    dialog_layout->addWidget(name_lineedit, 0, 1);
    dialog_layout->addWidget(topic_name_label, 1, 0);
    dialog_layout->addWidget(topic_name_lineedit, 1, 1);
    dialog_layout->addWidget(button_box, 2, 1);
    dialog_layout->setSizeConstraint(QLayout::SetFixedSize);
    setLayout(dialog_layout);
    setWindowTitle(tr("Add/Edit Component"));
}

//////////////////////////// Compass Dialog /////////////////////////////
CompassDialog::CompassDialog(QWidget *parent) : QDialog(parent)
{
    this->createDialog();
}

void CompassDialog::createDialog()
{
    QLabel *name_label = new QLabel(tr("Name"));
    name_lineedit = new QLineEdit;

    QLabel *topic_name_label = new QLabel(tr("Topic Name"));
    topic_name_lineedit = new QLineEdit;

    show_heading_checkbox = new QCheckBox(tr("Show Heading"));
    show_heading_checkbox->setChecked(true);
    show_label_checkbox = new QCheckBox(tr("Show Label"));
    show_label_checkbox->setChecked(true);

    button_box = new QDialogButtonBox(QDialogButtonBox::Cancel |
                                      QDialogButtonBox::Ok);
    connect(button_box, SIGNAL(accepted()), this, SLOT(accept()));
    connect(button_box, SIGNAL(rejected()), this, SLOT(reject()));

    // Checkbox group
    QVBoxLayout *compass_vlayout = new QVBoxLayout;
    compass_vlayout->addWidget(show_heading_checkbox);
    compass_vlayout->addWidget(show_label_checkbox);

    QGroupBox *checkbox_group = new QGroupBox(tr("Display Configuration"));
    checkbox_group->setLayout(compass_vlayout);

    // Compass dialog layout
    QGridLayout *dialog_layout = new QGridLayout;
    dialog_layout->addWidget(name_label, 0, 0);
    dialog_layout->addWidget(name_lineedit, 0, 1);
    dialog_layout->addWidget(topic_name_label, 1, 0);
    dialog_layout->addWidget(topic_name_lineedit, 1, 1);
    dialog_layout->addWidget(checkbox_group, 2, 0, 1, 2);
    dialog_layout->addWidget(button_box, 3, 1);
    dialog_layout->setSizeConstraint(QLayout::SetFixedSize);
    setLayout(dialog_layout);
    setWindowTitle(tr("Add/Edit Compass"));
}

/////////////////////////// Gps Dialog ///////////////////////////////
GpsDialog::GpsDialog(QWidget *parent) : QDialog(parent)
{
    this->createDialog();
}

void GpsDialog::createDialog()
{
    QLabel *name_label = new QLabel(tr("Name"));
    name_lineedit = new QLineEdit;

    QLabel *topic_name_label = new QLabel(tr("Topic Name"));
    topic_name_lineedit = new QLineEdit;

    lat_checkbox = new QCheckBox(tr("Latitude"));
    lat_checkbox->setChecked(true);
    long_checkbox = new QCheckBox(tr("Longitude"));
    long_checkbox->setChecked(true);
    alt_checkbox = new QCheckBox(tr("Alititude"));

    button_box = new QDialogButtonBox(QDialogButtonBox::Cancel |
                                      QDialogButtonBox::Ok);
    connect(button_box, SIGNAL(accepted()), this, SLOT(accept()));
    connect(button_box, SIGNAL(rejected()), this, SLOT(reject()));

    // Checkbox group
    QVBoxLayout *msg_vlayout = new QVBoxLayout;
    msg_vlayout->addWidget(lat_checkbox);
    msg_vlayout->addWidget(long_checkbox);
    msg_vlayout->addWidget(alt_checkbox);

    QGroupBox *checkbox_group = new QGroupBox(tr("NavSatFix Message"));
    checkbox_group->setLayout(msg_vlayout);

    // GPS dialog layout
    QGridLayout *dialog_layout = new QGridLayout;
    dialog_layout->addWidget(name_label, 0, 0);
    dialog_layout->addWidget(name_lineedit, 0, 1);
    dialog_layout->addWidget(topic_name_label, 1, 0);
    dialog_layout->addWidget(topic_name_lineedit, 1, 1);
    dialog_layout->addWidget(checkbox_group, 2, 0, 1, 2);
    dialog_layout->addWidget(button_box, 3, 1);
    dialog_layout->setSizeConstraint(QLayout::SetFixedSize);
    setLayout(dialog_layout);
    setWindowTitle(tr("Add/Edit GPS"));
}

/////////////////////////// Imu Dialog ///////////////////////////////
ImuDialog::ImuDialog(QWidget *parent) : QDialog(parent)
{
    this->createDialog();
}

void ImuDialog::createDialog()
{
    QLabel *name_label = new QLabel(tr("Name"));
    name_lineedit = new QLineEdit;

    QLabel *topic_name_label = new QLabel(tr("Topic Name"));
    topic_name_lineedit = new QLineEdit;

    roll_checkbox = new QCheckBox(tr("Roll"));
    roll_checkbox->setChecked(true);
    pitch_checkbox = new QCheckBox(tr("Pitch"));
    pitch_checkbox->setChecked(true);
    yaw_checkbox = new QCheckBox(tr("Yaw"));
    yaw_checkbox->setChecked(true);
    lin_accel_checkbox = new QCheckBox(tr("Linear Acceleration"));
    ang_vel_checkbox = new QCheckBox(tr("Angular Velocity"));

    show_att_checkbox = new QCheckBox(tr("Show Attitude Indicator"));
    show_heading_checkbox = new QCheckBox(tr("Show Heading Indicator"));
    show_labels_checkbox = new QCheckBox(tr("Show Labels"));
    show_labels_checkbox->setChecked(true);

    button_box = new QDialogButtonBox(QDialogButtonBox::Cancel |
                                      QDialogButtonBox::Ok);
    connect(button_box, SIGNAL(accepted()), this, SLOT(accept()));
    connect(button_box, SIGNAL(rejected()), this, SLOT(reject()));


    // IMU Message group
    QVBoxLayout *msg_vlayout = new QVBoxLayout;
    msg_vlayout->addWidget(roll_checkbox);
    msg_vlayout->addWidget(pitch_checkbox);
    msg_vlayout->addWidget(yaw_checkbox);
    msg_vlayout->addWidget(lin_accel_checkbox);
    msg_vlayout->addWidget(ang_vel_checkbox);

    QGroupBox *msg_group = new QGroupBox(tr("IMU Message"));
    msg_group->setLayout(msg_vlayout);


    // Display configuration group
    QVBoxLayout *display_vlayout = new QVBoxLayout;
    display_vlayout->addWidget(show_att_checkbox);
    display_vlayout->addWidget(show_heading_checkbox);
    display_vlayout->addWidget(show_labels_checkbox);

    QGroupBox *display_group = new QGroupBox(tr("Display Configuration"));
    display_group->setLayout(display_vlayout);


    // Dialog layout
    QGridLayout *dialog_layout = new QGridLayout;
    dialog_layout->addWidget(name_label, 0, 0);
    dialog_layout->addWidget(name_lineedit, 0, 1);
    dialog_layout->addWidget(topic_name_label, 1, 0);
    dialog_layout->addWidget(topic_name_lineedit, 1, 1);
    dialog_layout->addWidget(msg_group, 2, 0, 1, 2);
    dialog_layout->addWidget(display_group, 3, 0, 1, 2);
    dialog_layout->addWidget(button_box, 4, 1);
    dialog_layout->setSizeConstraint(QLayout::SetFixedSize);
    setLayout(dialog_layout);
    setWindowTitle(tr("Add/Edit IMU"));
}

/////////////////////////// Odometry Dialog ///////////////////////////
OdometryDialog::OdometryDialog(QWidget *parent) : QDialog(parent)
{
    this->createDialog();
}

void OdometryDialog::createDialog()
{
    QLabel *name_label = new QLabel(tr("Name"));
    name_lineedit = new QLineEdit;

    QLabel *topic_name_label = new QLabel(tr("Topic Name"));
    topic_name_lineedit = new QLineEdit;

    pos_checkbox = new QCheckBox(tr("Position"));
    pos_checkbox->setChecked(true);

    ori_checkbox = new QCheckBox(tr("Orientation"));
    ori_checkbox->setChecked(true);

    lin_vel_checkbox = new QCheckBox(tr("Linear Velocity"));
    lin_vel_checkbox->setChecked(true);

    ang_vel_checkbox = new QCheckBox(tr("Angular Velocity"));
    ang_vel_checkbox->setChecked(true);

    show_att_checkbox = new QCheckBox(tr("Show Attitude Indicator"));
    show_att_checkbox->setChecked(true);

    show_heading_checkbox = new QCheckBox(tr("Show Heading Indicator"));
    show_heading_checkbox->setChecked(true);

    show_labels_checkbox = new QCheckBox(tr("Show Labels"));
    show_labels_checkbox->setChecked(true);

    update_map_checkbox = new QCheckBox(tr("Update Map"));
    update_map_checkbox->setChecked(true);

    button_box = new QDialogButtonBox(QDialogButtonBox::Cancel |
                                      QDialogButtonBox::Ok);
    connect(button_box, SIGNAL(accepted()), this, SLOT(accept()));
    connect(button_box, SIGNAL(rejected()), this, SLOT(reject()));

    // Message group
    QVBoxLayout *msg_vlayout = new QVBoxLayout;
    msg_vlayout->addWidget(pos_checkbox);
    msg_vlayout->addWidget(ori_checkbox);
    msg_vlayout->addWidget(lin_vel_checkbox);
    msg_vlayout->addWidget(ang_vel_checkbox);

    QGroupBox *device_group = new QGroupBox(tr("Odometry Message"));
    device_group->setLayout(msg_vlayout);

    // Display group
    QVBoxLayout *display_vlayout = new QVBoxLayout;
    display_vlayout->addWidget(show_att_checkbox);
    display_vlayout->addWidget(show_heading_checkbox);
    display_vlayout->addWidget(show_labels_checkbox);
    display_vlayout->addWidget(update_map_checkbox);

    QGroupBox *display_group = new QGroupBox(tr("Display Configuration"));
    display_group->setLayout(display_vlayout);

    // Create dialog layout
    QGridLayout *dialog_layout = new QGridLayout;
    dialog_layout->addWidget(name_label, 0, 0);
    dialog_layout->addWidget(name_lineedit, 0, 1);
    dialog_layout->addWidget(topic_name_label, 1, 0);
    dialog_layout->addWidget(topic_name_lineedit, 1, 1);
    dialog_layout->addWidget(device_group, 2, 0, 1, 2);
    dialog_layout->addWidget(display_group, 3, 0, 1, 2);
    dialog_layout->addWidget(button_box, 4, 1);
    dialog_layout->setSizeConstraint(QLayout::SetFixedSize);
    setLayout(dialog_layout);
    setWindowTitle("Add/Edit Odometry");
}

/////////////////////////// Pose Dialog ///////////////////////////
PoseDialog::PoseDialog(QWidget *parent) : QDialog(parent)
{
    this->createDialog();
}

void PoseDialog::createDialog()
{
    QLabel *name_label = new QLabel(tr("Name"));
    name_lineedit = new QLineEdit;

    QLabel *topic_name_label = new QLabel(tr("Topic Name"));
    topic_name_lineedit = new QLineEdit;

    pos_checkbox = new QCheckBox(tr("Position"));
    pos_checkbox->setChecked(true);

    ori_checkbox = new QCheckBox(tr("Orientation"));
    ori_checkbox->setChecked(true);

    show_att_checkbox = new QCheckBox(tr("Show Attitude Indicator"));
    show_att_checkbox->setChecked(true);

    show_heading_checkbox = new QCheckBox(tr("Show Heading Indicator"));
    show_heading_checkbox->setChecked(true);

    show_labels_checkbox = new QCheckBox(tr("Show Labels"));
    show_labels_checkbox->setChecked(true);

    is_stamped_checkbox = new QCheckBox(tr("Pose Is Stamped"));
    is_stamped_checkbox->setChecked(true);

    has_covariance_checkbox = new QCheckBox(tr("Pose Has Covariance"));
    has_covariance_checkbox->setChecked(true);

    update_map_checkbox = new QCheckBox(tr("UpdateMap"));
    update_map_checkbox->setChecked(true);

    button_box = new QDialogButtonBox(QDialogButtonBox::Cancel |
                                      QDialogButtonBox::Ok);
    connect(button_box, SIGNAL(accepted()), this, SLOT(accept()));
    connect(button_box, SIGNAL(rejected()), this, SLOT(reject()));

    // Message group
    QVBoxLayout *msg_vlayout = new QVBoxLayout;
    msg_vlayout->addWidget(pos_checkbox);
    msg_vlayout->addWidget(ori_checkbox);
    msg_vlayout->addWidget(is_stamped_checkbox);
    msg_vlayout->addWidget(has_covariance_checkbox);

    QGroupBox *device_group = new QGroupBox(tr("Pose Message"));
    device_group->setLayout(msg_vlayout);

    // Display group
    QVBoxLayout *display_vlayout = new QVBoxLayout;
    display_vlayout->addWidget(show_att_checkbox);
    display_vlayout->addWidget(show_heading_checkbox);
    display_vlayout->addWidget(show_labels_checkbox);
    display_vlayout->addWidget(update_map_checkbox);

    QGroupBox *display_group = new QGroupBox(tr("Display Configuration"));
    display_group->setLayout(display_vlayout);

    // Create dialog layout
    QGridLayout *dialog_layout = new QGridLayout;
    dialog_layout->addWidget(name_label, 0, 0);
    dialog_layout->addWidget(name_lineedit, 0, 1);
    dialog_layout->addWidget(topic_name_label, 1, 0);
    dialog_layout->addWidget(topic_name_lineedit, 1, 1);
    dialog_layout->addWidget(device_group, 2, 0, 1, 2);
    dialog_layout->addWidget(display_group, 3, 0, 1, 2);
    dialog_layout->addWidget(button_box, 4, 1);
    dialog_layout->setSizeConstraint(QLayout::SetFixedSize);
    setLayout(dialog_layout);
    setWindowTitle("Add/Edit Pose");
}

