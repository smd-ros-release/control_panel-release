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
 * \file   component_dialogs.h
 * \date   Jan 9, 2012
 * \author Matt Richard
 */
#ifndef CONTROL_PANEL_COMPONENT_DIALOGS_H
#define CONTROL_PANEL_COMPONENT_DIALOGS_H

#include <QDialog>

QT_BEGIN_NAMESPACE
class QDialogButtonBox;
QT_END_NAMESPACE

#include <QCheckBox>
#include <QLabel>
#include <QLineEdit>
#include <QString>

/**
 * \class ComponentDialog
 * \brief Dialog for adding/editing a robot configuration file component.
 */
class ComponentDialog : public QDialog
{
    Q_OBJECT

    public:
        ComponentDialog(QWidget *parent = 0);
        ComponentDialog(const QString &name, const QString &topic_name,
                        const QString &name_label_text = QString(),
                        const QString &topic_name_label_text = QString(),
                        QWidget *parent = 0);
        QString getName() const { return name_lineedit->text(); }
        QString getTopicName() const { return topic_name_lineedit->text(); }
        void setName(const QString &name) { name_lineedit->setText(name); }
        void setNameLabelText(const QString &name) { name_label->setText(name); }
        void setTopicName(const QString &name) { topic_name_lineedit->setText(name); }
        void setTopicNameLabelText(const QString &name) { topic_name_label->setText(name); }

    private:
        void createDialog();
        
        QLabel *name_label;
        QLabel *topic_name_label;
        QLineEdit *name_lineedit;
        QLineEdit *topic_name_lineedit;
        QDialogButtonBox *button_box;
};

/**
 * \class CompassDialog
 * \brief Dialog for adding/editing a compass component for a robot config file
 */
class CompassDialog : public QDialog
{
    Q_OBJECT

    public:
        CompassDialog(QWidget *parent = 0);
        QString getName() const { return name_lineedit->text(); }
        QString getTopicName() const { return topic_name_lineedit->text(); }
        bool isShowHeadingChecked() const { return show_heading_checkbox->isChecked(); }
        bool isShowLabelChecked() const { return show_label_checkbox->isChecked(); }
        void setName(const QString &name) { name_lineedit->setText(name); }
        void setTopicName(const QString &name) { topic_name_lineedit->setText(name); }
        void setShowHeadingChecked(bool checked) { show_heading_checkbox->setChecked(checked); }
        void setShowLabelChecked(bool checked) { show_label_checkbox->setChecked(checked); }

    private:
        void createDialog();

        QLineEdit *name_lineedit;
        QLineEdit *topic_name_lineedit;
        QCheckBox *show_heading_checkbox;
        QCheckBox *show_label_checkbox;
        QDialogButtonBox *button_box;
};

/**
 * \class GpsDialog
 * \brief Dialog for adding/editing a GPS component for a robot config file
 */
class GpsDialog : public QDialog
{
    Q_OBJECT

    public:
        GpsDialog(QWidget *parent = 0);
        QString getName() const { return name_lineedit->text(); }
        QString getTopicName() const { return topic_name_lineedit->text(); }
        bool isLatitudeChecked() const { return lat_checkbox->isChecked(); }
        bool isLongitudeChecked() const { return long_checkbox->isChecked(); }
        bool isAltitudeChecked() const { return alt_checkbox->isChecked(); }
        void setName(const QString &name) { name_lineedit->setText(name); }
        void setTopicName(const QString &name) { topic_name_lineedit->setText(name); }
        void setLatitudeChecked(bool checked) { lat_checkbox->setChecked(checked); }
        void setLongitudeChecked(bool checked) { long_checkbox->setChecked(checked); }
        void setAltitudeChecked(bool checked) { alt_checkbox->setChecked(checked); }

    private:
        void createDialog();

        QLineEdit *name_lineedit;
        QLineEdit *topic_name_lineedit;
        QCheckBox *lat_checkbox;
        QCheckBox *long_checkbox;
        QCheckBox *alt_checkbox;
        QDialogButtonBox *button_box;
};

/**
 * \class ImuDialog
 * \brief Dialog for adding/editing a IMU component for a robot config file
 */
class ImuDialog : public QDialog
{
    Q_OBJECT

    public:
        ImuDialog(QWidget *parent = 0);
        QString getName() const { return name_lineedit->text(); }
        QString getTopicName() const { return topic_name_lineedit->text(); }
        bool isRollChecked() const { return roll_checkbox->isChecked(); }
        bool isPitchChecked() const { return pitch_checkbox->isChecked(); }
        bool isYawChecked() const { return yaw_checkbox->isChecked(); }
        bool isAngularVelocityChecked() const { return ang_vel_checkbox->isChecked(); }
        bool isLinearAccelerationChecked() const { return lin_accel_checkbox->isChecked(); }
        bool isShowAttitudeChecked() const { return show_att_checkbox->isChecked(); }
        bool isShowHeadingChecked() const { return show_heading_checkbox->isChecked(); }
        bool isShowLabelsChecked() const { return show_labels_checkbox->isChecked(); }
        void setName(const QString &name) { name_lineedit->setText(name); }
        void setTopicName(const QString &name) { topic_name_lineedit->setText(name); }
        void setRollChecked(bool checked) { roll_checkbox->setChecked(checked); }
        void setPitchChecked(bool checked) { pitch_checkbox->setChecked(checked); }
        void setYawChecked(bool checked) { yaw_checkbox->setChecked(checked); }
        void setAngularVelocityChecked(bool checked) { ang_vel_checkbox->setChecked(checked); }
        void setLinearAccelerationChecked(bool checked) { lin_accel_checkbox->setChecked(checked); }
        void setShowAttitudeChecked(bool checked) { show_att_checkbox->setChecked(checked); }
        void setShowHeadingChecked(bool checked) { show_heading_checkbox->setChecked(checked); }
        void setShowLabelsChecked(bool checked) { show_labels_checkbox->setChecked(checked); }

    private:
        void createDialog();

        QLineEdit *name_lineedit;
        QLineEdit *topic_name_lineedit;
        QCheckBox *roll_checkbox;
        QCheckBox *pitch_checkbox;
        QCheckBox *yaw_checkbox;
        QCheckBox *ang_vel_checkbox;
        QCheckBox *lin_accel_checkbox;
        QCheckBox *show_att_checkbox;
        QCheckBox *show_heading_checkbox;
        QCheckBox *show_labels_checkbox;
        QDialogButtonBox *button_box;
};

/**
 * \class OdometryDialog
 * \brief Dialog for adding/editing an odometry component for a robot config file
 */
class OdometryDialog : public QDialog
{
    Q_OBJECT

    public:
        OdometryDialog(QWidget *parent = 0);
        QString getName() const { return name_lineedit->text(); }
        QString getTopicName() const { return topic_name_lineedit->text(); }
        bool isPositionChecked() const { return pos_checkbox->isChecked(); }
        bool isOrientationChecked() const { return ori_checkbox->isChecked(); }
        bool isLinearVelocityChecked() const { return lin_vel_checkbox->isChecked(); }
        bool isAngularVelocityChecked() const { return ang_vel_checkbox->isChecked(); }
        bool isShowAttitudeChecked() const { return show_att_checkbox->isChecked(); }
        bool isShowHeadingChecked() const { return show_heading_checkbox->isChecked(); }
        bool isShowLabelsChecked() const { return show_labels_checkbox->isChecked(); }
        bool isUpdateMapChecked() const { return update_map_checkbox->isChecked(); }
        void setName(const QString &name) { name_lineedit->setText(name); }
        void setTopicName(const QString &name) { topic_name_lineedit->setText(name); }
        void setPositionChecked(bool checked) { pos_checkbox->setChecked(checked); }
        void setOrientationChecked(bool checked) { ori_checkbox->setChecked(checked); }
        void setLinearVelocityChecked(bool checked) { lin_vel_checkbox->setChecked(checked); }
        void setAngularVelocityChecked(bool checked) { ang_vel_checkbox->setChecked(checked); }
        void setShowAttitudeChecked(bool checked) { show_att_checkbox->setChecked(checked); }
        void setShowHeadingChecked(bool checked) { show_heading_checkbox->setChecked(checked); }
        void setShowLabelsChecked(bool checked) { show_labels_checkbox->setChecked(checked); }
        void setUpdateMapChecked(bool checked) { update_map_checkbox->setChecked(checked); }

    private:
        void createDialog();

        QLineEdit *name_lineedit;
        QLineEdit *topic_name_lineedit;
        QCheckBox *pos_checkbox;
        QCheckBox *ori_checkbox;
        QCheckBox *lin_vel_checkbox;
        QCheckBox *ang_vel_checkbox;
        QCheckBox *show_att_checkbox;
        QCheckBox *show_heading_checkbox;
        QCheckBox *show_labels_checkbox;
        QCheckBox *update_map_checkbox;
        QDialogButtonBox *button_box;
};

/**
 * \class PoseDialog
 * \brief Dialog for adding/editing a pose component for a robot config file
 */
class PoseDialog : public QDialog
{
    Q_OBJECT

    public:
        PoseDialog(QWidget *parent = 0);
        QString getName() const { return name_lineedit->text(); }
        QString getTopicName() const { return topic_name_lineedit->text(); }
        bool isPositionChecked() const { return pos_checkbox->isChecked(); }
        bool isOrientationChecked() const { return ori_checkbox->isChecked(); }
        bool isShowAttitudeChecked() const { return show_att_checkbox->isChecked(); }
        bool isShowHeadingChecked() const { return show_heading_checkbox->isChecked(); }
        bool isShowLabelsChecked() const { return show_labels_checkbox->isChecked(); }
        bool isIsStampedChecked() const { return is_stamped_checkbox->isChecked(); }
        bool isHasCovarianceChecked() const { return has_covariance_checkbox->isChecked(); }
        bool isUpdateMapChecked() const { return update_map_checkbox->isChecked(); }
        void setName(const QString &name) { name_lineedit->setText(name); }
        void setTopicName(const QString &name) { topic_name_lineedit->setText(name); }
        void setPositionChecked(bool checked) { pos_checkbox->setChecked(checked); }
        void setOrientationChecked(bool checked) { ori_checkbox->setChecked(checked); }
        void setShowAttitudeChecked(bool checked) { show_att_checkbox->setChecked(checked); }
        void setShowHeadingChecked(bool checked) { show_heading_checkbox->setChecked(checked); }
        void setShowLabelsChecked(bool checked) { show_labels_checkbox->setChecked(checked); }
        void setIsStampedChecked(bool checked) { if(!checked) has_covariance_checkbox->setChecked(checked); is_stamped_checkbox->setChecked(checked); }
        void setHasCovarianceChecked(bool checked) { if(checked) is_stamped_checkbox->setChecked(checked); has_covariance_checkbox->setChecked(checked); }
        void setUpdateMapChecked(bool checked) { update_map_checkbox->setChecked(checked); }

    private:
        void createDialog();

        QLineEdit *name_lineedit;
        QLineEdit *topic_name_lineedit;
        QCheckBox *pos_checkbox;
        QCheckBox *ori_checkbox;
        QCheckBox *show_att_checkbox;
        QCheckBox *show_heading_checkbox;
        QCheckBox *show_labels_checkbox;
        QCheckBox *is_stamped_checkbox;
        QCheckBox *has_covariance_checkbox;
        QCheckBox *update_map_checkbox;
        QDialogButtonBox *button_box;
};

#endif // CONTROL_PANEL_COMPONENT_DIALOGS_H

