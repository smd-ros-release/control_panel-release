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
 * \file   joint_state_display.h
 * \date   Feb 8, 2012
 * \author Matt Richard
 */
#ifndef CONTROL_PANEL_JOINT_STATE_DISPLAY_H
#define CONTROL_PANEL_JOINT_STATE_DISPLAY_H

#include <QWidget>
#include <QString>
#include <QStringList>
#include <QList>

QT_BEGIN_NAMESPACE
class QHBoxLayout;
class QLabel;
QT_END_NAMESPACE

#include <vector>

/**
 * \class JointStateDisplay
 * \brief Displays the position, velocity, and effort of a robots' joints
 */
class JointStateDisplay : public QWidget
{
    Q_OBJECT

    public:
        JointStateDisplay(QWidget *parent = 0); // <-- DO NOT USE THIS CONSTRUCTOR
        JointStateDisplay(const QStringList &names, const QStringList &display_names,
            QWidget *parent = 0);
        JointStateDisplay(const QString &widget_name, const QStringList &names,
            const QStringList &display_names, bool show_pos = true,
            bool show_vel = true, bool show_eff = false, QWidget *parent = 0);
        void zeroValues();

    public slots:
        void updateJointStateDisplay(const QStringList &names,
            std::vector<double> pos, std::vector<double> vel);
        void updateJointStateDisplay(const QStringList &names,
            const std::vector<double> &pos, const std::vector<double> &vel,
            const std::vector<double> &eff);

    private:
        void createWidget();

        QHBoxLayout *widget_layout;

        QString joint_name_header;

        QStringList joint_names;
        QStringList joint_display_names;

        QList<QLabel *> *position_labels;
        QList<QLabel *> *velocity_labels;
        QList<QLabel *> *effort_labels;

        bool use_position;
        bool use_velocity;
        bool use_effort;

        std::vector<double> position;
        std::vector<double> velocity;
        std::vector<double> effort;
};

#endif // CONTROL_PANEL_JOINT_STATE_DISPLAY
