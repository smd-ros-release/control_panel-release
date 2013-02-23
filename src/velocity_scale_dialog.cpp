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
 * \file   velocity_scale_dialog.cpp
 * \date   April 4, 2012
 * \author Matt Richard
 */
#include <QtGui>
#include <control_panel/velocity_scale_dialog.h>

VelocityScaleDialog::VelocityScaleDialog(QWidget *parent) : QDialog(parent)
{
	QLabel *lin_vel_label = new QLabel(tr("Linear Velocity Scale"));
	QLabel *ang_vel_label = new QLabel(tr("Angular Velocity Scale"));

	// Create spinbox for setting linear velocity
	lin_vel_spinbox = new QDoubleSpinBox;
	lin_vel_spinbox->setDecimals(2);
	lin_vel_spinbox->setMinimum(0.0);
	lin_vel_spinbox->setSingleStep(0.1);

	// Create spinbox for setting angular velocity
	ang_vel_spinbox = new QDoubleSpinBox;
	ang_vel_spinbox->setDecimals(2);
	ang_vel_spinbox->setMinimum(0.0);
	ang_vel_spinbox->setSingleStep(0.1);

	QDialogButtonBox *button_box = new QDialogButtonBox(QDialogButtonBox::Cancel |
		QDialogButtonBox::Ok);

	connect(button_box, SIGNAL(accepted()), this, SLOT(accept()));
	connect(button_box, SIGNAL(rejected()), this, SLOT(reject()));

	// Create the dialogs layout
	QGridLayout *dialog_layout = new QGridLayout;
	dialog_layout->addWidget(lin_vel_label, 0, 0);
	dialog_layout->addWidget(lin_vel_spinbox, 0, 1);
	dialog_layout->addWidget(ang_vel_label, 1, 0);
	dialog_layout->addWidget(ang_vel_spinbox, 1, 1);
	dialog_layout->addWidget(button_box, 2, 0, 1, 2);
	dialog_layout->setSizeConstraint(QLayout::SetFixedSize);

	setLayout(dialog_layout);
	setWindowTitle(tr("Set Velocity Scale"));
}
