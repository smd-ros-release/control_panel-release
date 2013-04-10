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
 * \file   call_service_dialog.cpp
 * \date   March 16, 2012
 * \author Matt Richard
 */
#include <QtGui>
#include "control_panel/call_service_dialog.h"

CallServiceDialog::CallServiceDialog(QWidget *parent) : QDialog(parent)
{
	createDialog();
}

CallServiceDialog::CallServiceDialog(const QStringList &services,
	QWidget *parent) : QDialog(parent)
{
	createDialog();

	service_list->addItems(services);
}

QString CallServiceDialog::getSelectedService() const
{
	// Return selected service, or a null QString if no service is selected
	if(service_list->currentItem() != 0)
		return service_list->currentItem()->text();
	return QString();
}

void CallServiceDialog::createDialog()
{
	service_list = new QListWidget;
	service_list->setSortingEnabled(true);
	connect(service_list, SIGNAL(itemDoubleClicked(QListWidgetItem *)),
		this, SLOT(accept()));

	button_box = new QDialogButtonBox(QDialogButtonBox::Cancel |
		QDialogButtonBox::Ok);
	connect(button_box, SIGNAL(accepted()), this, SLOT(accept()));
	connect(button_box, SIGNAL(rejected()), this, SLOT(reject()));
    
	// Create dialog layout
	QVBoxLayout *dialog_layout = new QVBoxLayout;
	dialog_layout->addWidget(service_list);
	dialog_layout->addWidget(button_box);
	setLayout(dialog_layout);

	setWindowTitle(tr("Call Service"));
}
