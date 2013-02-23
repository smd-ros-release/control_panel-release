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
 * \file   call_service_dialog.h
 * \date   March 16, 2012
 * \author Matt Richard
 */
#ifndef CONTROL_PANEL_CALL_SERVICE_DIALOG_H
#define CONTROL_PANEL_CALL_SERVICE_DIALOG_H

#include <QDialog>
#include <QListWidget>

QT_BEGIN_NAMESPACE
class QDialogButtonBox;
class QString;
class QStringList;
QT_END_NAMESPACE

/**
 * \class CallServiceDialog
 * \brief Dialog that displays the list of services a robot provides and lets a user select one.
 */
class CallServiceDialog : public QDialog
{
	Q_OBJECT

	public:
		/**
		 * \brief Constructs the dialog with an empty QListWidget
		 *
		 * \param parent The parent widget.
		 */
		CallServiceDialog(QWidget *parent = 0);

		/**
		 * \brief Constructs the dialog and populates the QListWidget with services passed in
		 *
		 * \param services The list of services to add to the QListWidget
		 * \param parent   The perent widget.
		 */
		CallServiceDialog(const QStringList &services, QWidget *parent = 0);

		/**
		 * \brief Adds service to the ListWidget
		 *
		 * \param service Service to add to the list.
		 */
		void addService(const QString &service) { service_list->addItem(service); }

		/**
		 * \brief Returns the service that was selected by the user, or a null QString if no service was selected.
		 */
		QString getSelectedService() const;

	private:
		/**
		 * \brief Creates the dialog's widgets and layout
		 */
		void createDialog();

		QListWidget *service_list;
		QDialogButtonBox *button_box;
};

#endif // CONTROL_PANEL_CALL_SERVICE_DIALOG_H
