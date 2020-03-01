#pragma once
//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    gui_topic.h
\author  Kyle Vedder, (C) 2020
*/
//========================================================================

#include "topic_config.h"

#ifndef Q_MOC_RUN
#include <QWidget>
#include <QtGui>
#endif

class GUITopic : public QWidget {
  Q_OBJECT

  const TopicConfig& cfg;
  QLabel* name;
  QLabel* status;
  QPushButton* start_button;
  QPushButton* stop_button;
  QLabel* freq;

 public:
  GUITopic(QWidget* parent, const TopicConfig& cfg);
  ~GUITopic();

  void UpdateState();

 private:
 signals:
  void UpdateStateSignal();

 private slots:
  void StartCommand();
  void StopCommand();

  void UpdateStateSlot();
};
