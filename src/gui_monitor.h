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
\file    gui_monitor.h
\author  Kyle Vedder, (C) 2020
*/
//========================================================================

#include "gui_topic.h"
#include "topic_config.h"

#include <time.h>
#include <QWidget>
#include <QtGui>
#include <vector>

class GuiMonitor : public QWidget {
  Q_OBJECT

  std::vector<GUITopic*> topic_widgets_;

 public:
  void UpdateState();

  explicit GuiMonitor(const TopicConfigManager& tcm);
  ~GuiMonitor();

  QSize minimumSizeHint() const;
  QSize sizeHint() const;

 public slots:
 signals:
};
