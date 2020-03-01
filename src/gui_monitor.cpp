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
\file    gui_monitor.cpp
\author  Kyle Vedder, (C) 2020
*/
//========================================================================
#include "gui_monitor.h"

#ifndef Q_MOC_RUN
#include <QWidget>
#include <QtGui>
#include <vector>
#include "gui_topic.h"
#endif

GuiMonitor::GuiMonitor(const TopicConfigManager& tcm) : QWidget() {
  QVBoxLayout* layout = new QVBoxLayout(this);
  for (const TopicConfig& cfg : tcm.configs) {
    GUITopic* topic = new GUITopic(this, cfg);
    topic_widgets_.push_back(topic);
    layout->addWidget(topic);
  }
}

GuiMonitor::~GuiMonitor() {}

void GuiMonitor::UpdateState() {
  for (GUITopic* t : topic_widgets_) {
    t->UpdateState();
  }
}

QSize GuiMonitor::minimumSizeHint() const { return QSize(50, 50); }

QSize GuiMonitor::sizeHint() const { return QSize(640, 480); }
