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
\file    gui_topic.cpp
\author  Kyle Vedder, (C) 2020
*/
//========================================================================

#include "gui_topic.h"

#include <iostream>

GUITopic::GUITopic(QWidget* parent, const TopicConfig& cfg)
    : QWidget(parent),
      cfg(cfg),
      name(nullptr),
      status(nullptr),
      start_button(nullptr),
      stop_button(nullptr),
      freq(nullptr) {
  QHBoxLayout* horizontal_layout = new QHBoxLayout(this);

  name = new QLabel(cfg.human_name.c_str());
  name->setAlignment(Qt::AlignCenter);
  name->setFixedWidth(250);
  horizontal_layout->addWidget(name);

  status = new QLabel("");
  status->setText("Loading...");
  status->setFixedWidth(150);
  status->setAlignment(Qt::AlignCenter);
  status->setStyleSheet("QLabel { color : blue; }");
  horizontal_layout->addWidget(status);

  start_button = new QPushButton("Start");
  start_button->setFixedWidth(150);
  connect(start_button, SIGNAL(clicked()), this, SLOT(StartCommand()));
  horizontal_layout->addWidget(start_button);

  stop_button = new QPushButton("Stop");
  stop_button->setFixedWidth(150);
  connect(stop_button, SIGNAL(clicked()), this, SLOT(StopCommand()));
  horizontal_layout->addWidget(stop_button);

  freq = new QLabel("");
  freq->setText("- Hz");
  freq->setAlignment(Qt::AlignCenter);
  freq->setFixedWidth(150);
  freq->setStyleSheet("QLabel { color : red; }");
  horizontal_layout->addWidget(freq);

  connect(this, SIGNAL(UpdateStateSignal()), this, SLOT(UpdateStateSlot()));
}

GUITopic::~GUITopic() {}

void GUITopic::UpdateStateSlot() {

  if (cfg.target_freq <= 0) {
    status->setText("N/A");
    status->setStyleSheet("QLabel { color : gray; }");
    freq->setText("N/A");
    freq->setStyleSheet("QLabel { color : gray; }");
    return;
  }

  freq->setText((std::to_string(cfg.current_freq) + " Hz").c_str());
  if (cfg.current_freq >= cfg.target_freq) {
    freq->setStyleSheet("QLabel { color : green; }");
  } else {
    freq->setStyleSheet("QLabel { color : red; }");
  }

  if (cfg.current_freq > 0) {
    status->setText("On");
    status->setStyleSheet("QLabel { color : green; }");
  } else {
    status->setText("Off");
    status->setStyleSheet("QLabel { color : red; }");
  }
}

void GUITopic::UpdateState() { this->UpdateStateSignal(); }

void GUITopic::StartCommand() { cfg.RunStartCommand(); }

void GUITopic::StopCommand() { cfg.RunStopCommand(); }