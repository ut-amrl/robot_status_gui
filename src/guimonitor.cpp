/****************************************************************************
**
** Copyright (C) 2010 Nokia Corporation and/or its subsidiary(-ies).
** All rights reserved.
** Contact: Nokia Corporation (qt-info@nokia.com)
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:LGPL$
** Commercial Usage
** Licensees holding valid Qt Commercial licenses may use this file in
** accordance with the Qt Commercial License Agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and Nokia.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 2.1 as published by the Free Software
** Foundation and appearing in the file LICENSE.LGPL included in the
** packaging of this file.  Please review the following information to
** ensure the GNU Lesser General Public License version 2.1 requirements
** will be met: http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
**
** In addition, as a special exception, Nokia gives you certain additional
** rights.  These rights are described in the Nokia Qt LGPL Exception
** version 1.1, included in the file LGPL_EXCEPTION.txt in this package.
**
** GNU General Public License Usage
** Alternatively, this file may be used under the terms of the GNU
** General Public License version 3.0 as published by the Free Software
** Foundation and appearing in the file LICENSE.GPL included in the
** packaging of this file.  Please review the following information to
** ensure the GNU General Public License version 3.0 requirements will be
** met: http://www.gnu.org/copyleft/gpl.html.
**
** If you have questions regarding the use of this file, please contact
** Nokia at qt-info@nokia.com.
** $QT_END_LICENSE$
**
****************************************************************************/

#include "guimonitor.h"
#include <QtGui>
#include <QWidget>
#include <vector>

void callTerminal(std::string cmd);

GuiMonitor::GuiMonitor(QWidget* parent, std::vector<std::vector<std::string>> settings) :
    QWidget(),
    subSplitterElements_(),
    layout_(nullptr),
    checkHzLabels_() {

  layout_ = new QVBoxLayout();

  for (size_t i = 0; i < settings.size(); i++) {

    SplitInfo curr;
    CheckHzLabel checkHzLbl;

    QGroupBox* horizontalGroupBox1 = new QGroupBox();
    QHBoxLayout* sublayout = new QHBoxLayout;

    QLabel* name = new QLabel(settings[i][0].c_str());
    name->setAlignment(Qt::AlignCenter);
    name->setFixedWidth(250);

    QLabel* status = new QLabel("");

    QLabel* freq = new QLabel("");

    QPushButton* startBtn = new QPushButton("Start");
    startBtn->setFixedWidth(150);
    QPushButton* stopBtn = new QPushButton("Stop");
    stopBtn->setFixedWidth(150);

    switch (i) {
      case 0:
        startService0Cmd = settings[i][1];
        stopService0Cmd = settings[i][2];
        connect(startBtn,SIGNAL(clicked()),this,SLOT(startService0()));
        connect(stopBtn,SIGNAL(clicked()),this,SLOT(stopService0()));
        break;
      case 1:
        startService1Cmd = settings[i][1];
        stopService1Cmd = settings[i][2];
        connect(startBtn,SIGNAL(clicked()),this,SLOT(startService1()));
        connect(stopBtn,SIGNAL(clicked()),this,SLOT(stopService1()));
        break;
      case 2:
        startService2Cmd = settings[i][1];
        stopService2Cmd = settings[i][2];
        connect(startBtn,SIGNAL(clicked()),this,SLOT(startService2()));
        connect(stopBtn,SIGNAL(clicked()),this,SLOT(stopService2()));
        break;
      case 3:
        startService3Cmd = settings[i][1];
        stopService3Cmd = settings[i][2];
        connect(startBtn,SIGNAL(clicked()),this,SLOT(startService3()));
        connect(stopBtn,SIGNAL(clicked()),this,SLOT(stopService3()));
        break;
      case 4:
        startService4Cmd = settings[i][1];
        stopService4Cmd = settings[i][2];
        connect(startBtn,SIGNAL(clicked()),this,SLOT(startService4()));
        connect(stopBtn,SIGNAL(clicked()),this,SLOT(stopService4()));
        break;
      case 5:
        startService5Cmd = settings[i][1];
        stopService5Cmd = settings[i][2];
        connect(startBtn,SIGNAL(clicked()),this,SLOT(startService5()));
        connect(stopBtn,SIGNAL(clicked()),this,SLOT(stopService5()));
        break;
      case 6:
        startService6Cmd = settings[i][1];
        stopService6Cmd = settings[i][2];
        connect(startBtn,SIGNAL(clicked()),this,SLOT(startService6()));
        connect(stopBtn,SIGNAL(clicked()),this,SLOT(stopService6()));
        break;
      case 7:
        startService7Cmd = settings[i][1];
        stopService7Cmd = settings[i][2];
        connect(startBtn,SIGNAL(clicked()),this,SLOT(startService7()));
        connect(stopBtn,SIGNAL(clicked()),this,SLOT(stopService7()));
        break;
      case 8:
        startService8Cmd = settings[i][1];
        stopService8Cmd = settings[i][2];
        connect(startBtn,SIGNAL(clicked()),this,SLOT(startService8()));
        connect(stopBtn,SIGNAL(clicked()),this,SLOT(stopService8()));
        break;
      case 9:
        startService9Cmd = settings[i][1];
        stopService9Cmd = settings[i][2];
        connect(startBtn,SIGNAL(clicked()),this,SLOT(startService9()));
        connect(stopBtn,SIGNAL(clicked()),this,SLOT(stopService9()));
        break;
      case 10:
        startService10Cmd = settings[i][1];
        stopService10Cmd = settings[i][2];
        connect(startBtn,SIGNAL(clicked()),this,SLOT(startService10()));
        connect(stopBtn,SIGNAL(clicked()),this,SLOT(stopService10()));
        break;
    }

    if(settings[i].size() == 6) {
      //Standard Case for Config File

      status->setText("Loading...");
      status->setFixedWidth(150);
      status->setAlignment(Qt::AlignCenter);
      status->setStyleSheet("QLabel { color : blue; }");

      freq->setText("- Hz");
      freq->setAlignment(Qt::AlignCenter);
      freq->setFixedWidth(150);
      freq->setStyleSheet("QLabel { color : red; }");

      connect(this,SIGNAL(updateLabelSignal(int, int, QLabel*, QLabel*)),this,SLOT(updateLabelSlot(int, int, QLabel*, QLabel*)));

      checkHzLbl.lbl = freq;
      checkHzLbl.status = status;
      checkHzLbl.topic = settings[i][3];
      checkHzLbl.topic_type = settings[i][4];
      checkHzLbl.target_freq = std::stoi(settings[i][5]);
      checkHzLbl.old_freq = 0;
      checkHzLbl.new_freq = 0;
      checkHzLabels_.push_back(checkHzLbl);

    } else if (settings[i].size() == 7) {
      //Case for general settings and battery

      status->setText("");
      status->setFixedWidth(0);

      freq->setText("Battery: - V");
      freq->setAlignment(Qt::AlignCenter);
      freq->setFixedWidth(150);
      freq->setStyleSheet("QLabel { color : red; }");

      connect(this,SIGNAL(updateValLabelSignal(int, int, int, QLabel*)),this,SLOT(updateValLabelSlot(int, int, int, QLabel*)));

      checkValLbl_.lbl = freq;
      checkValLbl_.topic = settings[i][3];
      checkValLbl_.topic_type = settings[i][4];
      checkValLbl_.val = 0;
      checkValLbl_.targetV1 = std::stoi(settings[i][5]);
      checkValLbl_.targetV2 = std::stoi(settings[i][6]);


    } else {
      fprintf(stderr, "Error parsing line in Config file.\n");
    }

    curr.status = status;
    curr.freq = freq;
    curr.subSplit = horizontalGroupBox1;
    curr.subSplitLayout = sublayout;
    curr.name = name;
    curr.startButton = startBtn;
    curr.stopButton = stopBtn;
    subSplitterElements_.push_back(curr);
  }


  for (size_t i=0; i < subSplitterElements_.size(); i++) {
    subSplitterElements_[i].subSplitLayout->addWidget(subSplitterElements_[i].name);

    if(not subSplitterElements_[i].freq == NULL) {
      subSplitterElements_[i].subSplitLayout->addWidget(subSplitterElements_[i].status);
      subSplitterElements_[i].subSplitLayout->addWidget(subSplitterElements_[i].startButton);
      subSplitterElements_[i].subSplitLayout->addWidget(subSplitterElements_[i].stopButton);
      subSplitterElements_[i].subSplitLayout->addWidget(subSplitterElements_[i].freq);
    } else {
      subSplitterElements_[i].subSplitLayout->addWidget(subSplitterElements_[i].startButton);
      subSplitterElements_[i].subSplitLayout->addWidget(subSplitterElements_[i].stopButton);
    }



    subSplitterElements_[i].subSplit->setLayout(subSplitterElements_[i].subSplitLayout);

    layout_->addWidget(subSplitterElements_[i].subSplit);
  }

  setLayout(layout_);

}

GuiMonitor::~GuiMonitor() {
}

QSize GuiMonitor::minimumSizeHint() const {
  return QSize(50, 50);
}

QSize GuiMonitor::sizeHint() const {
  return QSize(640, 480);
}

void GuiMonitor::startService0() {
  callTerminal(startService0Cmd);
}
void GuiMonitor::startService1() {
  callTerminal(startService1Cmd);
}
void GuiMonitor::startService2() {
  callTerminal(startService2Cmd);
}
void GuiMonitor::startService3() {
  callTerminal(startService3Cmd);
}
void GuiMonitor::startService4() {
  callTerminal(startService4Cmd);
}
void GuiMonitor::startService5() {
  callTerminal(startService5Cmd);
}
void GuiMonitor::startService6() {
  callTerminal(startService6Cmd);
}
void GuiMonitor::startService7() {
  callTerminal(startService7Cmd);
}
void GuiMonitor::startService8() {
  callTerminal(startService8Cmd);
}
void GuiMonitor::startService9() {
  callTerminal(startService9Cmd);
}
void GuiMonitor::startService10() {
  callTerminal(startService10Cmd);
}

void GuiMonitor::stopService0() {
  callTerminal(stopService0Cmd);
}
void GuiMonitor::stopService1() {
  callTerminal(stopService1Cmd);
}
void GuiMonitor::stopService2() {
  callTerminal(stopService2Cmd);
}
void GuiMonitor::stopService3() {
  callTerminal(stopService3Cmd);
}
void GuiMonitor::stopService4() {
  callTerminal(stopService4Cmd);
}
void GuiMonitor::stopService5() {
  callTerminal(stopService5Cmd);
}
void GuiMonitor::stopService6() {
  callTerminal(stopService6Cmd);
}
void GuiMonitor::stopService7() {
  callTerminal(stopService7Cmd);
}
void GuiMonitor::stopService8() {
  callTerminal(stopService8Cmd);
}
void GuiMonitor::stopService9() {
  callTerminal(stopService9Cmd);
}
void GuiMonitor::stopService10() {
  callTerminal(stopService10Cmd);
}

void GuiMonitor::updateLabelSlot(int rate, int target_rate, QLabel* lbl, QLabel* status) {
  std::string text = std::to_string(rate);
  text = text + " Hz";

  lbl->setText(QString(text.c_str()));

  if(rate >= target_rate) {
    lbl->setStyleSheet("QLabel { color : green; }");
  } else {
    lbl->setStyleSheet("QLabel { color : red; }");
  }

  if(rate > 0) {
    status->setText(QString("On"));
    status->setStyleSheet("QLabel { color : green; }");
  } else {
    status->setText(QString("Off"));
    status->setStyleSheet("QLabel { color : red; }");
  }
}

void GuiMonitor::updateValLabelSlot(int val, int target1, int target2, QLabel* lbl) {
  std::string text = "Battery: ";
  text = text + std::to_string(val);
  text = text + " V";

  lbl->setText(QString(text.c_str()));

  if(val < target1) {
    lbl->setStyleSheet("QLabel { color : red; }");
  } else if (val < target2) {
    lbl->setStyleSheet("QLabel { color : orange; }");
  }
  else {
    lbl->setStyleSheet("QLabel { color : green; }");
  }
}

void callTerminal(std::string cmd) {
  if(!system(cmd.c_str())){
    fprintf(stderr, "Could not execute terminal command.\n");
  }
}

void GuiMonitor::updateLabelSignalCaller(int rate, int target_rate, QLabel* lbl, QLabel* status) {
  this->updateLabelSignal(rate, target_rate, lbl, status);
}

void GuiMonitor::updateValLabelSignalCaller(int val, int target1, int target2, QLabel* lbl) {
  this->updateValLabelSignal(val, target1, target2, lbl);
}
