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

#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QtGui>
#include <QWidget>
#include <vector>
#include <time.h>


class GuiMonitor : public QWidget {
Q_OBJECT

struct SplitInfo {
  QGroupBox* subSplit;
  QHBoxLayout* subSplitLayout;
  QLabel* name;
  QLabel* status;
  QPushButton* startButton;
  QPushButton* stopButton;
  QLabel* freq;
};

struct CheckHzLabel {
  QLabel* lbl;
  QLabel* status;
  std::string topic;
  std::string topic_type;
  int target_freq;

  int old_freq; 
  int new_freq;

};

struct CheckValLabel {
  QLabel* lbl;
  std::string topic;
  std::string topic_type;
  int targetV1;
  int targetV2;
  int val; 
};

public:
  std::vector<CheckHzLabel> checkHzLabels_;
  CheckValLabel checkValLbl_;

  time_t start_time;

  void updateLabelSignalCaller(int rate, int target_rate, QLabel* lbl, QLabel* status);
  void updateValLabelSignalCaller(int val, int target1, int target2, QLabel* lbl);
  
  GuiMonitor(QWidget* parent, std::vector<std::vector<std::string>> settings);
  ~GuiMonitor();

  QSize minimumSizeHint() const;
  QSize sizeHint() const;

public slots:
  void startService0();
  void startService1();
  void startService2();
  void startService3();
  void startService4();
  void startService5();
  void startService6();
  void startService7();
  void startService8();
  void startService9();
  void startService10();

  void stopService0();
  void stopService1();
  void stopService2();
  void stopService3();
  void stopService4();
  void stopService5();
  void stopService6();
  void stopService7();
  void stopService8();
  void stopService9();
  void stopService10();

  void updateLabelSlot(int rate, int target_rate, QLabel* lbl, QLabel* status);
  void updateValLabelSlot(int val, int target1, int target2, QLabel* lbl);

signals:
  void updateLabelSignal(int rate, int target_rate, QLabel* lbl, QLabel* status);
  void updateValLabelSignal(int val, int target1, int target2, QLabel* lbl);

private:
  std::vector<SplitInfo> subSplitterElements_;
  QVBoxLayout* layout_;

  std::string startService0Cmd;
  std::string startService1Cmd;
  std::string startService2Cmd;
  std::string startService3Cmd;
  std::string startService4Cmd;
  std::string startService5Cmd;
  std::string startService6Cmd;
  std::string startService7Cmd;
  std::string startService8Cmd;
  std::string startService9Cmd;
  std::string startService10Cmd;

  std::string stopService0Cmd;
  std::string stopService1Cmd;
  std::string stopService2Cmd;
  std::string stopService3Cmd;
  std::string stopService4Cmd;
  std::string stopService5Cmd;
  std::string stopService6Cmd;
  std::string stopService7Cmd;
  std::string stopService8Cmd;
  std::string stopService9Cmd;
  std::string stopService10Cmd;
};

#endif
