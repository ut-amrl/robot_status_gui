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
\file    main.cpp
\author  Joydeep Biswas, (C) 2010, Kyle Vedder, (C) 2020
*/
//========================================================================

#include <signal.h>

#include <ros/ros.h>
#include <QApplication>
#include <thread>
#include <unordered_map>
#include <vector>

#include "gui_monitor.h"
#include "topic_config.h"

bool running_ = true;
QApplication* app = nullptr;
GuiMonitor* gui = nullptr;
std::thread ros_thread;

void SignalHandler(const int signal) {
  running_ = false;
  if (app != nullptr) {
    app->exit();
  }
}

void ROSThreadBody(TopicConfigManager* tcm) {
  static constexpr float kLoopRate = 2.0f;
  ros::Rate rate(kLoopRate);
  while (ros::ok() && running_) {
    ros::spinOnce();
    tcm->ComputeResults(kLoopRate);
    if (gui != nullptr) {
      gui->UpdateState();
    }
    rate.sleep();
  }
}

std::string GetConfigPath(int argc, char** argv) {
  static constexpr auto kDefaultConfigFilePath = "config.txt";
  static constexpr auto kPathPrefix = "--config=";
  for (int i = 0; i < argc; ++i) {
    std::string arg(argv[i]);
    if (arg.rfind(kPathPrefix, 0) == 0) {
      return arg.substr(sizeof(kPathPrefix) + 1,
                        arg.length() - (sizeof(kPathPrefix) + 1));
    }
  }
  return kDefaultConfigFilePath;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "gui_monitor", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  signal(SIGINT, SignalHandler);

  TopicConfigManager tcm(GetConfigPath(argc, argv));
  auto subscribers = tcm.SetupSubscribers(&n);

  ros_thread = std::thread(ROSThreadBody, &tcm);

  app = new QApplication(argc, argv);
  gui = new GuiMonitor(tcm);

  gui->show();
  const auto gui_ret_val = app->exec();
  running_ = false;
  ros_thread.join();

  for (const auto& c : tcm.configs) {
    c.RunStopCommand();
  }  

  return gui_ret_val;
}
