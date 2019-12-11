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
\brief   OpenGL Viewer
\author  Joydeep Biswas, (C) 2010
*/
//========================================================================

#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <time.h>

#include <fstream>
#include <vector>
#include <QApplication>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>

#include "guimonitor.h"

struct SubUpdateStruct;

bool run_ = true;
GuiMonitor* gui_monitor;

std::vector<std::vector<std::string>> loadConfig();
void updateLabels();

struct SubUpdateStruct {

  int label_num = -1;

  void incrementFreqStringMsg(const std_msgs::String& msg) {
    gui_monitor->checkHzLabels_[label_num].new_freq = gui_monitor->checkHzLabels_[label_num].new_freq + 1;
  }
  void incrementFreqPoseStamped(const geometry_msgs::PoseStamped& msg) {
    gui_monitor->checkHzLabels_[label_num].new_freq = gui_monitor->checkHzLabels_[label_num].new_freq + 1;
  }
  void incrementFreqPath(const nav_msgs::Path& msg) {
    gui_monitor->checkHzLabels_[label_num].new_freq = gui_monitor->checkHzLabels_[label_num].new_freq + 1;
  }
  void incrementFreqImage(const sensor_msgs::Image& msg) {
    gui_monitor->checkHzLabels_[label_num].new_freq = gui_monitor->checkHzLabels_[label_num].new_freq + 1;
  }
  void incrementFreqTwist(const geometry_msgs::Twist& msg) {
    gui_monitor->checkHzLabels_[label_num].new_freq = gui_monitor->checkHzLabels_[label_num].new_freq + 1;
  }
  void incrementFreqLaserScan(const sensor_msgs::LaserScan& msg) {
    gui_monitor->checkHzLabels_[label_num].new_freq = gui_monitor->checkHzLabels_[label_num].new_freq + 1;
  }
};

void SignalHandler(int sig) {
  run_ = false;
}

void* RosThread(void*) {
  printf("In ROS loop\n");

  ros::Rate rate(20.0);
  while (ros::ok() && run_) {
    ros::spinOnce();
    rate.sleep();
    updateLabels();
  }
  return nullptr;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "gui_monitor", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  signal(SIGINT, SignalHandler);
  pthread_t ros_thread;
  if (pthread_create(&ros_thread, NULL, RosThread, nullptr) != 0) {
    fprintf(stderr, "ERROR: Unable to create ROS thread, exiting.\n");
    exit(1);
  }
  QApplication app(argc, argv);
  GuiMonitor gui(nullptr, loadConfig());
  gui.show();
  gui_monitor = &gui;
  gui_monitor->start_time = time(0);

  std::vector<ros::Subscriber> subscribers;
  std::vector<SubUpdateStruct> subUpdateStructs;

  for(size_t i = 0; i < gui_monitor->checkHzLabels_.size(); i++) {
     SubUpdateStruct curr;
     subUpdateStructs.push_back(curr);
  }

  for(size_t i = 0; i < gui_monitor->checkHzLabels_.size(); i++) {
    std::string type_str = gui_monitor->checkHzLabels_[i].topic_type;

    if(type_str == "std_msgs/String") {

      subUpdateStructs[i].label_num = static_cast<int>(i);
      ros::Subscriber sub = n.subscribe(gui_monitor->checkHzLabels_[i].topic, 10, &SubUpdateStruct::incrementFreqStringMsg, &subUpdateStructs[i]);
      subscribers.push_back(sub);

    } else if (type_str == "geometry_msgs/PoseStamped") {

      subUpdateStructs[i].label_num = static_cast<int>(i);
      ros::Subscriber sub = n.subscribe(gui_monitor->checkHzLabels_[i].topic, 10, &SubUpdateStruct::incrementFreqPoseStamped, &subUpdateStructs[i]);
      subscribers.push_back(sub);

    } else if (type_str == "nav_msgs/Path") {

      subUpdateStructs[i].label_num = static_cast<int>(i);
      ros::Subscriber sub = n.subscribe(gui_monitor->checkHzLabels_[i].topic, 10, &SubUpdateStruct::incrementFreqPath, &subUpdateStructs[i]);
      subscribers.push_back(sub);

    } else if (type_str == "sensor_msgs/Image") {

      subUpdateStructs[i].label_num = static_cast<int>(i);
      ros::Subscriber sub = n.subscribe(gui_monitor->checkHzLabels_[i].topic, 10, &SubUpdateStruct::incrementFreqImage, &subUpdateStructs[i]);
      subscribers.push_back(sub);
    } else if (type_str == "geometry_msgs/Twist") {

      subUpdateStructs[i].label_num = static_cast<int>(i);
      ros::Subscriber sub = n.subscribe(gui_monitor->checkHzLabels_[i].topic, 10, &SubUpdateStruct::incrementFreqTwist, &subUpdateStructs[i]);
      subscribers.push_back(sub);
    } else if (type_str == "sensor_msgs/LaserScan") {

      subUpdateStructs[i].label_num = static_cast<int>(i);
      ros::Subscriber sub = n.subscribe(gui_monitor->checkHzLabels_[i].topic, 10, &SubUpdateStruct::incrementFreqLaserScan, &subUpdateStructs[i]);
      subscribers.push_back(sub);
    } else {
      fprintf(stderr, "Unrecognized Type For Topic.\n");
    }

  }

  // if(gui_monitor->checkValLbl_.topic_type == "Status") {
  //   SubUpdateStruct curr;
  //   subUpdateStructs.push_back(curr);

  //   ros::Subscriber sub = n.subscribe(gui_monitor->checkValLbl_.topic, 10, &SubUpdateStruct::incrementFreqStatus, &subUpdateStructs[subUpdateStructs.size()-1]);
  //   subscribers.push_back(sub);
  // } else {
  //   fprintf(stderr, "Unrecognized Type For Topic.\n");
  // }

  return app.exec();
}

std::vector<std::vector<std::string>> loadConfig() {
  std::vector<std::vector<std::string>> allSettings;
  std::ifstream cFile ("config.txt");
  if (cFile.is_open()) {
    std::string line;
    while (getline(cFile, line)) {
      if (line[0] == '#' || line.empty()) {
        continue;
      }

      std::vector<std::string> currSettings;

      while (line.find("\"")!=std::string::npos) {
        line.erase(0, line.find("\"")+1);
        std::string setting = line.substr(0, line.find("\""));
        currSettings.push_back(setting);
        line.erase(0, line.find("\"")+1);
      }

      allSettings.push_back(currSettings);
    }
  } else {
    fprintf(stderr, "Couldn't open config file for reading\n");
  }
  return allSettings;
}

void updateLabels() {

  if(gui_monitor == NULL) return;

  double seconds_since_start = difftime( time(0), gui_monitor->start_time);

  if (seconds_since_start >= 1.0) {

  	gui_monitor->start_time = time(0);

    for (size_t i = 0; i < gui_monitor->checkHzLabels_.size(); i++) {
      gui_monitor->checkHzLabels_[i].old_freq = gui_monitor->checkHzLabels_[i].new_freq;
      gui_monitor->checkHzLabels_[i].new_freq = 0;

      gui_monitor->updateLabelSignalCaller(gui_monitor->checkHzLabels_[i].old_freq, gui_monitor->checkHzLabels_[i].target_freq, gui_monitor->checkHzLabels_[i].lbl, gui_monitor->checkHzLabels_[i].status);
      gui_monitor->updateValLabelSignalCaller(gui_monitor->checkValLbl_.val, gui_monitor->checkValLbl_.targetV1, gui_monitor->checkValLbl_.targetV2, gui_monitor->checkValLbl_.lbl);
    }
  }

}
