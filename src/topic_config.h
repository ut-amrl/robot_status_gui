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
\file    topic_config.h
\author  Kyle Vedder, (C) 2020
*/
//========================================================================

#include <ros/ros.h>

#include <iostream>
#include <string>
#include <vector>

struct TopicConfig {
  std::string human_name;
  std::string start_command;
  std::string stop_command;
  std::string topic_name;
  std::string topic_type;
  int target_freq;
  int current_freq;
  int update_count;
  TopicConfig() = delete;
  explicit TopicConfig(std::string line);

  template <typename T>
  void UpdateFrequency(const T& msg) {
    update_count++;
  }

  void ComputeResults(const float loop_rate);

  void RunStartCommand() const;
  void RunStopCommand() const;

 private:
  std::pair<bool, std::string> GetNextBlock(std::string* line) const;
};

struct TopicConfigManager {
  std::vector<TopicConfig> configs;

  TopicConfigManager() = delete;
  explicit TopicConfigManager(const std::string& config_file_path);

  std::vector<ros::Subscriber> SetupSubscribers(ros::NodeHandle* n);

  void ComputeResults(const float loop_rate);
};
