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
\file    topic_config.cpp
\author  Kyle Vedder, (C) 2020
*/
//========================================================================
#include "topic_config.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <fstream>
#include <iostream>
#include <string>

TopicConfig::TopicConfig(std::string line)
    : target_freq(0), current_freq(0), update_count(0) {
  const auto full_line = line;
  {
    auto human_name_res = GetNextBlock(&line);
    human_name = human_name_res.second;
    if (!human_name_res.first) {
      std::cerr << "Failed to parse Human Readable name out of line "
                << full_line << std::endl;
    }
  }

  {
    auto start_command_res = GetNextBlock(&line);
    start_command = start_command_res.second;
    if (!start_command_res.first) {
      std::cerr << "Failed to parse startup command out of line " << full_line
                << std::endl;
    }
  }

  {
    auto stop_command_res = GetNextBlock(&line);
    stop_command = stop_command_res.second;
    if (!stop_command_res.first) {
      std::cerr << "Failed to parse stop command out of line " << full_line
                << std::endl;
    }
  }

  {
    auto topic_name_res = GetNextBlock(&line);
    topic_name = topic_name_res.second;
    if (!topic_name_res.first) {
      std::cerr << "Failed to parse topic name out of line " << full_line
                << std::endl;
    }
  }

  {
    auto topic_type_res = GetNextBlock(&line);
    topic_type = topic_type_res.second;
    if (!topic_type_res.first) {
      std::cerr << "Failed to parse topic type out of line " << full_line
                << std::endl;
    }
  }

  {
    auto target_freq_res = GetNextBlock(&line);
    if (!target_freq_res.first) {
      std::cerr << "Failed to parse target freq out of line " << full_line
                << std::endl;
    }
    try {
      target_freq = std::stoi(target_freq_res.second);
    } catch (std::invalid_argument const& e) {
      std::cerr << "Failed to parse target freq value out of line" << std::endl;
      target_freq = 0;
    } catch (std::out_of_range const& e) {
      std::cerr << "Failed to parse target freq value out of line" << std::endl;
      target_freq = 0;
    }
  }
}

TopicConfigManager::TopicConfigManager(const std::string& config_file_path) {
  std::ifstream file(config_file_path);
  if (!file.is_open()) {
    std::cerr << "Failed to open config file " << config_file_path << std::endl;
    return;
  }
  std::string line;
  while (std::getline(file, line)) {
    if (line[0] == '#' || line.empty()) {
      continue;
    }
    configs.push_back(TopicConfig(line));
  }

  std::cout << "Loaded configs for " << configs.size() 
            << " topics." << std::endl;
}

void TopicConfig::ComputeResults(const float loop_rate) {
  current_freq = static_cast<int>(update_count * loop_rate);
  update_count = 0;
}

std::pair<bool, std::string> TopicConfig::GetNextBlock(
    std::string* line) const {
  const auto start_idx = line->find("\"");
  if (start_idx == std::string::npos) {
    return {false, ""};
  }
  line->erase(0, start_idx + 1);
  const auto end_idx = line->find("\"");
  if (end_idx == std::string::npos) {
    return {false, ""};
  }
  std::string block = line->substr(0, end_idx);
  line->erase(0, end_idx + 1);
  return {true, block};
}

std::vector<ros::Subscriber> TopicConfigManager::SetupSubscribers(
    ros::NodeHandle* n) {
  std::vector<ros::Subscriber> subscribers;
  for (TopicConfig& cfg : configs) {
    const auto& type_str = cfg.topic_type;
    if (type_str == "std_msgs/String") {
      subscribers.push_back(
          n->subscribe(cfg.topic_name, 10,
                       &TopicConfig::UpdateFrequency<std_msgs::String>, &cfg));
    } else if (type_str == "geometry_msgs/PoseStamped") {
      subscribers.push_back(n->subscribe(
          cfg.topic_name, 10,
          &TopicConfig::UpdateFrequency<geometry_msgs::PoseStamped>, &cfg));
    } else if (type_str == "nav_msgs/Path") {
      subscribers.push_back(
          n->subscribe(cfg.topic_name, 10,
                       &TopicConfig::UpdateFrequency<nav_msgs::Path>, &cfg));
    } else if (type_str == "sensor_msgs/Image") {
      subscribers.push_back(n->subscribe(
          cfg.topic_name, 10, &TopicConfig::UpdateFrequency<sensor_msgs::Image>,
          &cfg));
    } else if (type_str == "geometry_msgs/Twist") {
      subscribers.push_back(n->subscribe(
          cfg.topic_name, 10,
          &TopicConfig::UpdateFrequency<geometry_msgs::Twist>, &cfg));
    } else if (type_str == "sensor_msgs/LaserScan") {
      subscribers.push_back(n->subscribe(
          cfg.topic_name, 10,
          &TopicConfig::UpdateFrequency<sensor_msgs::LaserScan>, &cfg));
    } else {
      std::cerr << "Unknown type: " << type_str << std::endl;
    }
  }
  return subscribers;
}

void TopicConfigManager::ComputeResults(const float loop_rate) {
  for (TopicConfig& cfg : configs) {
    cfg.ComputeResults(loop_rate);
  }
}