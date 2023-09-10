// Copyright 2022 The Autoware Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware_control_center/node_registry.hpp"

#include <rclcpp/rclcpp.hpp>

namespace autoware_control_center
{
void NodeRegistry::register_node(
  const std::string & name, const unique_identifier_msgs::msg::UUID & uuid)
{
  if (is_registered(name)) {
    RCLCPP_WARN(
      rclcpp::get_logger("NodeRegistry"), "Node %s is already registered. Ignoring.", name.c_str());
    return;
  }

  autoware_node_info_map_[name] = {rclcpp::Clock().now(), name, uuid};

  RCLCPP_INFO(rclcpp::get_logger("NodeRegistry"), "Node %s is registered.", name.c_str());
}

void NodeRegistry::unregister_node(const std::string & name)
{
  if (!is_registered(name)) {
    RCLCPP_WARN(
      rclcpp::get_logger("NodeRegistry"), "Node %s is not registered. Ignoring.", name.c_str());
    return;
  }

  autoware_node_info_map_.erase(name);

  RCLCPP_INFO(rclcpp::get_logger("NodeRegistry"), "Node %s is unregistered.", name.c_str());
}

bool NodeRegistry::is_registered(const std::string & name) const
{
  return autoware_node_info_map_.find(name) != autoware_node_info_map_.end();
}

std::optional<unique_identifier_msgs::msg::UUID> NodeRegistry::get_uuid(
  const std::string & name) const
{
  if (!is_registered(name)) {
    return std::nullopt;
  }

  return autoware_node_info_map_.at(name).uuid;
}

}  // namespace autoware_control_center
