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

#ifndef AUTOWARE_CONTROL_CENTER__NODE_REGISTRY_HPP_
#define AUTOWARE_CONTROL_CENTER__NODE_REGISTRY_HPP_

#include <rclcpp/rclcpp.hpp>

#include <unique_identifier_msgs/msg/uuid.hpp>

#include <optional>
#include <string>
#include <unordered_map>

namespace autoware_control_center
{
struct AutowareNodeInfo
{
  rclcpp::Time time_registering;
  std::string name;
  unique_identifier_msgs::msg::UUID uuid;
};

class NodeRegistry
{
public:
  NodeRegistry() = default;

  void register_node(const std::string & name, const unique_identifier_msgs::msg::UUID & uuid);

  void unregister_node(const std::string & name);

  bool is_registered(const std::string & name) const;

  std::optional<unique_identifier_msgs::msg::UUID> get_uuid(const std::string & name) const;

private:
  std::unordered_map<std::string, AutowareNodeInfo> autoware_node_info_map_;
};
}  // namespace autoware_control_center

#endif  // AUTOWARE_CONTROL_CENTER__NODE_REGISTRY_HPP_
