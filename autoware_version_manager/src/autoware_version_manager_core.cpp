// Copyright 2023 The Autoware Foundation
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

#include "include/autoware_version_manager_core.hpp"

#include "include/parse_version.hpp"
#include "include/version_types.hpp"

#include <rclcpp/rclcpp.hpp>

namespace autoware_version_manager
{

AutowareVersionManagerNode::AutowareVersionManagerNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("autoware_version_manager_node", node_options)
{
  const std::filesystem::path path_version_autoware =
    declare_parameter<std::string>("path_version_autoware");
  const std::filesystem::path path_version_component_interface =
    declare_parameter<std::string>("path_version_component_interface");

  try {
    version_autoware_ = parse_version::parse_autoware_version(path_version_autoware);
    version_component_interface_ =
      parse_version::parse_interface_version(path_version_component_interface);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to parse version: %s", e.what());
    exit(EXIT_FAILURE);
  }

  // print
  RCLCPP_INFO(
    get_logger(), "Autoware version: %d.%02d.%d", version_autoware_.year, version_autoware_.month,
    version_autoware_.micro);

  RCLCPP_INFO(
    get_logger(), "Component interface version: %d.%d.%d", version_component_interface_.major,
    version_component_interface_.minor, version_component_interface_.patch);
}

}  // namespace autoware_version_manager

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware_version_manager::AutowareVersionManagerNode)
