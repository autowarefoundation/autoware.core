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

#ifndef AUTOWARE_VERSION_MANAGER_CORE_HPP_
#define AUTOWARE_VERSION_MANAGER_CORE_HPP_

#include "version_types.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_system_msgs/srv/get_version.hpp>

namespace autoware_version_manager
{
class AutowareVersionManagerNode : public rclcpp::Node
{
public:
  explicit AutowareVersionManagerNode(const rclcpp::NodeOptions & node_options);

private:
  VersionAutoware version_autoware_;
  VersionInterface version_component_interface_;
};

}  // namespace autoware_version_manager

#endif  // AUTOWARE_VERSION_MANAGER_CORE_HPP_
