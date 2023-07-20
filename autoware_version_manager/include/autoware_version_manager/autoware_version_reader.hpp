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

#ifndef AUTOWARE_VERSION_MANAGER__AUTOWARE_VERSION_READER_HPP_
#define AUTOWARE_VERSION_MANAGER__AUTOWARE_VERSION_READER_HPP_

#include "autoware_version_manager/version_types.hpp"

#include <rclcpp/rclcpp.hpp>

#include <optional>

namespace autoware_version_manager
{
class AutowareVersionReaderNode : public rclcpp::Node
{
public:
  explicit AutowareVersionReaderNode(const rclcpp::NodeOptions & node_options);

  const VersionAutoware & get_version_autoware() const;
  const VersionInterface & get_version_component_interface() const;

private:
  VersionAutoware version_autoware_;
  VersionInterface version_component_interface_;
};

std::optional<VersionAutoware> get_version_autoware_directly();

}  // namespace autoware_version_manager

#endif  // AUTOWARE_VERSION_MANAGER__AUTOWARE_VERSION_READER_HPP_
