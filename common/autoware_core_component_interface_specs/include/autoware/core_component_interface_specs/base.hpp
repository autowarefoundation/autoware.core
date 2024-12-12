// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__CORE_COMPONENT_INTERFACE_SPECS__BASE_HPP_
#define AUTOWARE__CORE_COMPONENT_INTERFACE_SPECS__BASE_HPP_

#include <rclcpp/qos.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_map_msgs/msg/map_projector_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace autoware::core_component_interface_specs
{

struct InterfaceBase
{
  static constexpr char name[] = "";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

  auto get_qos() const
  {
    return rclcpp::QoS{depth}.reliability(reliability).durability(durability);
  }
};

}  // namespace autoware::core_component_interface_specs

#endif  // AUTOWARE__CORE_COMPONENT_INTERFACE_SPECS__BASE_HPP_
