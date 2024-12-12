// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__CORE_COMPONENT_INTERFACE_SPECS__PLANNING_HPP_
#define AUTOWARE__CORE_COMPONENT_INTERFACE_SPECS__PLANNING_HPP_

#include <autoware/core_component_interface_specs/base.hpp>
#include <rclcpp/qos.hpp>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>

namespace autoware::core_component_interface_specs::planning
{

struct LaneletRoute : InterfaceBase
{
  using Message = autoware_planning_msgs::msg::LaneletRoute;
  static constexpr char name[] = "/planning/mission_planning/route_selector/main/route";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
};

struct Trajectory : InterfaceBase
{
  using Message = autoware_planning_msgs::msg::Trajectory;
  static constexpr char name[] = "/planning/scenario_planning/trajectory";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

}  // namespace autoware::core_component_interface_specs::planning

#endif  // AUTOWARE__CORE_COMPONENT_INTERFACE_SPECS__PLANNING_HPP_
