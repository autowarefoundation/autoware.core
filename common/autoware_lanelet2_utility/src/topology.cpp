// Copyright 2025 TIER IV, Inc.
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

#include <autoware_lanelet2_utility/topology.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <vector>

namespace autoware::lanelet2_utility
{
std::optional<lanelet::ConstLanelet> left_lanelet(
  const lanelet::ConstLanelet & lanelet,
  const lanelet::routing::RoutingGraphConstPtr routing_graph);

std::optional<lanelet::ConstLanelet> right_lanelet(
  const lanelet::ConstLanelet & lanelet,
  const lanelet::routing::RoutingGraphConstPtr routing_graph);

std::optional<lanelet::ConstLanelet> left_opposite_lanelet(
  const lanelet::ConstLanelet & lanelet,
  const lanelet::routing::RoutingGraphConstPtr routing_graph);

std::optional<lanelet::ConstLanelet> right_opposite_lanelet(
  const lanelet::ConstLanelet & lanelet,
  const lanelet::routing::RoutingGraphConstPtr routing_graph);

std::optional<lanelet::ConstLanelet> leftmost_lanelet(
  const lanelet::ConstLanelet & lanelet,
  const lanelet::routing::RoutingGraphConstPtr routing_graph);

std::optional<lanelet::ConstLanelet> rightmost_lanelet(
  const lanelet::ConstLanelet & lanelet,
  const lanelet::routing::RoutingGraphConstPtr routing_graph);

lanelet::ConstLanelets left_lanelets(
  const lanelet::ConstLanelet & lanelet, const lanelet::routing::RoutingGraphConstPtr routing_graph,
  const bool include_opposite, const bool invert_opposite_lane);

lanelet::ConstLanelets right_lanelets(
  const lanelet::ConstLanelet & lanelet, const lanelet::routing::RoutingGraphConstPtr routing_graph,
  const bool include_opposite, const bool invert_opposite_lane);

lanelet::ConstLanelets following_lanelets(
  const lanelet::ConstLanelet & lanelet,
  const lanelet::routing::RoutingGraphConstPtr routing_graph);

lanelet::ConstLanelets previous_lanelets(
  const lanelet::ConstLanelet & lanelet,
  const lanelet::routing::RoutingGraphConstPtr routing_graph);

lanelet::ConstLanelets sibling_lanelets(
  const lanelet::ConstLanelet & lanelet,
  const lanelet::routing::RoutingGraphConstPtr routing_graph);

lanelet::ConstLanelets from_ids(
  const lanelet::LaneletMapConstPtr lanelet_map_ptr, const std::vector<lanelet::Id> & ids);
}  // namespace autoware::lanelet2_utility
