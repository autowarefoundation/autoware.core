// Copyright 2024 TIER IV, Inc.
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

#include "autoware/path_generator/utils.hpp"
#include "autoware_test_utils/autoware_test_utils.hpp"
#include "autoware_test_utils/mock_data_parser.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>

#include <gtest/gtest.h>

namespace autoware::path_generator
{
class UtilsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto lanelet_map_path = autoware::test_utils::get_absolute_path_to_lanelet_map(
      "autoware_test_utils", "lanelet2_map.osm");
    const auto map_bin_msg = autoware::test_utils::make_map_bin_msg(lanelet_map_path);

    planner_data_.lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
    lanelet::utils::conversion::fromBinMsg(
      map_bin_msg, planner_data_.lanelet_map_ptr, &planner_data_.traffic_rules_ptr,
      &planner_data_.routing_graph_ptr);

    const auto route_path = autoware::test_utils::get_absolute_path_to_route(
      "autoware_path_generator", "route_data.yaml");
    const auto route =
      autoware::test_utils::parse<std::optional<autoware_planning_msgs::msg::LaneletRoute>>(
        route_path);
    if (!route) {
      throw std::runtime_error(
        "Failed to parse YAML file: " + route_path + ". The file might be corrupted.");
    }

    const auto path_path =
      autoware::test_utils::get_absolute_path_to_route("autoware_path_generator", "path_data.yaml");
    try {
      path_ = autoware::test_utils::parse<
                std::optional<autoware_internal_planning_msgs::msg::PathWithLaneId>>(path_path)
                .value();
    } catch (const std::exception &) {
      throw std::runtime_error(
        "Failed to parse YAML file: " + path_path + ". The file might be corrupted.");
    }

    planner_data_.route_frame_id = route->header.frame_id;
    planner_data_.goal_pose = route->goal_pose;

    planner_data_.route_lanelets.clear();
    planner_data_.preferred_lanelets.clear();
    planner_data_.start_lanelets.clear();
    planner_data_.goal_lanelets.clear();

    size_t primitives_num = 0;
    for (const auto & route_section : route->segments) {
      primitives_num += route_section.primitives.size();
    }
    planner_data_.route_lanelets.reserve(primitives_num);

    for (const auto & route_section : route->segments) {
      for (const auto & primitive : route_section.primitives) {
        const auto id = primitive.id;
        const auto & lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(id);
        planner_data_.route_lanelets.push_back(lanelet);
        if (id == route_section.preferred_primitive.id) {
          planner_data_.preferred_lanelets.push_back(lanelet);
        }
      }
    }

    const auto set_lanelets_from_segment =
      [&](
        const autoware_planning_msgs::msg::LaneletSegment & segment,
        lanelet::ConstLanelets & lanelets) {
        lanelets.reserve(segment.primitives.size());
        for (const auto & primitive : segment.primitives) {
          const auto & lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(primitive.id);
          lanelets.push_back(lanelet);
        }
      };
    set_lanelets_from_segment(route->segments.front(), planner_data_.start_lanelets);
    set_lanelets_from_segment(route->segments.back(), planner_data_.goal_lanelets);
  }

  lanelet::ConstLanelet get_lanelet_closest_to_pose(const geometry_msgs::msg::Pose & pose) const
  {
    lanelet::ConstLanelet lanelet;
    if (!lanelet::utils::query::getClosestLanelet(
          planner_data_.preferred_lanelets, pose, &lanelet)) {
      throw std::runtime_error("Failed to get the closest lanelet to the given point.");
    }
    return lanelet;
  }

  PlannerData planner_data_;
  autoware_internal_planning_msgs::msg::PathWithLaneId path_;
};
}  // namespace autoware::path_generator
