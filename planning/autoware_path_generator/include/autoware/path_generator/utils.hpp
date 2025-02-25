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

#ifndef AUTOWARE__PATH_GENERATOR__UTILS_HPP_
#define AUTOWARE__PATH_GENERATOR__UTILS_HPP_

#include "autoware/path_generator/common_structs.hpp"

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>

#include <optional>
#include <utility>
#include <vector>

namespace autoware::path_generator
{
using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using autoware_internal_planning_msgs::msg::PathWithLaneId;

namespace utils
{
/**
 * @brief get lanelets within route that are in specified distance forward or backward from
 * current position
 * @param lanelet lanelet where ego vehicle is on
 * @param planner_data planner data
 * @param current_pose current pose of ego vehicle
 * @param backward_distance backward distance from ego vehicle
 * @param forward_distance forward distance from ego vehicle
 * @return lanelets in range (std::nullopt if target lanelet is not within route)
 */
std::optional<lanelet::ConstLanelets> get_lanelets_within_route(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data,
  const geometry_msgs::msg::Pose & current_pose, const double backward_distance,
  const double forward_distance);

/**
 * @brief get lanelets within route that are in specified distance backward from target
 * lanelet
 * @param lanelet target lanelet
 * @param planner_data planner data
 * @param distance backward distance from beginning of target lanelet
 * @return lanelets in range (std::nullopt if target lanelet is not within route)
 */
std::optional<lanelet::ConstLanelets> get_lanelets_within_route_up_to(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data, const double distance);

/**
 * @brief get lanelets within route that are in specified distance forward from target
 * lanelet
 * @param lanelet target lanelet
 * @param planner_data planner data
 * @param distance forward distance from end of target lanelet
 * @return lanelets in range (std::nullopt if target lanelet is not within route)
 */
std::optional<lanelet::ConstLanelets> get_lanelets_within_route_after(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data, const double distance);

/**
 * @brief get previous lanelet within route
 * @param lanelet target lanelet
 * @param planner_data planner data
 * @return lanelets in range (std::nullopt if previous lanelet is not found or not
 * within route)
 */
std::optional<lanelet::ConstLanelet> get_previous_lanelet_within_route(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data);

/**
 * @brief get next lanelet within route
 * @param lanelet target lanelet
 * @param planner_data planner data
 * @return lanelets in range (std::nullopt if next lanelet is not found or not
 * within route)
 */
std::optional<lanelet::ConstLanelet> get_next_lanelet_within_route(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data);

/**
 * @brief get waypoints in lanelet sequence and group them
 * @param lanelet_sequence lanelet sequence
 * @param lanelet_map lanelet map to get waypoints
 * @param group_separation_threshold maximum distance between waypoints to belong to same
 * group (see figure in README)
 * @param interval_margin_ratio ratio to expand interval bound of group according to the
 * lateral distance of first and last point of group
 * @return waypoint groups (each group is a pair of points and its interval)
 */
std::vector<std::pair<lanelet::ConstPoints3d, std::pair<double, double>>> get_waypoint_groups(
  const lanelet::LaneletSequence & lanelet_sequence, const lanelet::LaneletMap & lanelet_map,
  const double group_separation_threshold, const double interval_margin_ratio);

/**
 * @brief get position of first self-intersection (point where return
 * path intersects outward path) of lanelet sequence in arc length
 * @param lanelet_sequence target lanelet sequence (both left and right bound are
 * considered)
 * @param s_start longitudinal distance of point to start searching for self-intersections
 * @param s_end longitudinal distance of point to end search
 * @return longitudinal distance of self-intersecting point (std::nullopt if no
 * self-intersection)
 */
std::optional<double> get_first_self_intersection_arc_length(
  const lanelet::LaneletSequence & lanelet_sequence, const double s_start, const double s_end);

/**
 * @brief get position of first self-intersection of line string in arc length
 * @param line_string target line string
 * @param s_start longitudinal distance of point to start searching for self-intersections
 * @param s_end longitudinal distance of point to end search
 * @return longitudinal distance of self-intersecting point (std::nullopt if no
 * self-intersection)
 */
std::optional<double> get_first_self_intersection_arc_length(
  const lanelet::BasicLineString2d & line_string, const double s_start, const double s_end);

/**
 * @brief get bound of path cropped within specified range
 * @param lanelet_bound original bound of lanelet
 * @param lanelet_centerline centerline of lanelet
 * @param s_start longitudinal distance of start of bound
 * @param s_end longitudinal distance of end of bound
 * @return cropped bound
 */
std::vector<geometry_msgs::msg::Point> get_path_bound(
  const lanelet::CompoundLineString2d & lanelet_bound,
  const lanelet::CompoundLineString2d & lanelet_centerline, const double s_start,
  const double s_end);
}  // namespace utils
}  // namespace autoware::path_generator

#endif  // AUTOWARE__PATH_GENERATOR__UTILS_HPP_
