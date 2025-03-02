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

#include "autoware/trajectory/utils/find_intervals.hpp"

#include <autoware/motion_utils/constants.hpp>
#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/trajectory/path_point_with_lane_id.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/unit_conversion.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <utility>
#include <vector>

namespace autoware::path_generator
{
namespace utils
{
namespace
{
template <typename T>
bool exists(const std::vector<T> & vec, const T & item)
{
  return std::find(vec.begin(), vec.end(), item) != vec.end();
}

std::optional<double> calc_interpolated_z(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & input,
  const geometry_msgs::msg::Point target_pos, const size_t seg_idx)
{
  // Check if input is empty
  if (input.points.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("path_generator"), "Input is empty");
    return std::nullopt;
  }
  // Check if seg_idx is invalid: -2 is to avoid out of bounds error by seg_idx + 1
  if (seg_idx > input.points.size() - 2) {
    RCLCPP_WARN(rclcpp::get_logger("path_generator"),
                 "seg_idx: %zu is invalid for input size: %zu.\n"
                 "Use the z that of the last point as the interpolated z.",
                 seg_idx, input.points.size());

    // Return the z of the last point if interpolation is not possible
    return input.points.back().point.pose.position.z;
  }

  try {
    const double closest_to_target_dist = autoware::motion_utils::calcSignedArcLength(
      input.points, input.points.at(seg_idx).point.pose.position, target_pos);

    const double seg_dist =
      autoware::motion_utils::calcSignedArcLength(input.points, seg_idx, seg_idx + 1);

    const double closest_z = input.points.at(seg_idx).point.pose.position.z;
    const double next_z = input.points.at(seg_idx + 1).point.pose.position.z;

    return std::abs(seg_dist) < 1e-6
             ? next_z
             : closest_z + (next_z - closest_z) * closest_to_target_dist / seg_dist;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("path_generator"), "Error: %s", e.what());
    return std::nullopt;
  }
}

std::optional<double> calc_interpolated_velocity(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & input, const size_t seg_idx)
{
  // Check if input is empty
  if (input.points.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("path_generator"), "Input is empty");
    return std::nullopt;
  }
  // Check if seg_idx is invalid: -2 is to avoid out of bounds error by seg_idx + 1
  if (seg_idx > input.points.size() - 2) {
    RCLCPP_WARN(rclcpp::get_logger("path_generator"),
                 "seg_idx: %zu is invalid for input size: %zu.\n"
                 "Use the velocity that of the last point as the interpolated velocity.",
                 seg_idx, input.points.size());

    // Return the velocity of the last point if interpolation is not possible
    return input.points.back().point.longitudinal_velocity_mps;
  }

  try {
    const double seg_dist =
      autoware::motion_utils::calcSignedArcLength(input.points, seg_idx, seg_idx + 1);

    const double closest_vel = input.points.at(seg_idx).point.longitudinal_velocity_mps;
    const double next_vel = input.points.at(seg_idx + 1).point.longitudinal_velocity_mps;
    const double interpolated_vel = std::abs(seg_dist) < 1e-06 ? next_vel : closest_vel;
    return interpolated_vel;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("path_generator"), "Error: %s", e.what());
    return std::nullopt;
  }
}

}  // namespace

std::optional<lanelet::ConstLanelets> get_lanelets_within_route(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data,
  const geometry_msgs::msg::Pose & current_pose, const double backward_distance,
  const double forward_distance)
{
  if (!exists(planner_data.route_lanelets, lanelet)) {
    return std::nullopt;
  }

  const auto arc_coordinates = lanelet::utils::getArcCoordinates({lanelet}, current_pose);
  const auto lanelet_length = lanelet::utils::getLaneletLength2d(lanelet);

  const auto backward_lanelets = get_lanelets_within_route_up_to(
    lanelet, planner_data, backward_distance - arc_coordinates.length);
  if (!backward_lanelets) {
    return std::nullopt;
  }

  const auto forward_lanelets = get_lanelets_within_route_after(
    lanelet, planner_data, forward_distance - (lanelet_length - arc_coordinates.length));
  if (!forward_lanelets) {
    return std::nullopt;
  }

  lanelet::ConstLanelets lanelets(std::move(*backward_lanelets));
  lanelets.push_back(lanelet);
  std::move(forward_lanelets->begin(), forward_lanelets->end(), std::back_inserter(lanelets));

  return lanelets;
}

std::optional<lanelet::ConstLanelets> get_lanelets_within_route_up_to(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data, const double distance)
{
  if (!exists(planner_data.route_lanelets, lanelet)) {
    return std::nullopt;
  }

  lanelet::ConstLanelets lanelets{};
  auto current_lanelet = lanelet;
  auto length = 0.;

  while (rclcpp::ok() && length < distance) {
    const auto prev_lanelet = get_previous_lanelet_within_route(current_lanelet, planner_data);
    if (!prev_lanelet) {
      break;
    }

    lanelets.push_back(*prev_lanelet);
    current_lanelet = *prev_lanelet;
    length += lanelet::utils::getLaneletLength2d(*prev_lanelet);
  }

  std::reverse(lanelets.begin(), lanelets.end());
  return lanelets;
}

std::optional<lanelet::ConstLanelets> get_lanelets_within_route_after(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data, const double distance)
{
  if (!exists(planner_data.route_lanelets, lanelet)) {
    return std::nullopt;
  }

  lanelet::ConstLanelets lanelets{};
  auto current_lanelet = lanelet;
  auto length = 0.;

  while (rclcpp::ok() && length < distance) {
    const auto next_lanelet = get_next_lanelet_within_route(current_lanelet, planner_data);
    if (!next_lanelet) {
      break;
    }

    lanelets.push_back(*next_lanelet);
    current_lanelet = *next_lanelet;
    length += lanelet::utils::getLaneletLength2d(*next_lanelet);
  }

  return lanelets;
}

std::optional<lanelet::ConstLanelet> get_previous_lanelet_within_route(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data)
{
  if (exists(planner_data.start_lanelets, lanelet)) {
    return std::nullopt;
  }

  const auto prev_lanelets = planner_data.routing_graph_ptr->previous(lanelet);
  if (prev_lanelets.empty()) {
    return std::nullopt;
  }

  const auto prev_lanelet_itr = std::find_if(
    prev_lanelets.cbegin(), prev_lanelets.cend(),
    [&](const lanelet::ConstLanelet & l) { return exists(planner_data.route_lanelets, l); });
  if (prev_lanelet_itr == prev_lanelets.cend()) {
    return std::nullopt;
  }
  return *prev_lanelet_itr;
}

std::optional<lanelet::ConstLanelet> get_next_lanelet_within_route(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data)
{
  if (planner_data.preferred_lanelets.empty()) {
    return std::nullopt;
  }

  if (exists(planner_data.goal_lanelets, lanelet)) {
    return std::nullopt;
  }

  const auto next_lanelets = planner_data.routing_graph_ptr->following(lanelet);
  if (
    next_lanelets.empty() ||
    next_lanelets.front().id() == planner_data.preferred_lanelets.front().id()) {
    return std::nullopt;
  }

  const auto next_lanelet_itr = std::find_if(
    next_lanelets.cbegin(), next_lanelets.cend(),
    [&](const lanelet::ConstLanelet & l) { return exists(planner_data.route_lanelets, l); });
  if (next_lanelet_itr == next_lanelets.cend()) {
    return std::nullopt;
  }
  return *next_lanelet_itr;
}

std::optional<double> get_first_self_intersection_arc_length(
  const lanelet::LaneletSequence & lanelet_sequence, const double s_start, const double s_end)
{
  const auto s_left = get_first_self_intersection_arc_length(
    lanelet_sequence.leftBound2d().basicLineString(), s_start, s_end);
  const auto s_right = get_first_self_intersection_arc_length(
    lanelet_sequence.rightBound2d().basicLineString(), s_start, s_end);

  if (s_left && s_right) {
    return std::min(*s_left, *s_right);
  }
  return s_left ? s_left : s_right;
}

std::optional<double> get_first_self_intersection_arc_length(
  const lanelet::BasicLineString2d & line_string, const double s_start, const double s_end)
{
  const auto tree = lanelet::geometry::internal::makeIndexedSegmenTree(line_string);
  std::optional<std::pair<size_t, double>> last_intersection = std::nullopt;
  auto s = 0.;

  for (size_t i = 1; i < line_string.size() - 1; ++i) {
    if (last_intersection && i == last_intersection->first) {
      return s + last_intersection->second;
    }
    s += lanelet::geometry::distance2d(line_string.at(i - 1), line_string.at(i));
    if (s < s_start) {
      continue;
    }
    if (s > s_end) {
      break;
    }
    const auto self_intersections = lanelet::geometry::internal::getSelfIntersectionsAt(
      tree, 0, lanelet::BasicSegment2d{line_string.at(i - 1), line_string.at(i)});
    if (self_intersections.empty()) {
      continue;
    }
    last_intersection = {
      self_intersections.front().lastSegment.idx, self_intersections.front().lastSegment.s};
  }

  return std::nullopt;
}

std::vector<std::pair<lanelet::ConstPoints3d, std::pair<double, double>>> get_waypoint_groups(
  const lanelet::LaneletSequence & lanelet_sequence, const lanelet::LaneletMap & lanelet_map,
  const double group_separation_threshold, const double interval_margin_ratio)
{
  std::vector<std::pair<lanelet::ConstPoints3d, std::pair<double, double>>> waypoint_groups{};

  const auto get_interval_bound =
    [&](const lanelet::ConstPoint3d & point, const double lateral_distance_factor) {
      const auto arc_coordinates = lanelet::geometry::toArcCoordinates(
        lanelet_sequence.centerline2d(), lanelet::utils::to2D(point));
      return arc_coordinates.length + lateral_distance_factor * std::abs(arc_coordinates.distance);
    };

  for (const auto & lanelet : lanelet_sequence) {
    if (!lanelet.hasAttribute("waypoints")) {
      continue;
    }

    const auto waypoints_id = lanelet.attribute("waypoints").asId().value();
    const auto & waypoints = lanelet_map.lineStringLayer.get(waypoints_id);

    if (
      waypoint_groups.empty() ||
      lanelet::geometry::distance2d(waypoint_groups.back().first.back(), waypoints.front()) >
        group_separation_threshold) {
      waypoint_groups.emplace_back().second.first =
        get_interval_bound(waypoints.front(), -interval_margin_ratio);
    }
    waypoint_groups.back().second.second =
      get_interval_bound(waypoints.back(), interval_margin_ratio);

    waypoint_groups.back().first.insert(
      waypoint_groups.back().first.end(), waypoints.begin(), waypoints.end());
  }

  return waypoint_groups;
}

std::vector<geometry_msgs::msg::Point> get_path_bound(
  const lanelet::CompoundLineString2d & lanelet_bound,
  const lanelet::CompoundLineString2d & lanelet_centerline, const double s_start,
  const double s_end)
{
  const auto path_start_point =
    lanelet::geometry::interpolatedPointAtDistance(lanelet_centerline, s_start);
  const auto path_end_point =
    lanelet::geometry::interpolatedPointAtDistance(lanelet_centerline, s_end);

  auto s_bound_start =
    lanelet::geometry::toArcCoordinates(
      lanelet::utils::to2D(lanelet_bound.lineStrings().front()), path_start_point)
      .length;
  auto s_bound_end = lanelet::geometry::toArcCoordinates(lanelet_bound, path_end_point).length;

  std::vector<geometry_msgs::msg::Point> path_bound{};
  auto s = 0.;

  for (auto it = lanelet_bound.begin(); it != std::prev(lanelet_bound.end()); ++it) {
    s += lanelet::geometry::distance2d(*it, *std::next(it));
    if (s < s_bound_start) {
      continue;
    }

    if (path_bound.empty()) {
      const auto interpolated_point =
        lanelet::geometry::interpolatedPointAtDistance(lanelet_bound, s_bound_start);
      path_bound.push_back(
        lanelet::utils::conversion::toGeomMsgPt(lanelet::utils::to3D(interpolated_point)));
    } else {
      path_bound.push_back(lanelet::utils::conversion::toGeomMsgPt(*it));
    }

    if (s >= s_bound_end) {
      const auto interpolated_point =
        lanelet::geometry::interpolatedPointAtDistance(lanelet_bound, s_bound_end);
      path_bound.push_back(
        lanelet::utils::conversion::toGeomMsgPt(lanelet::utils::to3D(interpolated_point)));
      break;
    }
  }

  return path_bound;
}

std::optional<PathWithLaneId> set_goal(
  const double search_radius_range, const PathWithLaneId & input,
  const geometry_msgs::msg::Pose & goal)
{
  try {
    if (input.points.empty()) {
      return std::nullopt;
    }

    PathWithLaneId output;

    // Calculate refined_goal with interpolation
    // Note: `goal` does not have valid z, that will be calculated by interpolation here
    PathPointWithLaneId refined_goal{};
    const size_t closest_seg_idx_for_goal =
      autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
        input.points, goal, 3.0, M_PI_4);

    // Set goal
    refined_goal.point.pose = goal;

    if (auto z = calc_interpolated_z(input, goal.position, closest_seg_idx_for_goal)) {
      refined_goal.point.pose.position.z = *z;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("path_generator"), "Failed to calculate z for goal");
      return std::nullopt;
    }

    refined_goal.point.longitudinal_velocity_mps = 0.0;

    // Calculate pre_refined_goal with interpolation
    // Note: z and velocity are filled
    PathPointWithLaneId pre_refined_goal{};
    constexpr double goal_to_pre_goal_distance = -1.0;
    pre_refined_goal.point.pose =
      autoware_utils::calc_offset_pose(goal, goal_to_pre_goal_distance, 0.0, 0.0);
    const size_t closest_seg_idx_for_pre_goal =
      autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
        input.points, pre_refined_goal.point.pose, 3.0, M_PI_4);

    if (
      auto z = calc_interpolated_z(
        input, pre_refined_goal.point.pose.position, closest_seg_idx_for_pre_goal)) {
      pre_refined_goal.point.pose.position.z = *z;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("path_generator"), "Failed to calculate z for pre_goal");
      return std::nullopt;
    }

    if (auto velocity = calc_interpolated_velocity(input, closest_seg_idx_for_pre_goal)) {
      pre_refined_goal.point.longitudinal_velocity_mps = *velocity;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("path_generator"), "Failed to calculate velocity for pre_goal");
      return std::nullopt;
    }

    // Find min_dist_out_of_circle_index whose distance to goal is longer than search_radius_range
    const auto min_dist_out_of_circle_index =
      autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
        input.points, goal, search_radius_range, M_PI_4);

    // Create output points
    output.points.reserve(output.points.size() + min_dist_out_of_circle_index + 3);
    for (size_t i = 0; i <= min_dist_out_of_circle_index; ++i) {
      output.points.push_back(input.points.at(i));
    }
    output.points.push_back(pre_refined_goal);
    output.points.push_back(refined_goal);

    // Add lane IDs from skipped points to the pre-goal point
    auto & pre_goal = output.points.at(output.points.size() - 2);
    for (size_t i = min_dist_out_of_circle_index + 1; i < input.points.size(); ++i) {
      for (const auto target_lane_id : input.points.at(i).lane_ids) {
        const bool is_lane_id_found =
          std::find(pre_goal.lane_ids.begin(), pre_goal.lane_ids.end(), target_lane_id) !=
          pre_goal.lane_ids.end();
        if (!is_lane_id_found) {
          pre_goal.lane_ids.push_back(target_lane_id);
        }
      }
    }

    // Attach goal lane IDs to the last point
    output.points.back().lane_ids = input.points.back().lane_ids;

    output.left_bound = input.left_bound;
    output.right_bound = input.right_bound;

    return output;
  } catch (std::out_of_range & ex) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("path_generator").get_child("utils"), "failed to set goal: " << ex.what());
    return std::nullopt;
  }
}

const geometry_msgs::msg::Pose refine_goal(
  const geometry_msgs::msg::Pose & goal, const lanelet::ConstLanelet & goal_lanelet)
{
  const auto lanelet_point = lanelet::utils::conversion::toLaneletPoint(goal.position);
  const double distance = boost::geometry::distance(
    goal_lanelet.polygon2d().basicPolygon(), lanelet::utils::to2D(lanelet_point).basicPoint());

  // You are almost at the goal
  if (distance < std::numeric_limits<double>::epsilon()) {
    return goal;
  }

  // Get the closest segment to the goal
  const auto segment = lanelet::utils::getClosestSegment(
    lanelet::utils::to2D(lanelet_point), goal_lanelet.centerline());

  // If the segment is empty, return the original goal.
  if (segment.empty()) {
    return goal;
  }

  geometry_msgs::msg::Pose refined_goal;
  {
    // find position
    const auto p1 = segment.front().basicPoint();
    const auto p2 = segment.back().basicPoint();
    const auto direction_vector = (p2 - p1).normalized();
    const auto p1_to_goal = lanelet_point.basicPoint() - p1;
    const double s = direction_vector.dot(p1_to_goal);
    const auto refined_point = p1 + direction_vector * s;

    refined_goal.position.x = refined_point.x();
    refined_goal.position.y = refined_point.y();
    refined_goal.position.z = refined_point.z();

    // find orientation
    const double yaw = std::atan2(direction_vector.y(), direction_vector.x());
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(0, 0, yaw);
    refined_goal.orientation = tf2::toMsg(tf_quat);
  }
  return refined_goal;
}

PathWithLaneId refine_path_for_goal(
  const double search_radius_range, const PathWithLaneId & input,
  const geometry_msgs::msg::Pose & goal)
{
  PathWithLaneId filtered_path = input;

  filtered_path.points = autoware::motion_utils::removeOverlapPoints(filtered_path.points);

  // Always set zero velocity at the end of path for safety
  if (!filtered_path.points.empty()) {
    filtered_path.points.back().point.longitudinal_velocity_mps = 0.0;
  }

  // If set_goal returns a valid path, return it
  if (const auto path_with_goal = set_goal(search_radius_range, filtered_path, goal)) {
    return *path_with_goal;
  }

  // Otherwise, return the original path with zero velocity at the end
  return filtered_path;
}

std::optional<lanelet::ConstLanelets> extract_lanelets_from_path(
  const PathWithLaneId & refined_path, const std::shared_ptr<const PlannerData> & planner_data)
{
  lanelet::ConstLanelets refined_path_lanelets;
  for (size_t i = 0; i < refined_path.points.size(); ++i) {
    try {
      const auto & path_point = refined_path.points.at(i);

      // TODO(sasakisasaki): It seems sometimes path_point.lane_ids is empty.
      //                     We should check this case.
      int64_t lane_id = path_point.lane_ids.at(0);
      lanelet::ConstLanelet lanelet = planner_data->lanelet_map_ptr->laneletLayer.get(lane_id);

      const bool is_unique =
        std::find(refined_path_lanelets.begin(), refined_path_lanelets.end(), lanelet) ==
        refined_path_lanelets.end();

      if (is_unique) {
        refined_path_lanelets.push_back(lanelet);
      }
    } catch (const std::out_of_range & e) {
      RCLCPP_ERROR(rclcpp::get_logger("path_generator"), "Out of range error: %s", e.what());
      return std::nullopt;
    }

  }
  return refined_path_lanelets;
}

std::optional<lanelet::ConstLanelet> get_goal_lanelet(const PlannerData & planner_data)
{
  const lanelet::Id goal_lane_id = planner_data.goal_lane_id;
  for (const auto & llt : planner_data.route_lanelets) {
    if (llt.id() == goal_lane_id) {
      return llt;
    }
  }
  return std::nullopt;
}

bool is_in_lanelets(const geometry_msgs::msg::Pose & pose, const lanelet::ConstLanelets & lanes)
{
  for (const auto & lane : lanes) {
    if (lanelet::utils::isInLanelet(pose, lane)) {
      return true;
    }
  }
  return false;
}

bool is_path_valid(
  const PathWithLaneId & refined_path, const std::shared_ptr<const PlannerData> & planner_data)
{
  // Extract lanelets from the refined path
  const auto lanelets_opt = extract_lanelets_from_path(refined_path, planner_data);
  if (!lanelets_opt) {
    RCLCPP_ERROR(rclcpp::get_logger("path_generator"), "Failed to extract lanelets from path");
    return false;
  }
  const auto & lanelets = lanelets_opt.value();

  // std::any_of detects whether any point lies outside lanelets
  const bool has_points_outside_lanelet = std::any_of(
    refined_path.points.begin(), refined_path.points.end(),
    [&lanelets](const auto & refined_path_point) {
      return !is_in_lanelets(refined_path_point.point.pose, lanelets);
    });

  // Return true if no points lie outside the extracted lanelets
  return !has_points_outside_lanelet;
}

PathWithLaneId modify_path_for_smooth_goal_connection(
  const PathWithLaneId & path, const std::shared_ptr<const PlannerData> & planner_data)
{
  const auto goal = planner_data->goal_pose;

  geometry_msgs::msg::Pose refined_goal{};
  {
    lanelet::ConstLanelet goal_lanelet;

    // First, polish up the goal pose if possible
    if (const auto goal_lanelet = get_goal_lanelet(*planner_data)) {
      refined_goal = refine_goal(goal, *goal_lanelet);
    } else {
      refined_goal = goal;
    }
  }
  double goal_search_radius{
    planner_data->path_generator_parameters.refine_goal_search_radius_range};

  bool is_valid_path{false};
  PathWithLaneId refined_path;

  // Then, refine the path for the goal
  while (goal_search_radius >= 0 && !is_valid_path) {
    refined_path = refine_path_for_goal(goal_search_radius, path, refined_goal);
    if (is_path_valid(refined_path, planner_data)) {
      is_valid_path = true;
    }
    goal_search_radius -= planner_data->path_generator_parameters.search_radius_decrement;
  }

  // It is better to return the original path if the refined path is not valid
  if (!is_valid_path) {
    return path;
  }
  return refined_path;
}

TurnIndicatorsCommand get_turn_signal(
  const PathWithLaneId & path, const PlannerData & planner_data,
  const geometry_msgs::msg::Pose & current_pose, const double current_vel,
  const double search_distance, const double search_time, const double angle_threshold_deg,
  const double base_link_to_front)
{
  TurnIndicatorsCommand turn_signal;
  turn_signal.command = TurnIndicatorsCommand::NO_COMMAND;

  const lanelet::BasicPoint2d current_point{current_pose.position.x, current_pose.position.y};
  const auto base_search_distance = search_distance + current_vel * search_time;

  std::vector<lanelet::Id> searched_lanelet_ids = {};
  std::optional<double> arc_length_from_vehicle_front_to_lanelet_start = std::nullopt;

  auto calc_arc_length =
    [&](const lanelet::ConstLanelet & lanelet, const lanelet::BasicPoint2d & point) -> double {
    return lanelet::geometry::toArcCoordinates(lanelet.centerline2d(), point).length;
  };

  for (const auto & point : path.points) {
    for (const auto & lane_id : point.lane_ids) {
      if (exists(searched_lanelet_ids, lane_id)) {
        continue;
      }
      searched_lanelet_ids.push_back(lane_id);

      const auto lanelet = planner_data.lanelet_map_ptr->laneletLayer.get(lane_id);
      if (!get_next_lanelet_within_route(lanelet, planner_data)) {
        continue;
      }

      if (
        !arc_length_from_vehicle_front_to_lanelet_start &&
        !lanelet::geometry::inside(lanelet, current_point)) {
        continue;
      }

      if (lanelet.hasAttribute("turn_direction")) {
        turn_signal.command =
          turn_signal_command_map.at(lanelet.attribute("turn_direction").value());

        if (arc_length_from_vehicle_front_to_lanelet_start) {  // ego is in front of lanelet
          if (
            *arc_length_from_vehicle_front_to_lanelet_start >
            lanelet.attributeOr("turn_signal_distance", base_search_distance)) {
            turn_signal.command = TurnIndicatorsCommand::NO_COMMAND;
          }
          return turn_signal;
        }

        // ego is inside lanelet
        const auto required_end_point_opt =
          get_turn_signal_required_end_point(lanelet, angle_threshold_deg);
        if (!required_end_point_opt) continue;
        if (
          calc_arc_length(lanelet, current_point) <=
          calc_arc_length(lanelet, required_end_point_opt.value())) {
          return turn_signal;
        }
      }

      const auto lanelet_length = lanelet::utils::getLaneletLength2d(lanelet);
      if (arc_length_from_vehicle_front_to_lanelet_start) {
        *arc_length_from_vehicle_front_to_lanelet_start += lanelet_length;
      } else {
        arc_length_from_vehicle_front_to_lanelet_start =
          lanelet_length - calc_arc_length(lanelet, current_point) - base_link_to_front;
      }
      break;
    }
  }

  return turn_signal;
}

std::optional<lanelet::ConstPoint2d> get_turn_signal_required_end_point(
  const lanelet::ConstLanelet & lanelet, const double angle_threshold_deg)
{
  std::vector<geometry_msgs::msg::Pose> centerline_poses(lanelet.centerline().size());
  std::transform(
    lanelet.centerline().begin(), lanelet.centerline().end(), centerline_poses.begin(),
    [](const auto & point) {
      geometry_msgs::msg::Pose pose{};
      pose.position = lanelet::utils::conversion::toGeomMsgPt(point);
      return pose;
    });

  // NOTE: Trajectory does not support less than 4 points, so resample if less than 4.
  //       This implementation should be replaced in the future
  if (centerline_poses.size() < 4) {
    const auto lanelet_length = autoware::motion_utils::calcArcLength(centerline_poses);
    const auto resampling_interval = lanelet_length / 4.0;
    std::vector<double> resampled_arclength;
    for (double s = 0.0; s < lanelet_length; s += resampling_interval) {
      resampled_arclength.push_back(s);
    }
    if (lanelet_length - resampled_arclength.back() < autoware::motion_utils::overlap_threshold) {
      resampled_arclength.back() = lanelet_length;
    } else {
      resampled_arclength.push_back(lanelet_length);
    }
    centerline_poses =
      autoware::motion_utils::resamplePoseVector(centerline_poses, resampled_arclength);
    if (centerline_poses.size() < 4) return std::nullopt;
  }

  auto centerline =
    autoware::trajectory::Trajectory<geometry_msgs::msg::Pose>::Builder{}.build(centerline_poses);
  if (!centerline) return std::nullopt;
  centerline->align_orientation_with_trajectory_direction();

  const auto terminal_yaw = tf2::getYaw(centerline->compute(centerline->length()).orientation);
  const auto intervals = autoware::trajectory::find_intervals(
    centerline.value(),
    [terminal_yaw, angle_threshold_deg](const geometry_msgs::msg::Pose & point) {
      const auto yaw = tf2::getYaw(point.orientation);
      return std::fabs(autoware_utils::normalize_radian(yaw - terminal_yaw)) <
             autoware_utils::deg2rad(angle_threshold_deg);
    });
  if (intervals.empty()) return std::nullopt;

  return lanelet::utils::conversion::toLaneletPoint(
    centerline->compute(intervals.front().start).position);
}
}  // namespace utils
}  // namespace autoware::path_generator
