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

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/geometry.hpp>

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

double calc_interpolated_z(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & input,
  const geometry_msgs::msg::Point target_pos, const size_t seg_idx)
{
  const double closest_to_target_dist = autoware::motion_utils::calcSignedArcLength(
    input.points, input.points.at(seg_idx).point.pose.position,
    target_pos);  // TODO(murooka) implement calcSignedArcLength(points, idx, point)
  const double seg_dist =
    autoware::motion_utils::calcSignedArcLength(input.points, seg_idx, seg_idx + 1);

  const double closest_z = input.points.at(seg_idx).point.pose.position.z;
  const double next_z = input.points.at(seg_idx + 1).point.pose.position.z;
  const double interpolated_z =
    std::abs(seg_dist) < 1e-6
      ? next_z
      : closest_z + (next_z - closest_z) * closest_to_target_dist / seg_dist;
  return interpolated_z;
}

double calc_interpolated_velocity(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & input, const size_t seg_idx)
{
  const double seg_dist =
    autoware::motion_utils::calcSignedArcLength(input.points, seg_idx, seg_idx + 1);

  const double closest_vel = input.points.at(seg_idx).point.longitudinal_velocity_mps;
  const double next_vel = input.points.at(seg_idx + 1).point.longitudinal_velocity_mps;
  const double interpolated_vel = std::abs(seg_dist) < 1e-06 ? next_vel : closest_vel;
  return interpolated_vel;
}

template <class T>
size_t find_nearest_segment_index(
  const std::vector<T> & points, const geometry_msgs::msg::Pose & pose, const double dist_threshold,
  const double yaw_threshold)
{
  const auto nearest_idx =
    autoware::motion_utils::findNearestSegmentIndex(points, pose, dist_threshold, yaw_threshold);
  if (nearest_idx) {
    return nearest_idx.value();
  }

  return autoware::motion_utils::findNearestSegmentIndex(points, pose.position);
}

std::optional<size_t> find_index_out_of_goal_search_range(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Pose & goal, const int64_t goal_lane_id,
  const double max_dist = std::numeric_limits<double>::max())
{
  if (points.empty()) {
    return std::nullopt;
  }

  // find goal index
  size_t min_dist_index;
  {
    bool found = false;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < points.size(); ++i) {
      const auto & lane_ids = points.at(i).lane_ids;

      const double dist_to_goal =
        autoware_utils::calc_distance2d(points.at(i).point.pose, goal);
      const bool is_goal_lane_id_in_point =
        std::find(lane_ids.begin(), lane_ids.end(), goal_lane_id) != lane_ids.end();
      if (dist_to_goal < max_dist && dist_to_goal < min_dist && is_goal_lane_id_in_point) {
        min_dist_index = i;
        min_dist = dist_to_goal;
        found = true;
      }
    }
    if (!found) {
      return std::nullopt;
    }
  }

  // find index out of goal search range
  size_t min_dist_out_of_range_index = min_dist_index;
  for (int i = min_dist_index; 0 <= i; --i) {
    const double dist = autoware_utils::calc_distance2d(points.at(i).point, goal);
    min_dist_out_of_range_index = i;
    if (max_dist < dist) {
      break;
    }
  }

  return min_dist_out_of_range_index;
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

// goal does not have z
bool set_goal(
  const double search_radius_range, [[maybe_unused]] const double search_rad_range,
  const PathWithLaneId & input, const geometry_msgs::msg::Pose & goal, const int64_t goal_lane_id,
  PathWithLaneId * output_ptr)
{
  try {
    if (input.points.empty()) {
      return false;
    }

    // calculate refined_goal with interpolation
    // NOTE: goal does not have valid z, that will be calculated by interpolation here
    PathPointWithLaneId refined_goal{};
    const size_t closest_seg_idx_for_goal =
      find_nearest_segment_index(input.points, goal, 3.0, M_PI_4);
    refined_goal.point.pose = goal;
    refined_goal.point.pose.position.z =
      calc_interpolated_z(input, goal.position, closest_seg_idx_for_goal);
    refined_goal.point.longitudinal_velocity_mps = 0.0;

    // calculate pre_refined_goal with interpolation
    // NOTE: z and velocity are filled
    PathPointWithLaneId pre_refined_goal{};
    constexpr double goal_to_pre_goal_distance = -1.0;
    pre_refined_goal.point.pose =
      autoware_utils::calc_offset_pose(goal, goal_to_pre_goal_distance, 0.0, 0.0);
    const size_t closest_seg_idx_for_pre_goal =
      find_nearest_segment_index(input.points, pre_refined_goal.point.pose, 3.0, M_PI_4);
    pre_refined_goal.point.pose.position.z =
      calc_interpolated_z(input, pre_refined_goal.point.pose.position, closest_seg_idx_for_pre_goal);
    pre_refined_goal.point.longitudinal_velocity_mps =
      calc_interpolated_velocity(input, closest_seg_idx_for_pre_goal);

    // find min_dist_out_of_circle_index whose distance to goal is longer than search_radius_range
    const auto min_dist_out_of_circle_index_opt =
      find_index_out_of_goal_search_range(input.points, goal, goal_lane_id, search_radius_range);
    if (!min_dist_out_of_circle_index_opt) {
      return false;
    }
    const size_t min_dist_out_of_circle_index = min_dist_out_of_circle_index_opt.value();

    // create output points
    output_ptr->points.reserve(output_ptr->points.size() + min_dist_out_of_circle_index + 3);
    for (size_t i = 0; i <= min_dist_out_of_circle_index; ++i) {
      output_ptr->points.push_back(input.points.at(i));
    }
    output_ptr->points.push_back(pre_refined_goal);
    output_ptr->points.push_back(refined_goal);

    {  // fill skipped lane ids
      // pre refined goal
      auto & pre_goal = output_ptr->points.at(output_ptr->points.size() - 2);
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

      // goal
      output_ptr->points.back().lane_ids = input.points.back().lane_ids;
    }

    output_ptr->left_bound = input.left_bound;
    output_ptr->right_bound = input.right_bound;
    return true;
  } catch (std::out_of_range & ex) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("behavior_path_planner").get_child("utils"),
      "failed to set goal: " << ex.what());
    return false;
  }
}

const geometry_msgs::msg::Pose refine_goal(const geometry_msgs::msg::Pose & goal, const lanelet::ConstLanelet & goal_lanelet)
{
  const auto lanelet_point = lanelet::utils::conversion::toLaneletPoint(goal.position);
  const double distance = boost::geometry::distance(
    goal_lanelet.polygon2d().basicPolygon(), lanelet::utils::to2D(lanelet_point).basicPoint());
  if (distance < std::numeric_limits<double>::epsilon()) {
    return goal;
  }

  const auto segment = lanelet::utils::getClosestSegment(
    lanelet::utils::to2D(lanelet_point), goal_lanelet.centerline());
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
  const double search_radius_range, const double search_rad_range, const PathWithLaneId & input,
  const geometry_msgs::msg::Pose & goal, const int64_t goal_lane_id)
{
  PathWithLaneId filtered_path = input;
  PathWithLaneId path_with_goal;
  filtered_path.points = autoware::motion_utils::removeOverlapPoints(filtered_path.points);

  // always set zero velocity at the end of path for safety
  if (!filtered_path.points.empty()) {
    filtered_path.points.back().point.longitudinal_velocity_mps = 0.0;
  }

  if (set_goal(
        search_radius_range, search_rad_range, filtered_path, goal, goal_lane_id,
        &path_with_goal)) {
    return path_with_goal;
  }
  return filtered_path;
}

lanelet::ConstLanelets extract_lanelets_from_path(
  const PathWithLaneId & refined_path,
  const std::shared_ptr<const PlannerData> & planner_data)
{
  lanelet::ConstLanelets refined_path_lanelets;
  for (size_t i = 0; i < refined_path.points.size(); ++i) {
    const auto & path_point = refined_path.points.at(i);
    int64_t lane_id = path_point.lane_ids.at(0);
    lanelet::ConstLanelet lanelet = planner_data->lanelet_map_ptr->laneletLayer.get(lane_id);
    bool is_unique =
      std::find(refined_path_lanelets.begin(), refined_path_lanelets.end(), lanelet) ==
      refined_path_lanelets.end();
    if (is_unique) {
      refined_path_lanelets.push_back(lanelet);
    }
  }
  return refined_path_lanelets;
}

bool get_goal_lanelet(const PlannerData & planner_data, lanelet::ConstLanelet * goal_lanelet)
{
  const lanelet::Id goal_lane_id = planner_data.goal_lane_id;
  for (const auto & llt : planner_data.route_lanelets) {
    if (llt.id() == goal_lane_id) {
      *goal_lanelet = llt;
      return true;
    }
  }
  return false;
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
  const PathWithLaneId & refined_path,
  const std::shared_ptr<const PlannerData> & planner_data)
{
  const auto lanelets = extract_lanelets_from_path(refined_path, planner_data);
  // std::any_of detects whether any point lies outside lanelets
  bool has_points_outside_lanelet = std::any_of(
    refined_path.points.begin(), refined_path.points.end(),
    [&lanelets](const auto & refined_path_point) {
      return !is_in_lanelets(refined_path_point.point.pose, lanelets);
    });
  return !has_points_outside_lanelet;
}

PathWithLaneId modify_path_for_smooth_goal_connection(
  const PathWithLaneId & path, const std::shared_ptr<const PlannerData> & planner_data
)
{
  const auto goal = planner_data->goal_pose;
  const auto goal_lane_id = planner_data->goal_lane_id;

  geometry_msgs::msg::Pose refined_goal{};
  {
    lanelet::ConstLanelet goal_lanelet;

    // First, polish up the goal pose if possible
    if (get_goal_lanelet(*planner_data, &goal_lanelet)) {
      refined_goal = refine_goal(goal, goal_lanelet);
    } else {
      refined_goal = goal;
    }
  }
  double goal_search_radius{planner_data->path_generator_parameters.refine_goal_search_radius_range};

  // TODO(shen): define in the parameter
  constexpr double range_reduce_by{1.0};  // set a reasonable value, 10% - 20% of the
                                          // refine_goal_search_radius_range is recommended
  bool is_valid_path{false};
  PathWithLaneId refined_path;

  // Then, refine the path for the goal
  while (goal_search_radius >= 0 && !is_valid_path) {
    refined_path =
      refine_path_for_goal(goal_search_radius, M_PI * 0.5, path, refined_goal, goal_lane_id);
    if (is_path_valid(refined_path, planner_data)) {
      is_valid_path = true;
    }
    goal_search_radius -= range_reduce_by;
  }
  return refined_path;
}
}  // namespace utils
}  // namespace autoware::path_generator
