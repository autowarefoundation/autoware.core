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

#include <autoware_lanelet2_utils/stop_line.hpp>

#include <boost/geometry/algorithms/intersection.hpp>

#include <set>
#include <string>
#include <utility>
#include <vector>

namespace autoware::lanelet2_utils
{

template <typename GeometryT>
std::optional<std::pair<size_t, Point2d>> find_first_intersection(
  const PathWithLaneId & path, const std::pair<size_t, size_t> path_range,
  const GeometryT & geometry)
{
  for (size_t i = path_range.first; i < path_range.second && i < path.points.size() - 1; ++i) {
    const auto & p_front = path.points.at(i).point.pose.position;
    const auto & p_back = path.points.at(i + 1).point.pose.position;
    const LineString2d path_segment = {{p_front.x, p_front.y}, {p_back.x, p_back.y}};

    std::vector<Point2d> collision_points;
    boost::geometry::intersection(geometry, path_segment, collision_points);

    if (!collision_points.empty()) {
      return std::make_pair(i, collision_points.front());
    }
  }
  return std::nullopt;
}

std::optional<LineString2d> generate_stop_line(
  const PathWithLaneId & path, const lanelet::ConstPolygons3d & no_stopping_areas,
  const double ego_width, const double stop_line_margin)
{
  for (const auto & no_stopping_area : no_stopping_areas) {
    const auto & area_poly = lanelet::utils::to2D(no_stopping_area).basicPolygon();

    std::pair<size_t, size_t> path_range = {0, path.points.size() - 1};
    auto intersection_result = find_first_intersection(path, path_range, area_poly);

    if (intersection_result) {
      size_t i = intersection_result->first;
      const Point2d & collision_point = intersection_result->second;

      const auto p0 = path.points.at(i).point.pose.position;
      const auto p1 = path.points.at(i + 1).point.pose.position;
      const double yaw = autoware::universe_utils::calcAzimuthAngle(p0, p1);
      const double w = ego_width;
      const double l = stop_line_margin;

      LineString2d stop_line;
      stop_line.emplace_back(
        -l * std::cos(yaw) + collision_point.x() + w * std::cos(yaw + M_PI_2),
        collision_point.y() + w * std::sin(yaw + M_PI_2));
      stop_line.emplace_back(
        -l * std::cos(yaw) + collision_point.x() + w * std::cos(yaw - M_PI_2),
        collision_point.y() + w * std::sin(yaw - M_PI_2));
      return stop_line;
    }
  }
  return {};
}

universe_utils::LineString2d get_stop_line_geometry2d(
  const lanelet::ConstLineString3d & stop_line, const double extend_length)
{
  return autoware::behavior_velocity_planner::planning_utils::extendLine(
    stop_line[0], stop_line[1], extend_length);
}

std::optional<LineString2d> get_stop_line_geometry2d(
  const PathWithLaneId & path, const lanelet::Optional<lanelet::ConstLineString3d> & stop_line,
  const lanelet::ConstPolygons3d & no_stopping_areas, const double stop_line_margin,
  const double stop_line_extend_length, const double vehicle_width)
{
  if (stop_line && stop_line->size() >= 2) {
    return get_stop_line_geometry2d(stop_line.value(), stop_line_extend_length);
  }
  return generate_stop_line(path, no_stopping_areas, vehicle_width, stop_line_margin);
}

std::vector<lanelet::ConstLineString3d> get_stop_lines_from_stop_sign(
  const lanelet::ConstLanelets & lanelets, const std::string & stop_sign_id)
{
  std::vector<lanelet::ConstLineString3d> stoplines;
  std::set<lanelet::Id> checklist;

  for (const auto & ll : lanelets) {
    std::vector<std::shared_ptr<const lanelet::TrafficSign>> traffic_sign_reg_elems =
      ll.regulatoryElementsAs<const lanelet::TrafficSign>();

    if (!traffic_sign_reg_elems.empty()) {
      for (const auto & ts : traffic_sign_reg_elems) {
        if (ts->type() != stop_sign_id) {
          continue;
        }

        lanelet::ConstLineStrings3d traffic_sign_stoplines = ts->refLines();
        if (!traffic_sign_stoplines.empty()) {
          auto id = traffic_sign_stoplines.front().id();
          if (checklist.find(id) == checklist.end()) {
            checklist.insert(id);
            stoplines.push_back(traffic_sign_stoplines.front());
          }
        }
      }
    }
  }
  return stoplines;
}

std::optional<lanelet::ConstLineString3d> get_stop_line_from_map(
  const lanelet::Id lane_id, const lanelet::LaneletMapPtr & lanelet_map_ptr,
  const std::string & attribute_name, bool check_id_match)
{
  lanelet::ConstLanelet lanelet = lanelet_map_ptr->laneletLayer.get(lane_id);
  const auto road_markings = lanelet.regulatoryElementsAs<lanelet::autoware::RoadMarking>();
  lanelet::ConstLineStrings3d stop_line;

  for (const auto & road_marking : road_markings) {
    const std::string type =
      road_marking->roadMarking().attributeOr(lanelet::AttributeName::Type, "none");

    if (type == lanelet::AttributeValueString::StopLine) {
      if (check_id_match) {
        const int target_id = road_marking->roadMarking().attributeOr(attribute_name, 0);
        if (target_id != lane_id) {
          continue;
        }
      }
      stop_line.push_back(road_marking->roadMarking());
      break;
    }
  }

  if (stop_line.empty()) {
    return {};
  }
  return stop_line.front();
}

std::optional<size_t> get_stop_line_index_from_map(
  const PathWithLaneId & path, const std::pair<size_t, size_t> lane_interval,
  lanelet::ConstLanelet assigned_lanelet, const lanelet::LaneletMapPtr & lanelet_map_ptr,
  double stop_line_extend_length, double nearest_dist_threshold, double nearest_yaw_threshold)
{
  auto stopline_opt = get_stop_line_from_map(assigned_lanelet.id(), lanelet_map_ptr, "", false);

  if (!stopline_opt) {
    return std::nullopt;
  }

  const auto stopline = *stopline_opt;
  const LineString2d extended_stopline =
    get_stop_line_geometry2d(stopline, stop_line_extend_length);

  auto intersection_result = find_first_intersection(path, lane_interval, extended_stopline);
  if (intersection_result) {
    return intersection_result->first;
  }

  const auto p_start = stopline.front();
  const auto p_end = stopline.back();
  geometry_msgs::msg::Pose stop_point_from_map;
  stop_point_from_map.position.x = 0.5 * (p_start.x() + p_end.x());
  stop_point_from_map.position.y = 0.5 * (p_start.y() + p_end.y());
  stop_point_from_map.position.z = 0.5 * (p_start.z() + p_end.z());

  return autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    path.points, stop_point_from_map, nearest_dist_threshold, nearest_yaw_threshold);
}

}  // namespace autoware::lanelet2_utils
