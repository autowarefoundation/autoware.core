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
#include <vector>

namespace stop_line
{

// jterates over RoadMarking regulatory elements and returns the first stop line that satisfies a
// predicate.
template <typename Predicate>
inline std::optional<lanelet::ConstLineString3d> find_regulatory_stop_line(
  const lanelet::Lanelet & lanelet_obj, Predicate pred)
{
  const auto road_markings = lanelet_obj.regulatoryElementsAs<lanelet::autoware::RoadMarking>();
  for (const auto & rm : road_markings) {
    const std::string type = rm->roadMarking().attributeOr(lanelet::AttributeName::Type, "none");
    if (pred(rm, type)) {
      return rm->roadMarking();
    }
  }
  return std::nullopt;
}

// helper for NoStoppingArea: generate a 2D stop line by finding intersections of the path with
// no-stopping areas.
std::optional<LineString2d> generate_stop_line(
  const PathWithLaneId & path, const lanelet::ConstPolygons3d & no_stopping_areas, double ego_width,
  double stop_line_margin)
{
  const auto & points = path.points;
  for (const auto & no_stopping_area : no_stopping_areas) {
    // Convert the 3D polygon to a 2D polygon using lanelet's to2D utility.
    const auto area_poly = lanelet::utils::to2D(no_stopping_area).basicPolygon();

    // Iterate over each segment of the path.
    for (size_t i = 0; i < points.size() - 1; ++i) {
      const auto & p0 = points.at(i).point.pose.position;
      const auto & p1 = points.at(i + 1).point.pose.position;

      // Construct a 2D segment from the path.
      Point2d pt0(p0.x, p0.y);
      Point2d pt1(p1.x, p1.y);
      LineString2d segment;
      segment.push_back(pt0);
      segment.push_back(pt1);

      // Find intersections between the area polygon and the path segment.
      std::vector<Point2d> collision_points;
      boost::geometry::intersection(area_poly, segment, collision_points);

      if (!collision_points.empty()) {
        // Compute the azimuth (yaw) of the segment.
        double yaw = autoware::universe_utils::calcAzimuthAngle(p0, p1);
        const double w = ego_width;
        const double l = stop_line_margin;

        // Compute the stop line points inline:
        LineString2d stop_line;
        stop_line.push_back(Point2d(
          collision_points.front().x() - l * std::cos(yaw) + w * std::cos(yaw + M_PI_2),
          collision_points.front().y() + w * std::sin(yaw + M_PI_2)));
        stop_line.push_back(Point2d(
          collision_points.front().x() - l * std::cos(yaw) + w * std::cos(yaw - M_PI_2),
          collision_points.front().y() + w * std::sin(yaw - M_PI_2)));

        return stop_line;
      }
    }
  }
  return std::nullopt;
}

// NoStoppingArea version.
std::optional<LineString2d> get_stop_line(
  const PathWithLaneId & path, const lanelet::autoware::NoStoppingArea & no_stopping_area_reg_elem,
  double stop_line_margin, double stop_line_extend_length, double vehicle_width)
{
  const auto & maybe_stop_line = no_stopping_area_reg_elem.stopLine();
  if (maybe_stop_line && maybe_stop_line->size() >= 2) {
    return autoware::behavior_velocity_planner::planning_utils::extendLine(
      maybe_stop_line.value()[0], maybe_stop_line.value()[1], stop_line_extend_length);
  }
  return generate_stop_line(
    path, no_stopping_area_reg_elem.noStoppingAreas(), vehicle_width, stop_line_margin);
}

// DetectionArea version.
LineString2d get_stop_line(
  const lanelet::autoware::DetectionArea & detection_area, double extend_length)
{
  const auto stop_line = detection_area.stopLine();
  return autoware::behavior_velocity_planner::planning_utils::extendLine(
    stop_line[0], stop_line[1], extend_length);
}

// Map version.
std::optional<lanelet::ConstLineString3d> get_stop_line(
  lanelet::Id lane_id, const lanelet::LaneletMapPtr & lanelet_map_ptr,
  const std::string & attribute_name)
{
  lanelet::Lanelet lanelet_obj = lanelet_map_ptr->laneletLayer.get(lane_id);
  return find_regulatory_stop_line(
    lanelet_obj, [lane_id, &attribute_name](const auto & rm, const std::string & type) {
      const int target_id = rm->roadMarking().attributeOr(attribute_name, 0);
      return (
        type == lanelet::AttributeValueString::StopLine && target_id == static_cast<int>(lane_id));
    });
}

// Intersection version.
std::optional<size_t> get_stop_line(
  const tier4_planning_msgs::msg::PathWithLaneId & path, const lanelet::Lanelet & assigned_lanelet,
  const PlannerData & planner_data)
{
  const auto & points = path.points;
  auto opt_stopline =
    find_regulatory_stop_line(assigned_lanelet, [](const auto &, const std::string & type) {
      return (type == lanelet::AttributeValueString::StopLine);
    });
  if (!opt_stopline) {
    return std::nullopt;
  }
  const auto & stop_line_reg = opt_stopline.value();
  const auto p_start = stop_line_reg.front();
  const auto p_end = stop_line_reg.back();
  const LineString2d extended_stopline =
    autoware::behavior_velocity_planner::planning_utils::extendLine(
      p_start, p_end, planner_data.stop_line_extend_length);

  // iterate through the path segments.
  for (size_t i = 0; i < points.size() - 1; i++) {
    const auto & p_front = points.at(i).point.pose.position;
    const auto & p_back = points.at(i + 1).point.pose.position;
    Point2d front_pt(p_front.x, p_front.y);
    Point2d back_pt(p_back.x, p_back.y);
    LineString2d path_segment;
    path_segment.push_back(front_pt);
    path_segment.push_back(back_pt);
    std::vector<Point2d> collision_points;
    boost::geometry::intersection(extended_stopline, path_segment, collision_points);
    if (!collision_points.empty()) {
      return i;
    }
  }

  // compute the midpoint of the stop line and use soft constraints.
  geometry_msgs::msg::Pose stop_point_from_map;
  stop_point_from_map.position.x = 0.5 * (p_start.x() + p_end.x());
  stop_point_from_map.position.y = 0.5 * (p_start.y() + p_end.y());
  stop_point_from_map.position.z = 0.5;

  return autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    points, stop_point_from_map, planner_data.ego_nearest_dist_threshold,
    planner_data.ego_nearest_yaw_threshold);
}

// StopSign version.
std::vector<lanelet::ConstLineString3d> get_stop_line(
  const lanelet::ConstLanelets & lanelets, const std::string & stop_sign_id)
{
  std::vector<lanelet::ConstLineString3d> stoplines;
  std::set<lanelet::Id> checklist;
  for (const auto & ll : lanelets) {
    auto traffic_sign_reg_elems = ll.regulatoryElementsAs<lanelet::TrafficSign>();
    for (const auto & ts : traffic_sign_reg_elems) {
      if (ts->type() != stop_sign_id) {
        continue;
      }
      auto traffic_sign_stoplines = ts->refLines();
      if (!traffic_sign_stoplines.empty()) {
        auto id = traffic_sign_stoplines.front().id();
        if (checklist.find(id) == checklist.end()) {
          checklist.insert(id);
          stoplines.push_back(traffic_sign_stoplines.front());
        }
      }
    }
  }
  return stoplines;
}

}  // namespace stop_line
