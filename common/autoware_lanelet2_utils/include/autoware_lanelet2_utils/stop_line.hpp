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

#ifndef AUTOWARE_LANELET2_UTILS__STOP_LINE_HPP_
#define AUTOWARE_LANELET2_UTILS__STOP_LINE_HPP_

#include <cmath>
#include <optional>
#include <set>
#include <string>
#include <vector>

// Autoware & lanelet2 includes
#include <autoware/behavior_velocity_planner_common/planner_data.hpp>      // for PlannerData
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>  // for planning_utils::
#include <autoware/motion_utils/trajectory/trajectory.hpp>  // findFirstNearestIndexWithSoftConstraints
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/detection_area.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/no_stopping_area.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/road_marking.hpp>

#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Polygon.h>

namespace stop_line
{

using PathWithLaneId = tier4_planning_msgs::msg::PathWithLaneId;
using PlannerData = autoware::behavior_velocity_planner::PlannerData;
using Point2d = autoware::universe_utils::Point2d;
using LineString2d = autoware::universe_utils::Line2d;

///// (TODO) Import it directly from
/// autoware_behavior_velocity_intersection_module/interpolated_path_info.hpp possible?
struct InterpolatedPathInfo
{
  /** the interpolated path */
  tier4_planning_msgs::msg::PathWithLaneId path;
  /** discretization interval of interpolation */
  double ds{0.0};
  /** the intersection lanelet id */
  lanelet::Id lane_id{0};
  /** the associative lane ids of lane_id */
  std::set<lanelet::Id> associative_lane_ids{};
  /** the range of indices for the path points with associative lane id */
  std::optional<std::pair<size_t, size_t>> lane_id_interval{std::nullopt};
};
//////////////////////////////////////////////////////////////////

/**
 * @brief retrieves a 2d stop line based on a nostoppingarea regulatory element
 *
 * checks if an explicit stop line is provided by the regulatory element. if not, it generates a
 * stop line by computing the intersection of the given path with no-stopping areas and offsetting
 * the collision point based on the vehicle width
 *
 * @param[in] path the planned path
 * @param[in] no_stopping_area the nostoppingarea regulatory element
 * @param[in] stop_line_margin the offset margin applied when generating the stop line
 * @param[in] stop_line_extend_length the length to extend the generated stop line
 * @param[in] vehicle_width the width of the vehicle for offsetting the stop line
 * @return an optional 2d stop line, if successfully generated or extracted
 */
std::optional<LineString2d> get_stop_line(
  const PathWithLaneId & path, const lanelet::autoware::NoStoppingArea & no_stopping_area_reg_elem,
  double stop_line_margin, double stop_line_extend_length, double vehicle_width);

/**
 * @brief retrieves a 3d stop line from a given lane id
 *
 * searches the lanelet map for the provided lane id and retrieves an associated stop line if
 * available
 *
 * @param[in] lane_id the lane id to find the stop line
 * @param[in] lanelet_map_ptr pointer to the lanelet map
 * @param[in] attribute_name the attribute name used to filter stop lines (usually a target id)
 * @return a 3d stop line if found, otherwise std::nullopt
 */
std::optional<lanelet::ConstLineString3d> get_stop_line(
  lanelet::Id lane_id, const lanelet::LaneletMapPtr & lanelet_map_ptr,
  const std::string & attribute_name);

/**
 * @brief retrieves a 2d stop line derived from a detectionarea regulatory element
 *
 * extracts the stop line from the detectionarea element and extends it by a specified length
 *
 * @param[in] detection_area the detectionarea regulatory element
 * @param[in] extend_length the length to extend the stop line
 * @return a 2d stop line if successfully extracted and extended
 */
LineString2d get_stop_line(
  const lanelet::autoware::DetectionArea & detection_area, double extend_length);

/**
 * @brief retrieves a 2d stop line intersection from the path and assigned lanelet
 *
 * obtains the assigned lanelet's stop line, extends it by the planner data parameters,
 * and searches for intersections with the provided path.
 * If no direct intersection occurs, the nearest path index is used instead.
 *
 * @param[in] path the planned path with lane ids
 * @param[in] assigned_lanelet the lanelet assigned to check for stop lines
 * @param[in] planner_data plannerdata containing parameters for planning
 * @return a 2d stop line intersection if found, otherwise std::nullopt
 */
std::optional<size_t> get_stop_line(
  const PathWithLaneId & path, const lanelet::Lanelet & assigned_lanelet,
  const PlannerData & planner_data);

/**
 * @brief retrieves stop lines from lanelets based on stopsign regulatory elements
 *
 * searches lanelets for trafficsigns matching the given stop sign id and retrieves their associated
 * stop lines
 *
 * @param[in] lanelets a collection of lanelets to search within
 * @param[in] stop_sign_id identifier of the stop sign regulatory element
 * @return a vector of 3d stop lines matching the provided stop sign id
 */
std::vector<lanelet::ConstLineString3d> get_stop_line(
  const lanelet::ConstLanelets & lanelets, const std::string & stop_sign_id);

}  // namespace stop_line

#endif  // AUTOWARE_LANELET2_UTILS__STOP_LINE_HPP_
