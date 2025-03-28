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
#include <utility>
#include <vector>

// Autoware & lanelet2 includes
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>  // for planning_utils::
#include <autoware/motion_utils/trajectory/trajectory.hpp>  // findFirstNearestIndexWithSoftConstraints
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/detection_area.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/no_stopping_area.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/road_marking.hpp>

#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Polygon.h>

namespace autoware::lanelet2_utils
{

using PathWithLaneId = tier4_planning_msgs::msg::PathWithLaneId;
using Point2d = autoware::universe_utils::Point2d;
using LineString2d = autoware::universe_utils::Line2d;

/**
 * @brief helper function to generate stop_line
 *
 * @param[in] path planned path
 * @param[in] no_stopping_areas lanelet no-stopping areas
 * @param[in] ego_width width of the ego vehicle
 * @param[in] stop_line_margin margin for generating new stop line
 * @return stop line
 */

std::optional<LineString2d> generate_stop_line(
  const PathWithLaneId & path, const lanelet::ConstPolygons3d & no_stopping_areas,
  const double ego_width, const double stop_line_margin);

/**
 * @brief Get the extended 2D geometry of a given stop line
 (originally from lanelet::autoware::DetectionArea & detection_area input)
 *
 * @param[in] stop_line original stop line
 * @param[in] extend_length length to extend the stop line
 * @return extended 2D line
 */
LineString2d get_stop_line_geometry2d(
  const lanelet::ConstLineString3d & stop_line, double extend_length);

/**
 * @brief Retrieve or generate a stop line geometry based on regulatory elements and path
 * (originally from lanelet::autoware::NoStoppingArea & no_stopping_area_reg_elem input)
 *
 * @param[in] path the planned path
 * @param[in] stop_line optional existing stop line
 * @param[in] no_stopping_areas lanelet no-stopping areas
 * @param[in] stop_line_margin margin for generating new stop line
 * @param[in] stop_line_extend_length extension length for stop line
 * @param[in] vehicle_width width of the vehicle
 * @return stop line as 2D geometry, if found or generated
 */
std::optional<LineString2d> get_stop_line_geometry2d(
  const PathWithLaneId & path, const lanelet::Optional<lanelet::ConstLineString3d> & stop_line,
  const lanelet::ConstPolygons3d & no_stopping_areas, double stop_line_margin,
  double stop_line_extend_length, double vehicle_width);

/**
 * @brief Extract stop lines associated with a stop sign ID from lanelets
 *
 * @param[in] lanelets input lanelets
 * @param[in] stop_sign_id traffic sign ID for stop sign
 * @return vector of matching stop lines
 */
std::vector<lanelet::ConstLineString3d> get_stop_lines_from_stop_sign(
  const lanelet::ConstLanelets & lanelets, const std::string & stop_sign_id);

/**
 * @brief Get stop line regulatory element from a lanelet, optionally checking matching ID
 *
 * @param[in] lane_id lanelet ID
 * @param[in] lanelet_map_ptr lanelet map
 * @param[in] attribute_name optional attribute to match
 * @param[in] check_id_match whether to check if attribute ID matches lane ID
 * @return stop line if found
 */
std::optional<lanelet::ConstLineString3d> get_stop_line_from_map(
  const lanelet::Id lane_id, const lanelet::LaneletMapPtr & lanelet_map_ptr,
  const std::string & attribute_name, bool check_id_match = true);

/**
 * @brief Get stop line index on path from map, or fall back to nearest point
 *
 * @param[in] path the planned path
 * @param[in] lane_interval path segment to search
 * @param[in] assigned_lanelet assigned lane
 * @param[in] lanelet_map_ptr lanelet map (planner_data_->lanelet_map)
 * @param[in] stop_line_extend_length stop line extension (planner_data_->stop_line_extend_length)
 * @param[in] nearest_dist_threshold threshold for nearest search
 * (planner_data_->nearest_dist_threshold)
 * @param[in] nearest_yaw_threshold yaw threshold for nearest search
 * (planner_data_->nearest_yaw_threshold)
 * @return index of stop line on path, if found
 */
std::optional<size_t> get_stop_line_index_from_map(
  const PathWithLaneId & path, const std::pair<size_t, size_t> lane_interval,
  lanelet::ConstLanelet assigned_lanelet, const lanelet::LaneletMapPtr & lanelet_map_ptr,
  double stop_line_extend_length, double nearest_dist_threshold, double nearest_yaw_threshold);

}  // namespace autoware::lanelet2_utils

#endif  // AUTOWARE_LANELET2_UTILS__STOP_LINE_HPP_
