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

#include "autoware_lanelet2_utils/stop_line.hpp"

#include "map_loader.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// Message definitions
#include <autoware_planning_msgs/msg/path_point.hpp>
#include <tier4_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

namespace fs = std::filesystem;

// Helper function to generate a straight path along the x-axis.
tier4_planning_msgs::msg::PathWithLaneId generate_straight_path(
  const size_t nb_points, const float velocity = 0.0, const double resolution = 1.0)
{
  tier4_planning_msgs::msg::PathWithLaneId path;
  for (auto x = 0UL; x < nb_points; ++x) {
    tier4_planning_msgs::msg::PathPointWithLaneId p;
    p.point.pose.position.x = resolution * static_cast<double>(x);
    p.point.pose.position.y = 0.0;
    p.point.pose.position.z = 0.0;
    p.point.longitudinal_velocity_mps = velocity;
    path.points.push_back(p);
  }
  return path;
}

namespace autoware
{
class TestWithIntersectionCrossingMap : public ::testing::Test
{
protected:
  lanelet::LaneletMapPtr lanelet_map_ptr_{nullptr};
  lanelet::LaneletMapPtr traffic_sign_map_ptr_{nullptr};

  void SetUp() override
  {
    const auto sample_map_dir =
      fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
      "sample_map";
    const auto intersection_crossing_map_path = sample_map_dir / "intersection" / "crossing.osm";

    lanelet_map_ptr_ = load_mgrs_coordinate_map_non_const(intersection_crossing_map_path.string());

    const auto traffic_sign_map_path = sample_map_dir / "intersection" / "traffic_sign.osm";

    traffic_sign_map_ptr_ = load_mgrs_coordinate_map_non_const(traffic_sign_map_path.string());
  }
};

// test 1: get_stop_line_geometry2d with a valid predefined stop line.
TEST(StopLineUtilsTest, GetStopLineGeometry2d)
{
  using lanelet2_utils::generate_stop_line;
  using lanelet2_utils::get_stop_line_geometry2d;

  const tier4_planning_msgs::msg::PathWithLaneId path = generate_straight_path(10);

  lanelet::Polygon3d no_stopping_area;
  no_stopping_area.push_back(lanelet::Point3d(lanelet::InvalId, 3.0, -1.0));
  no_stopping_area.push_back(lanelet::Point3d(lanelet::InvalId, 3.0, 1.0));
  no_stopping_area.push_back(lanelet::Point3d(lanelet::InvalId, 5.0, 1.0));
  no_stopping_area.push_back(lanelet::Point3d(lanelet::InvalId, 5.0, -1.0));
  const lanelet::ConstPolygons3d no_stopping_areas = {no_stopping_area};

  const double stop_line_margin = 1.0;
  const double stop_line_extend_length = 1.0;
  const double vehicle_width = 1.0;

  // Case 1: valid predefined stop line exists
  {
    lanelet::LineString3d predefined_stop_line;
    predefined_stop_line.push_back(lanelet::Point3d(lanelet::InvalId, 0.0, 0.0));
    predefined_stop_line.push_back(lanelet::Point3d(lanelet::InvalId, 1.0, 0.0));

    const auto result = get_stop_line_geometry2d(
      path, predefined_stop_line, no_stopping_areas, stop_line_margin, stop_line_extend_length,
      vehicle_width);

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->size(), 2UL);
    EXPECT_DOUBLE_EQ(result->front().x(), -1.0);
    EXPECT_DOUBLE_EQ(result->front().y(), 0.0);
    EXPECT_DOUBLE_EQ(result->back().x(), 2.0);
    EXPECT_DOUBLE_EQ(result->back().y(), 0.0);
  }

  // Case 2: invalid predefined stop line (size < 2)
  {
    lanelet::LineString3d invalid_stop_line;
    invalid_stop_line.push_back(lanelet::Point3d(lanelet::InvalId, 0.0, 0.0));

    const auto result = get_stop_line_geometry2d(
      path, invalid_stop_line, no_stopping_areas, stop_line_margin, stop_line_extend_length,
      vehicle_width);

    const auto expected =
      generate_stop_line(path, no_stopping_areas, vehicle_width, stop_line_margin);

    ASSERT_TRUE(result.has_value());
    ASSERT_TRUE(expected.has_value());
    ASSERT_EQ(result->size(), expected->size());

    for (size_t i = 0; i < result->size(); ++i) {
      EXPECT_DOUBLE_EQ(result->at(i).x(), expected->at(i).x());
      EXPECT_DOUBLE_EQ(result->at(i).y(), expected->at(i).y());
    }
  }
}

// test 2: get_stop_line_geometry2d using a regulatory element line.
TEST(StopLineGeometryTest, GetStopLineGeometry2dFromRegElem)
{
  using autoware::lanelet2_utils::get_stop_line_geometry2d;

  lanelet::LineString3d reg_elem_line(
    lanelet::InvalId,
    {lanelet::Point3d(lanelet::InvalId, 0.0, 0.0), lanelet::Point3d(lanelet::InvalId, 0.0, 1.0)});

  const double extend_length = 1.0;
  auto extended_line = lanelet2_utils::get_stop_line_geometry2d(reg_elem_line, extend_length);

  ASSERT_EQ(extended_line.size(), 2UL);
  EXPECT_DOUBLE_EQ(extended_line[0].x(), reg_elem_line[0].y());
  EXPECT_DOUBLE_EQ(extended_line[0].y(), reg_elem_line[0].y() - extend_length);
  EXPECT_DOUBLE_EQ(extended_line[1].x(), reg_elem_line[1].x());
  EXPECT_DOUBLE_EQ(extended_line[1].y(), reg_elem_line[1].y() + extend_length);
}

// test 3: get_stop_line_from_map retrieves the stop line for a specific lane (ID 2273) from the
// map, then verifies that the stop line exists and has the expected ID.
TEST_F(TestWithIntersectionCrossingMap, GetStopLineFromMap)
{
  lanelet::Id valid_lane_id = 2273;
  auto non_const_map_ptr = lanelet_map_ptr_;
  auto stopline_opt =
    lanelet2_utils::get_stop_line_from_map(valid_lane_id, non_const_map_ptr, "", false);
  ASSERT_TRUE(stopline_opt.has_value()) << "No stop line found on lane " << valid_lane_id;
  ASSERT_EQ(stopline_opt->id(), 1993) << "Wrong id";
}

// test 4: get_stop_line_index_from_map when no stop line is present.
TEST_F(TestWithIntersectionCrossingMap, GetStopLineIndexFromMap_NoStopLine)
{
  // create a lanelet without a stop line and add it to the map
  lanelet::Id no_stop_line_lane_id = 10000;
  lanelet::Points3d left_points, right_points;
  left_points.push_back(lanelet::Point3d(lanelet::utils::getId(), 0.0, 1.0, 0.0));
  left_points.push_back(lanelet::Point3d(lanelet::utils::getId(), 10.0, 1.0, 0.0));
  right_points.push_back(lanelet::Point3d(lanelet::utils::getId(), 0.0, -1.0, 0.0));
  right_points.push_back(lanelet::Point3d(lanelet::utils::getId(), 10.0, -1.0, 0.0));

  lanelet::LineString3d left_ls(lanelet::utils::getId(), left_points);
  lanelet::LineString3d right_ls(lanelet::utils::getId(), right_points);
  lanelet::Lanelet lanelet = lanelet::Lanelet(no_stop_line_lane_id, left_ls, right_ls);

  lanelet_map_ptr_->add(lanelet);

  tier4_planning_msgs::msg::PathWithLaneId path;
  path.points.resize(1);
  std::pair<size_t, size_t> lane_interval = {0, 0};

  auto result = lanelet2_utils::get_stop_line_index_from_map(
    path, lane_interval, lanelet, lanelet_map_ptr_, 5.0, 1.0, 0.1);
  EXPECT_FALSE(result.has_value());
}

// test 5: get_stop_line_index_from_map when the path immediately crosses the stop line.
TEST_F(TestWithIntersectionCrossingMap, GetStopLineIndexFromMap_IntersectionFound)
{
  lanelet::Id valid_lane_id = 2273;
  auto assigned_lanelet = lanelet_map_ptr_->laneletLayer.get(valid_lane_id);

  auto stopline_opt =
    lanelet2_utils::get_stop_line_from_map(valid_lane_id, lanelet_map_ptr_, "", false);
  ASSERT_TRUE(stopline_opt.has_value());

  autoware::lanelet2_utils::LineString2d extended_stopline =
    lanelet2_utils::get_stop_line_geometry2d(*stopline_opt, 5.0);
  tier4_planning_msgs::msg::PathWithLaneId path;

  geometry_msgs::msg::Pose start_pose, end_pose;
  start_pose.position.x = extended_stopline.front().x() - 1.0;
  start_pose.position.y = extended_stopline.front().y();
  end_pose.position.x = extended_stopline.back().x() + 1.0;
  end_pose.position.y = extended_stopline.back().y();

  tier4_planning_msgs::msg::PathPointWithLaneId start_point, end_point;
  start_point.point.pose = start_pose;
  end_point.point.pose = end_pose;
  path.points = {start_point, end_point};

  std::pair<size_t, size_t> lane_interval = {0, path.points.size() - 1};

  auto result = lanelet2_utils::get_stop_line_index_from_map(
    path, lane_interval, assigned_lanelet, lanelet_map_ptr_, 5.0, 1.0, 0.1);

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result.value(), 0);
}

// test 6: get_stop_line_index_from_map using the nearest point method.
TEST_F(TestWithIntersectionCrossingMap, GetStopLineIndexFromMap_NearestPoint)
{
  lanelet::Id valid_lane_id = 2273;
  auto assigned_lanelet = lanelet_map_ptr_->laneletLayer.get(valid_lane_id);

  auto stopline_opt =
    lanelet2_utils::get_stop_line_from_map(valid_lane_id, lanelet_map_ptr_, "", false);
  ASSERT_TRUE(stopline_opt.has_value());

  auto stopline = *stopline_opt;
  geometry_msgs::msg::Point midpoint;
  midpoint.x = 0.5 * (stopline.front().x() + stopline.back().x());
  midpoint.y = 0.5 * (stopline.front().y() + stopline.back().y());
  midpoint.z = 0.5 * (stopline.front().z() + stopline.back().z());

  tier4_planning_msgs::msg::PathWithLaneId path;
  tier4_planning_msgs::msg::PathPointWithLaneId point;
  point.point.pose.position = midpoint;
  path.points = {point};

  std::pair<size_t, size_t> lane_interval = {0, 0};

  auto result = lanelet2_utils::get_stop_line_index_from_map(
    path, lane_interval, assigned_lanelet, lanelet_map_ptr_, 0.0, 0.1, 0.1);

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result.value(), 0);
}

// test 7: get_stop_line_index_from_map when the crossing does not occur at the start.
TEST_F(TestWithIntersectionCrossingMap, GetStopLineIndexFromMap_NotZero)
{
  lanelet::Id valid_lane_id = 2273;
  auto assigned_lanelet = lanelet_map_ptr_->laneletLayer.get(valid_lane_id);

  auto stopline_opt =
    lanelet2_utils::get_stop_line_from_map(valid_lane_id, lanelet_map_ptr_, "", false);
  ASSERT_TRUE(stopline_opt.has_value());

  autoware::lanelet2_utils::LineString2d extended_stopline =
    lanelet2_utils::get_stop_line_geometry2d(*stopline_opt, 5.0);

  tier4_planning_msgs::msg::PathWithLaneId path;

  geometry_msgs::msg::Pose p1, p2, p3;
  p1.position.x = extended_stopline.front().x() - 3.0;
  p1.position.y = extended_stopline.front().y();

  p2.position.x = extended_stopline.front().x() - 1.0;
  p2.position.y = extended_stopline.front().y();

  p3.position.x = extended_stopline.back().x() + 1.0;
  p3.position.y = extended_stopline.back().y();

  tier4_planning_msgs::msg::PathPointWithLaneId pp1, pp2, pp3;
  pp1.point.pose = p1;
  pp2.point.pose = p2;
  pp3.point.pose = p3;
  path.points = {pp1, pp2, pp3};

  std::pair<size_t, size_t> lane_interval = {0, path.points.size() - 1};

  auto result = lanelet2_utils::get_stop_line_index_from_map(
    path, lane_interval, assigned_lanelet, lanelet_map_ptr_, 5.0, 1.0, 0.1);

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result.value(), 1);
}

// test 7: get_stop_line_from_stop_sign
TEST_F(TestWithIntersectionCrossingMap, GetStopLinesFromStopSign)
{
  const lanelet::ConstLanelets lanelets(
    traffic_sign_map_ptr_->laneletLayer.begin(), traffic_sign_map_ptr_->laneletLayer.end());

  std::string stop_sign_id = "stop_sign";
  auto stoplines = lanelet2_utils::get_stop_lines_from_stop_sign(lanelets, stop_sign_id);

  // verify that at least one stop line is returned.
  ASSERT_FALSE(stoplines.empty()) << "No stop lines were found for stop sign type: "
                                  << stop_sign_id;

  // check the id is correct in the results.
  lanelet::Id expected_stop_line_id = 9350;
  bool found_expected = false;
  for (const auto & sl : stoplines) {
    if (sl.id() == expected_stop_line_id) {
      found_expected = true;
      break;
    }
  }
  EXPECT_TRUE(found_expected) << "Expected stop line with id " << expected_stop_line_id
                              << " was not found.";
}

}  // namespace autoware

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
