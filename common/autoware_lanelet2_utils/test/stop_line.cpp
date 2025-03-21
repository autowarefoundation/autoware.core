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

#include <gtest/gtest.h>
#include <filesystem>
#include <string>
#include <memory>
#include <vector>
#include <cmath>

// For M_PI_4
#ifndef M_PI_4
#define M_PI_4 0.7853981633974483
#endif

// Message definitions
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>
#include <tier4_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <autoware_planning_msgs/msg/path_point.hpp>


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
  TEST(StopLineUtilsTest, GetStopLineGeometry2d)
  {
    using lanelet2_utils::get_stop_line_geometry2d;
    using lanelet2_utils::generate_stop_line;

    
    // Create straight test path (10 points along x-axis)
    const tier4_planning_msgs::msg::PathWithLaneId path = generate_straight_path(10);
  
    // Configure no-stopping area that intersects the path
    lanelet::Polygon3d no_stopping_area;
    no_stopping_area.push_back(lanelet::Point3d(lanelet::InvalId, 3.0, -1.0));
    no_stopping_area.push_back(lanelet::Point3d(lanelet::InvalId, 3.0, 1.0));
    no_stopping_area.push_back(lanelet::Point3d(lanelet::InvalId, 5.0, 1.0));
    no_stopping_area.push_back(lanelet::Point3d(lanelet::InvalId, 5.0, -1.0));
    const lanelet::ConstPolygons3d no_stopping_areas = {no_stopping_area};
  
    // Test parameters
    const double stop_line_margin = 1.0;
    const double stop_line_extend_length = 1.0;
    const double vehicle_width = 1.0;
  
    // Case 1: Valid predefined stop line exists
    {
      lanelet::LineString3d predefined_stop_line;
      predefined_stop_line.push_back(lanelet::Point3d(lanelet::InvalId, 0.0, 0.0));
      predefined_stop_line.push_back(lanelet::Point3d(lanelet::InvalId, 1.0, 0.0));
      
      const auto result = get_stop_line_geometry2d(
        path, predefined_stop_line, no_stopping_areas, 
        stop_line_margin, stop_line_extend_length, vehicle_width
      );
  
      // Verify extended stop line coordinates
      ASSERT_TRUE(result.has_value());
      ASSERT_EQ(result->size(), 2UL);
      EXPECT_DOUBLE_EQ(result->front().x(), -1.0);
      EXPECT_DOUBLE_EQ(result->front().y(), 0.0);
      EXPECT_DOUBLE_EQ(result->back().x(), 2.0); 
      EXPECT_DOUBLE_EQ(result->back().y(), 0.0);
    }
  
    // Case 2: Invalid predefined stop line (size < 2)
    {
      lanelet::LineString3d invalid_stop_line;
      invalid_stop_line.push_back(lanelet::Point3d(lanelet::InvalId, 0.0, 0.0));
      
      const auto result = get_stop_line_geometry2d(
        path, invalid_stop_line, no_stopping_areas,
        stop_line_margin, stop_line_extend_length, vehicle_width
      );
  
      const auto expected = generate_stop_line(
        path, no_stopping_areas, vehicle_width, stop_line_margin
      );
  
      // Verify fallback to generated stop line
      ASSERT_TRUE(result.has_value());
      ASSERT_TRUE(expected.has_value());
      ASSERT_EQ(result->size(), expected->size());
      
      for (size_t i = 0; i < result->size(); ++i) {
        EXPECT_DOUBLE_EQ(result->at(i).x(), expected->at(i).x());
        EXPECT_DOUBLE_EQ(result->at(i).y(), expected->at(i).y());
      }
    }
  }  
  
  TEST(StopLineGeometryTest, GetStopLineGeometry2dFromRegElem)
  {
    using autoware::lanelet2_utils::get_stop_line_geometry2d;
    
    // Create a simple line string to represent a regulatory element’s stop line.
    lanelet::LineString3d reg_elem_line(lanelet::InvalId, {
      lanelet::Point3d(lanelet::InvalId, 0.0, 0.0),
      lanelet::Point3d(lanelet::InvalId, 0.0, 1.0)
    });
  
    // Test with an extension length.
    const double extend_length = 1.0;
    auto extended_line = lanelet2_utils::get_stop_line_geometry2d(reg_elem_line, extend_length);
    
    ASSERT_EQ(extended_line.size(), 2UL);
    EXPECT_DOUBLE_EQ(extended_line[0].x(), reg_elem_line[0].y());
    EXPECT_DOUBLE_EQ(extended_line[0].y(), reg_elem_line[0].y() - extend_length);
    EXPECT_DOUBLE_EQ(extended_line[1].x(), reg_elem_line[1].x());
    EXPECT_DOUBLE_EQ(extended_line[1].y(), reg_elem_line[1].y() + extend_length);
  }
}  // namespace autoware

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
