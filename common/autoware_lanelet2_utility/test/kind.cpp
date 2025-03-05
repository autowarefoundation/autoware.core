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

#include "local_projector.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_lanelet2_extension/projection/mgrs_projector.hpp>
#include <autoware_lanelet2_utility/kind.hpp>

#include <gtest/gtest.h>
#include <lanelet2_io/Io.h>

#include <filesystem>
#include <string>

namespace fs = std::filesystem;

class TestWithRoadShoulderHighwayMap : public ::testing::Test
{
protected:
  lanelet::LaneletMapConstPtr lanelet_map_ptr_{nullptr};

  void SetUp() override
  {
    const auto sample_map_dir =
      fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utility")) /
      "sample_map";
    const auto road_shoulder_highway_map_path = sample_map_dir / "road_shoulder" / "highway.osm";

    lanelet_map_ptr_ = load_local_coordinate_map(road_shoulder_highway_map_path.string());
  }
};

TEST_F(TestWithRoadShoulderHighwayMap, LoadCheck)
{
  const auto point8 = lanelet_map_ptr_->pointLayer.get(1);
  EXPECT_EQ(point8.x(), 0.0);
  EXPECT_EQ(point8.y(), 0.0);
  const auto point10 = lanelet_map_ptr_->pointLayer.get(2);
  EXPECT_EQ(point10.x(), 4.0);
  EXPECT_EQ(point10.y(), 0.0);
  const auto point15 = lanelet_map_ptr_->pointLayer.get(3);
  EXPECT_EQ(point15.x(), 8.0);
  EXPECT_EQ(point15.y(), 0.0);
}

TEST_F(TestWithRoadShoulderHighwayMap, is_road_lane)
{
  const auto ll = lanelet_map_ptr_->laneletLayer.get(47);
  EXPECT_EQ(autoware::lanelet2_utility::is_road_lane(ll), true);
  EXPECT_EQ(autoware::lanelet2_utility::is_shoulder_lane(ll), false);
  EXPECT_EQ(autoware::lanelet2_utility::is_bicycle_lane(ll), false);
}

TEST_F(TestWithRoadShoulderHighwayMap, is_shoulder_lane)
{
  const auto ll = lanelet_map_ptr_->laneletLayer.get(44);
  EXPECT_EQ(autoware::lanelet2_utility::is_road_lane(ll), false);
  EXPECT_EQ(autoware::lanelet2_utility::is_shoulder_lane(ll), true);
  EXPECT_EQ(autoware::lanelet2_utility::is_bicycle_lane(ll), false);
}

// TODO(soblin): add and use bicycle lane map

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
