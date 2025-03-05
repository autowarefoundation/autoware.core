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
#include <autoware_lanelet2_utility/topology.hpp>

#include <gtest/gtest.h>
#include <lanelet2_io/Io.h>

#include <filesystem>
#include <string>

namespace fs = std::filesystem;

namespace autoware
{

class TestWithIntersectionCrossingMap : public ::testing::Test
{
protected:
  lanelet::LaneletMapConstPtr lanelet_map_ptr_{nullptr};
  lanelet::routing::RoutingGraphConstPtr routing_graph_ptr_{nullptr};

  void SetUp() override
  {
    const auto sample_map_dir =
      fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utility")) /
      "sample_map";
    const auto intersection_crossing_map_path = sample_map_dir / "intersection" / "crossing.osm";

    lanelet_map_ptr_ = load_local_coordinate_map(intersection_crossing_map_path.string());
    routing_graph_ptr_ = lanelet2_utility::instantiate_routing_graph(lanelet_map_ptr_);
  }
};

TEST_F(TestWithIntersectionCrossingMap, LoadCheck)
{
  const auto point1861 = lanelet_map_ptr_->pointLayer.get(1861);
  EXPECT_EQ(point1861.x(), 0.0);
  EXPECT_EQ(point1861.y(), 0.0);
}

TEST_F(TestWithIntersectionCrossingMap, shoulder_lane_is_inaccesible_on_routing_graph)
{
  const auto left =
    lanelet2_utility::left_lanelet(lanelet_map_ptr_->laneletLayer.get(2398), routing_graph_ptr_);
  EXPECT_EQ(left.has_value(), false);
}

TEST_F(TestWithIntersectionCrossingMap, bicycle_lane_is_inaccesible_on_routing_graph)
{
  const auto left =
    lanelet2_utility::left_lanelet(lanelet_map_ptr_->laneletLayer.get(2450), routing_graph_ptr_);
  EXPECT_EQ(left.has_value(), false);
}

TEST_F(TestWithIntersectionCrossingMap, left_lanelet_without_lc_permission)
{
  const auto left =
    lanelet2_utility::left_lanelet(lanelet_map_ptr_->laneletLayer.get(2400), routing_graph_ptr_);
  EXPECT_EQ(left.value().id(), 2399);
}

TEST_F(TestWithIntersectionCrossingMap, left_lanelet_with_lc_permission)
{
  const auto left =
    lanelet2_utility::left_lanelet(lanelet_map_ptr_->laneletLayer.get(2399), routing_graph_ptr_);
  EXPECT_EQ(left.value().id(), 2398);
}

TEST_F(TestWithIntersectionCrossingMap, right_lanelet_without_lc_permission)
{
  const auto left =
    lanelet2_utility::right_lanelet(lanelet_map_ptr_->laneletLayer.get(2399), routing_graph_ptr_);
  EXPECT_EQ(left.value().id(), 2400);
}

TEST_F(TestWithIntersectionCrossingMap, right_lanelet_with_lc_permission)
{
  const auto left =
    lanelet2_utility::right_lanelet(lanelet_map_ptr_->laneletLayer.get(2398), routing_graph_ptr_);
  EXPECT_EQ(left.value().id(), 2399);
}

TEST_F(TestWithIntersectionCrossingMap, right_opposite_lanelet_null_because_it_is_middle_lane)
{
  const auto right_opposite = lanelet2_utility::right_opposite_lanelet(
    lanelet_map_ptr_->laneletLayer.get(2451), lanelet_map_ptr_);
  EXPECT_EQ(right_opposite.has_value(), false);
}

TEST_F(TestWithIntersectionCrossingMap, right_opposite_lanelet_valid)
{
  const auto lanelet = lanelet_map_ptr_->laneletLayer.get(2479);
  const auto right_opposite = lanelet2_utility::right_opposite_lanelet(lanelet, lanelet_map_ptr_);
  EXPECT_EQ(lanelet_map_ptr_->laneletLayer.findUsages(lanelet.rightBound().invert()).size(), 1);
  // EXPECT_EQ(right_opposite.value().id(), lanelet_map_ptr_->laneletLayer.get(2407).id());
}

/*
TEST_F(TestWithIntersectionCrossingInverseMap, left_opposite_lanelet_valid)
n{
  const auto left =
    lanelet2_utility::left_lanelet(lanelet_map_ptr_->laneletLayer.get(2398), routing_graph_ptr_);
  EXPECT_EQ(left.value().id(), 2397);
}

TEST_F(TestWithIntersectionCrossingInverseMap, left_opposite_lanelet_null)
{
  const auto left =
    lanelet2_utility::left_lanelet(lanelet_map_ptr_->laneletLayer.get(2398), routing_graph_ptr_);
  EXPECT_EQ(left.value().id(), 2397);
}
*/

}  // namespace autoware
