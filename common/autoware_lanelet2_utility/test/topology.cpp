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
  const auto point1861 = lanelet_map_ptr_->pointLayer.get(1824);
  EXPECT_EQ(point1861.x(), 0.0);
  EXPECT_EQ(point1861.y(), 0.0);
}

TEST_F(TestWithIntersectionCrossingMap, shoulder_lane_is_inaccesible_on_routing_graph)
{
  const auto left =
    lanelet2_utility::left_lanelet(lanelet_map_ptr_->laneletLayer.get(2341), routing_graph_ptr_);
  EXPECT_EQ(left.has_value(), false);
}

TEST_F(TestWithIntersectionCrossingMap, bicycle_lane_is_inaccesible_on_routing_graph)
{
  const auto left =
    lanelet2_utility::left_lanelet(lanelet_map_ptr_->laneletLayer.get(2372), routing_graph_ptr_);
  EXPECT_EQ(left.has_value(), false);
}

TEST_F(TestWithIntersectionCrossingMap, left_lanelet_without_lc_permission)
{
  const auto left =
    lanelet2_utility::left_lanelet(lanelet_map_ptr_->laneletLayer.get(2335), routing_graph_ptr_);
  EXPECT_EQ(left.value().id(), 2334);
}

TEST_F(TestWithIntersectionCrossingMap, left_lanelet_with_lc_permission)
{
  const auto left =
    lanelet2_utility::left_lanelet(lanelet_map_ptr_->laneletLayer.get(2334), routing_graph_ptr_);
  EXPECT_EQ(left.value().id(), 2333);
}

TEST_F(TestWithIntersectionCrossingMap, right_lanelet_without_lc_permission)
{
  const auto left =
    lanelet2_utility::right_lanelet(lanelet_map_ptr_->laneletLayer.get(2333), routing_graph_ptr_);
  EXPECT_EQ(left.value().id(), 2334);
}

TEST_F(TestWithIntersectionCrossingMap, right_lanelet_with_lc_permission)
{
  const auto left =
    lanelet2_utility::right_lanelet(lanelet_map_ptr_->laneletLayer.get(2334), routing_graph_ptr_);
  EXPECT_EQ(left.value().id(), 2335);
}

}  // namespace autoware
