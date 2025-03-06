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

#include "map_loader.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_lanelet2_utility/topology.hpp>
#include <range/v3/all.hpp>

#include <gtest/gtest.h>
#include <lanelet2_io/Io.h>

#include <filesystem>
#include <set>
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

    lanelet_map_ptr_ = load_mgrs_coordinate_map(intersection_crossing_map_path.string());
    routing_graph_ptr_ = lanelet2_utility::instantiate_routing_graph(lanelet_map_ptr_);
  }
};

TEST_F(TestWithIntersectionCrossingMap, LoadCheck)
{
  const auto point = lanelet_map_ptr_->pointLayer.get(1791);
  EXPECT_NEAR(point.x(), 100.0, 0.05);
  EXPECT_NEAR(point.y(), 100.0, 0.05);
}

TEST_F(TestWithIntersectionCrossingMap, shoulder_lane_is_inaccessible_on_routing_graph)
{
  const auto lane =
    lanelet2_utility::left_lanelet(lanelet_map_ptr_->laneletLayer.get(2257), routing_graph_ptr_);
  EXPECT_EQ(lane.has_value(), false);
}

TEST_F(TestWithIntersectionCrossingMap, bicycle_lane_is_inaccessible_on_routing_graph)
{
  const auto lane =
    lanelet2_utility::left_lanelet(lanelet_map_ptr_->laneletLayer.get(2286), routing_graph_ptr_);
  EXPECT_EQ(lane.has_value(), false);
}

TEST_F(TestWithIntersectionCrossingMap, left_lanelet_without_lc_permission)
{
  const auto lane =
    lanelet2_utility::left_lanelet(lanelet_map_ptr_->laneletLayer.get(2246), routing_graph_ptr_);
  EXPECT_EQ(lane.value().id(), 2245);
}

TEST_F(TestWithIntersectionCrossingMap, left_lanelet_with_lc_permission)
{
  const auto lane =
    lanelet2_utility::left_lanelet(lanelet_map_ptr_->laneletLayer.get(2245), routing_graph_ptr_);
  EXPECT_EQ(lane.value().id(), 2244);
}

TEST_F(TestWithIntersectionCrossingMap, right_lanelet_without_lc_permission)
{
  const auto lane =
    lanelet2_utility::right_lanelet(lanelet_map_ptr_->laneletLayer.get(2245), routing_graph_ptr_);
  EXPECT_EQ(lane.value().id(), 2246);
}

TEST_F(TestWithIntersectionCrossingMap, right_lanelet_with_lc_permission)
{
  const auto lane =
    lanelet2_utility::right_lanelet(lanelet_map_ptr_->laneletLayer.get(2244), routing_graph_ptr_);
  EXPECT_EQ(lane.value().id(), 2245);
}

TEST_F(TestWithIntersectionCrossingMap, right_opposite_lanelet)
{
  const auto lane = lanelet2_utility::right_opposite_lanelet(
    lanelet_map_ptr_->laneletLayer.get(2288), lanelet_map_ptr_);
  EXPECT_EQ(lane.value().id(), 2311);
}

TEST_F(TestWithIntersectionCrossingMap, leftmost_lanelet_valid)
{
  const auto lane = lanelet2_utility::leftmost_lanelet(
    lanelet_map_ptr_->laneletLayer.get(2288), routing_graph_ptr_);
  EXPECT_EQ(lane.value().id(), 2286);
}

TEST_F(TestWithIntersectionCrossingMap, leftmost_lanelet_null)
{
  const auto lane = lanelet2_utility::leftmost_lanelet(
    lanelet_map_ptr_->laneletLayer.get(2286), routing_graph_ptr_);
  EXPECT_EQ(lane.has_value(), false);
}

TEST_F(TestWithIntersectionCrossingMap, rightmost_lanelet_valid)
{
  const auto lane = lanelet2_utility::rightmost_lanelet(
    lanelet_map_ptr_->laneletLayer.get(2286), routing_graph_ptr_);
  EXPECT_EQ(lane.value().id(), 2288);
}

TEST_F(TestWithIntersectionCrossingMap, rightmost_lanelet_null)
{
  const auto lane = lanelet2_utility::rightmost_lanelet(
    lanelet_map_ptr_->laneletLayer.get(2288), routing_graph_ptr_);
  EXPECT_EQ(lane.has_value(), false);
}

TEST_F(TestWithIntersectionCrossingMap, left_lanelets_without_opposite)
{
  const auto lefts = lanelet2_utility::left_lanelets(
    lanelet_map_ptr_->laneletLayer.get(2288), lanelet_map_ptr_, routing_graph_ptr_);
  EXPECT_EQ(lefts.size(), 2);
  EXPECT_EQ(lefts[0].id(), 2287);
  EXPECT_EQ(lefts[1].id(), 2286);
}

TEST_F(TestWithIntersectionCrossingMap, right_lanelets_without_opposite)
{
  const auto lefts = lanelet2_utility::right_lanelets(
    lanelet_map_ptr_->laneletLayer.get(2286), lanelet_map_ptr_, routing_graph_ptr_);
  EXPECT_EQ(lefts.size(), 2);
  EXPECT_EQ(lefts[0].id(), 2287);
  EXPECT_EQ(lefts[1].id(), 2288);
}

TEST_F(TestWithIntersectionCrossingMap, following_lanelets)
{
  const auto following = lanelet2_utility::following_lanelets(
    lanelet_map_ptr_->laneletLayer.get(2244), routing_graph_ptr_);
  EXPECT_EQ(following.size(), 2);
  const auto ids = following | ranges::views::transform([](const auto & l) { return l.id(); }) |
                   ranges::to<std::set>();
  EXPECT_EQ(ids.find(2271) != ids.end(), true);
  EXPECT_EQ(ids.find(2265) != ids.end(), true);
}

/*
class TestWithIntersection_T_ShapeMap : public ::testing::Test
{
protected:
  lanelet::LaneletMapConstPtr lanelet_map_ptr_{nullptr};
  lanelet::routing::RoutingGraphConstPtr routing_graph_ptr_{nullptr};

  void SetUp() override
  {
    const auto sample_map_dir =
      fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utility")) /
      "sample_map";
    const auto intersection_t_shape_map_path = sample_map_dir / "intersection" / "T-Shape.osm";

    lanelet_map_ptr_ = load_local_coordinate_map(intersection_t_shape_map_path.string());
    routing_graph_ptr_ = lanelet2_utility::instantiate_routing_graph(lanelet_map_ptr_);
  }
};

TEST_F(TestWithIntersection_T_ShapeMap, LoadCheck)
{
  const auto point = lanelet_map_ptr_->pointLayer.get(212);
  EXPECT_EQ(point.x(), 0.0);
  EXPECT_EQ(point.y(), 0.0);
}
*/
}  // namespace autoware

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
