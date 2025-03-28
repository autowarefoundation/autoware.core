// Copyright 2024 TIER IV, Inc.
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

#include "utils_test.hpp"

#include <lanelet2_core/geometry/Lanelet.h>

namespace autoware::path_generator
{
struct GetFirstIntersectionArcLengthTestParam
{
  std::string description;
  std::vector<lanelet::Id> lane_ids;
  double s_start;
  double s_end;
  std::optional<double> expected_s_intersection;
};

std::ostream & operator<<(std::ostream & os, const GetFirstIntersectionArcLengthTestParam & p)
{
  return os << p.description;
}

struct GetFirstIntersectionArcLengthTest
: public UtilsTest,
  public ::testing::WithParamInterface<GetFirstIntersectionArcLengthTestParam>
{
  void SetUp() override
  {
    UtilsTest::SetUp();
    set_map("autoware_test_utils", "overlap/lanelet2_map.osm");
  }
};

TEST_P(GetFirstIntersectionArcLengthTest, getFirstIntersectionArcLength)
{
  const auto & p = GetParam();

  const auto result = utils::get_first_intersection_arc_length(
    get_lanelets_from_ids(p.lane_ids), p.s_start, p.s_end, vehicle_info_.vehicle_length_m);

  ASSERT_EQ(result.has_value(), p.expected_s_intersection.has_value());

  constexpr auto epsilon = 1e-1;
  if (p.expected_s_intersection.has_value()) {
    ASSERT_NEAR(*result, *p.expected_s_intersection, epsilon);
  }
}

INSTANTIATE_TEST_SUITE_P(
  , GetFirstIntersectionArcLengthTest,
  ::testing::Values(
    GetFirstIntersectionArcLengthTestParam{
      "UTurnWithGap", {12200, 12214, 12199}, 0.0, std::numeric_limits<double>::max(), std::nullopt},
    GetFirstIntersectionArcLengthTestParam{
      "UTurnWithFullCrossing",
      {16652, 16655, 12996, 12997, 12995, 16665, 16662},
      0.0,
      std::numeric_limits<double>::max(),
      194.477},
    GetFirstIntersectionArcLengthTestParam{
      "UTurnWithHalfCrossing",
      {16698, 16702, 13129, 13130, 13128, 16705, 16699},
      0.0,
      std::numeric_limits<double>::max(),
      195.507},
    GetFirstIntersectionArcLengthTestParam{
      "Overpass",
      {13744, 13745, 13747, 13746, 13805},
      0.0,
      std::numeric_limits<double>::max(),
      311.068},
    GetFirstIntersectionArcLengthTestParam{
      "OverpassWithStartEdgeIntersection",
      {13744, 13745, 13747, 13746, 13805},
      36.0,
      std::numeric_limits<double>::max(),
      325.376},
    GetFirstIntersectionArcLengthTestParam{
      "OverpassWithIntersectionBehind",
      {13744, 13745, 13747, 13746, 13805},
      76.0,
      std::numeric_limits<double>::max(),
      std::nullopt},
    GetFirstIntersectionArcLengthTestParam{
      "OverpassWithIntersectionAhead",
      {13744, 13745, 13747, 13746, 13805},
      0.0,
      16.0,
      std::nullopt}),
  ::testing::PrintToStringParamName{});

TEST_F(UtilsTest, getFirstSelfIntersectionArcLength)
{
  constexpr double epsilon = 1e-1;

  {  // line string is empty
    const auto result = utils::get_first_self_intersection_arc_length(lanelet::BasicLineString2d{});

    ASSERT_FALSE(result);
  }

  {  // line string is straight line
    const auto result = utils::get_first_self_intersection_arc_length(
      lanelet::BasicLineString2d{{0.0, 0.0}, {1.0, 0.0}});

    ASSERT_FALSE(result);
  }

  {  // line string has no self-intersection
    const auto result = utils::get_first_self_intersection_arc_length(
      lanelet::BasicLineString2d{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {-1.0, 1.0}, {-1.0, -1.0}});

    ASSERT_FALSE(result);
  }

  {  // line string has self-intersection
    const auto result = utils::get_first_self_intersection_arc_length(
      lanelet::BasicLineString2d{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}, {0.0, -1.0}});

    ASSERT_TRUE(result);
    ASSERT_NEAR(*result, 4.0, epsilon);
  }

  {  // line string has overlap
    const auto result = utils::get_first_self_intersection_arc_length(lanelet::BasicLineString2d{
      {0.0, 0.0},
      {1.0, 0.0},
      {1.0, -1.0},
      {2.0, -1.0},
      {2.0, 1.0},
      {1.0, 1.0},
      {1.0, 0.0},
      {0.0, 0.0}});

    ASSERT_TRUE(result);
    ASSERT_NEAR(*result, 7.0, epsilon);
  }
}

TEST_F(UtilsTest, getPathBound)
{
  set_map("autoware_test_utils", "2km_test.osm");
  constexpr auto epsilon = 1e-1;

  {  // lanelet sequence is empty
    const auto [left, right] = utils::get_path_bounds(get_lanelets_from_ids({}), {}, {});

    ASSERT_TRUE(left.empty());
    ASSERT_TRUE(right.empty());
  }

  {  // normal case
    const auto [left, right] = utils::get_path_bounds(get_lanelets_from_ids({4417}), 1.0, 24.0);

    ASSERT_EQ(left.size(), 2);
    ASSERT_NEAR(left[0].x, -999.0, epsilon);
    ASSERT_NEAR(left[0].y, 3.5, epsilon);
    ASSERT_NEAR(left[1].x, -976.0, epsilon);
    ASSERT_NEAR(left[1].y, 3.5, epsilon);
    ASSERT_EQ(right.size(), 2);
    ASSERT_NEAR(right[0].x, -999.0, epsilon);
    ASSERT_NEAR(right[0].y, 0, epsilon);
    ASSERT_NEAR(right[1].x, -976.0, epsilon);
    ASSERT_NEAR(right[1].y, 0, epsilon);
  }

  {  // normal case with multiple lanelets
    const auto [left, right] =
      utils::get_path_bounds(get_lanelets_from_ids({4429, 4434}), 1.0, 49.0);

    ASSERT_EQ(left.size(), 3);
    ASSERT_NEAR(left[0].x, -974.0, epsilon);
    ASSERT_NEAR(left[0].y, 3.5, epsilon);
    ASSERT_NEAR(left[1].x, -950.0, epsilon);
    ASSERT_NEAR(left[1].y, 3.5, epsilon);
    ASSERT_NEAR(left[2].x, -926.0, epsilon);
    ASSERT_NEAR(left[2].y, 3.5, epsilon);
    ASSERT_EQ(right.size(), 3);
    ASSERT_NEAR(right[0].x, -974.0, epsilon);
    ASSERT_NEAR(right[0].y, 0.0, epsilon);
    ASSERT_NEAR(right[1].x, -950.0, epsilon);
    ASSERT_NEAR(right[1].y, 0.0, epsilon);
    ASSERT_NEAR(right[2].x, -926.0, epsilon);
    ASSERT_NEAR(right[2].y, 0.0, epsilon);
  }

  {  // start of bound is negative
    const auto [left, right] = utils::get_path_bounds(get_lanelets_from_ids({4417}), -1.0, 24.0);

    ASSERT_EQ(left.size(), 2);
    ASSERT_NEAR(left[0].x, -1000.0, epsilon);
    ASSERT_NEAR(left[0].y, 3.5, epsilon);
    ASSERT_NEAR(left[1].x, -976.0, epsilon);
    ASSERT_NEAR(left[1].y, 3.5, epsilon);
    ASSERT_EQ(right.size(), 2);
    ASSERT_NEAR(right[0].x, -1000.0, epsilon);
    ASSERT_NEAR(right[0].y, 0, epsilon);
    ASSERT_NEAR(right[1].x, -976.0, epsilon);
    ASSERT_NEAR(right[1].y, 0, epsilon);
  }

  {  // end of bound exceeds lanelet length
    const auto [left, right] = utils::get_path_bounds(get_lanelets_from_ids({4417}), 1.0, 26.0);

    ASSERT_EQ(left.size(), 2);
    ASSERT_NEAR(left[0].x, -999.0, epsilon);
    ASSERT_NEAR(left[0].y, 3.5, epsilon);
    ASSERT_NEAR(left[1].x, -975.0, epsilon);
    ASSERT_NEAR(left[1].y, 3.5, epsilon);
    ASSERT_EQ(right.size(), 2);
    ASSERT_NEAR(right[0].x, -999.0, epsilon);
    ASSERT_NEAR(right[0].y, 0, epsilon);
    ASSERT_NEAR(right[1].x, -975.0, epsilon);
    ASSERT_NEAR(right[1].y, 0, epsilon);
  }

  {  // start of bound is larger than end
    const auto [left, right] =
      utils::get_path_bounds(get_lanelets_from_ids({4429, 4434}), 30.0, 20.0);

    ASSERT_EQ(left.size(), 3);
    ASSERT_NEAR(left[0].x, -975.0, epsilon);
    ASSERT_NEAR(left[0].y, 3.5, epsilon);
    ASSERT_NEAR(left[1].x, -950.0, epsilon);
    ASSERT_NEAR(left[1].y, 3.5, epsilon);
    ASSERT_NEAR(left[2].x, -925.0, epsilon);
    ASSERT_NEAR(left[2].y, 3.5, epsilon);
    ASSERT_EQ(right.size(), 3);
    ASSERT_NEAR(right[0].x, -975.0, epsilon);
    ASSERT_NEAR(right[0].y, 0.0, epsilon);
    ASSERT_NEAR(right[1].x, -950.0, epsilon);
    ASSERT_NEAR(right[1].y, 0.0, epsilon);
    ASSERT_NEAR(right[2].x, -925.0, epsilon);
    ASSERT_NEAR(right[2].y, 0.0, epsilon);
  }
}

TEST_F(UtilsTest, cropLineString)
{
  constexpr auto epsilon = 1e-1;

  {  // line string is empty
    const auto result = utils::crop_line_string({}, {}, {});

    ASSERT_TRUE(result.empty());
  }

  {  // line string has only 1 point
    const auto result = utils::crop_line_string({geometry_msgs::msg::Point{}}, {}, {});

    ASSERT_EQ(result.size(), 1);
  }

  {  // normal case
    const auto result = utils::crop_line_string(
      {lanelet::utils::conversion::toGeomMsgPt(lanelet::BasicPoint3d{0.0, 0.0, 0.0}),
       lanelet::utils::conversion::toGeomMsgPt(lanelet::BasicPoint3d{3.0, 0.0, 0.0})},
      1.0, 2.0);

    ASSERT_EQ(result.size(), 2);
    ASSERT_NEAR(result[0].x, 1.0, epsilon);
    ASSERT_NEAR(result[0].y, 0.0, epsilon);
    ASSERT_NEAR(result[1].x, 2.0, epsilon);
    ASSERT_NEAR(result[1].y, 0.0, epsilon);
  }

  {  // start of crop range is negative
    const auto result = utils::crop_line_string(
      {lanelet::utils::conversion::toGeomMsgPt(lanelet::BasicPoint3d{0.0, 0.0, 0.0}),
       lanelet::utils::conversion::toGeomMsgPt(lanelet::BasicPoint3d{3.0, 0.0, 0.0})},
      -1.0, 2.0);

    ASSERT_EQ(result.size(), 2);
    ASSERT_NEAR(result[0].x, 0.0, epsilon);
    ASSERT_NEAR(result[0].y, 0.0, epsilon);
    ASSERT_NEAR(result[1].x, 3.0, epsilon);
    ASSERT_NEAR(result[1].y, 0.0, epsilon);
  }

  {  // start of crop range is larger than end
    const auto result = utils::crop_line_string(
      {lanelet::utils::conversion::toGeomMsgPt(lanelet::BasicPoint3d{0.0, 0.0, 0.0}),
       lanelet::utils::conversion::toGeomMsgPt(lanelet::BasicPoint3d{3.0, 0.0, 0.0})},
      2.0, 1.0);

    ASSERT_EQ(result.size(), 2);
    ASSERT_NEAR(result[0].x, 0.0, epsilon);
    ASSERT_NEAR(result[0].y, 0.0, epsilon);
    ASSERT_NEAR(result[1].x, 3.0, epsilon);
    ASSERT_NEAR(result[1].y, 0.0, epsilon);
  }
}

TEST_F(UtilsTest, GetArcLengthOnBounds)
{
  const auto epsilon = 1e-1;

  {  // lanelet sequence is empty
    const auto result = utils::get_arc_length_on_bounds({}, {});

    ASSERT_NEAR(result[0], {}, epsilon);
    ASSERT_NEAR(result[1], {}, epsilon);
  }

  {  // normal case
    const auto result = utils::get_arc_length_on_bounds(get_lanelets_from_ids({50}), 10.0);

    ASSERT_NEAR(result[0], 11.293, epsilon);
    ASSERT_NEAR(result[1], 8.823, epsilon);
  }

  {  // input arc length is negative
    const auto result = utils::get_arc_length_on_bounds(get_lanelets_from_ids({50}), -10.0);

    ASSERT_NEAR(result[0], 0.0, epsilon);
    ASSERT_NEAR(result[1], 0.0, epsilon);
  }

  {  // input arc length exceeds lanelet length
    const auto result = utils::get_arc_length_on_bounds(get_lanelets_from_ids({50}), 100.0);

    ASSERT_NEAR(result[0], 100.0, epsilon);
    ASSERT_NEAR(result[1], 100.0, epsilon);
  }
}

TEST_F(UtilsTest, GetArcLengthOnCenterline)
{
  const auto epsilon = 1e-1;

  {  // lanelet sequence is empty
    const auto result = utils::get_arc_length_on_centerline({}, {{}}, {{}});

    ASSERT_TRUE(result[0].has_value());
    ASSERT_NEAR(*result[0], {}, epsilon);
    ASSERT_TRUE(result[1].has_value());
    ASSERT_NEAR(*result[1], {}, epsilon);
  }

  {  // normal case
    const auto result =
      utils::get_arc_length_on_centerline(get_lanelets_from_ids({50}), 11.293, 8.823);

    ASSERT_TRUE(result[0].has_value());
    ASSERT_NEAR(*result[0], 10.0, epsilon);
    ASSERT_TRUE(result[1].has_value());
    ASSERT_NEAR(*result[1], 10.0, epsilon);
  }

  {  // input arc length is negative
    const auto result = utils::get_arc_length_on_centerline(get_lanelets_from_ids({50}), -10, -10);

    ASSERT_TRUE(result[0].has_value());
    ASSERT_NEAR(*result[0], 0.0, epsilon);
    ASSERT_TRUE(result[1].has_value());
    ASSERT_NEAR(*result[1], 0.0, epsilon);
  }

  {  // input arc length exceeds lanelet length
    const auto result =
      utils::get_arc_length_on_centerline(get_lanelets_from_ids({50}), 100.0, 100.0);

    ASSERT_TRUE(result[0].has_value());
    ASSERT_NEAR(*result[0], 100.0, epsilon);
    ASSERT_TRUE(result[1].has_value());
    ASSERT_NEAR(*result[1], 100.0, epsilon);
  }

  {  // input arc length is null
    const auto result =
      utils::get_arc_length_on_centerline(get_lanelets_from_ids({50}), std::nullopt, std::nullopt);

    ASSERT_FALSE(result[0].has_value());
    ASSERT_FALSE(result[1].has_value());
  }
}
}  // namespace autoware::path_generator