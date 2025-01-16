// Copyright 2025 AutoCore Technology (Nanjing) Co., Ltd.
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

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>

#define private public
#include "autoware/gnss_poser/gnss_poser_node.hpp"

using namespace autoware::gnss_poser;

// Test fixture for GNSSPoser
class GNSSPoserTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions node_options;
    node_options.append_parameter_override("base_frame", "base_link");
    node_options.append_parameter_override("gnss_base_frame", "gnss_antenna");
    node_options.append_parameter_override("map_frame", "map");
    node_options.append_parameter_override("use_gnss_ins_orientation", false);
    node_options.append_parameter_override("buff_epoch", 10);
    node_options.append_parameter_override("gnss_pose_pub_method", 0);

    gnss_poser_ = std::make_shared<GNSSPoser>(node_options);
  }

  std::shared_ptr<GNSSPoser> gnss_poser_;
};

// Test case for is_fixed method
TEST_F(GNSSPoserTest, IsFixedTest)
{
  sensor_msgs::msg::NavSatStatus status;
  status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  EXPECT_TRUE(GNSSPoser::is_fixed(status));

  status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  EXPECT_FALSE(GNSSPoser::is_fixed(status));
}

// Test case for can_get_covariance method
TEST_F(GNSSPoserTest, CanGetCovarianceTest)
{
  sensor_msgs::msg::NavSatFix fix;
  fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  EXPECT_TRUE(GNSSPoser::can_get_covariance(fix));

  fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  EXPECT_FALSE(GNSSPoser::can_get_covariance(fix));
}

// Test case for get_median_position method
TEST_F(GNSSPoserTest, GetMedianPositionTest)
{
  boost::circular_buffer<geometry_msgs::msg::Point> buffer(3);
  geometry_msgs::msg::Point p1, p2, p3;
  p1.x = 1.0; p1.y = 2.0; p1.z = 3.0;
  p2.x = 4.0; p2.y = 5.0; p2.z = 6.0;
  p3.x = 7.0; p3.y = 8.0; p3.z = 9.0;

  buffer.push_back(p1);
  buffer.push_back(p2);
  buffer.push_back(p3);

  geometry_msgs::msg::Point median = GNSSPoser::get_median_position(buffer);
  EXPECT_DOUBLE_EQ(median.x, 4.0);
  EXPECT_DOUBLE_EQ(median.y, 5.0);
  EXPECT_DOUBLE_EQ(median.z, 6.0);
}

// Test case for get_average_position method
TEST_F(GNSSPoserTest, GetAveragePositionTest)
{
  boost::circular_buffer<geometry_msgs::msg::Point> buffer(3);
  geometry_msgs::msg::Point p1, p2, p3;
  p1.x = 1.0; p1.y = 2.0; p1.z = 3.0;
  p2.x = 4.0; p2.y = 5.0; p2.z = 6.0;
  p3.x = 7.0; p3.y = 8.0; p3.z = 9.0;

  buffer.push_back(p1);
  buffer.push_back(p2);
  buffer.push_back(p3);

  geometry_msgs::msg::Point average = GNSSPoser::get_average_position(buffer);
  EXPECT_DOUBLE_EQ(average.x, 4.0);
  EXPECT_DOUBLE_EQ(average.y, 5.0);
  EXPECT_DOUBLE_EQ(average.z, 6.0);
}

// Test case for get_quaternion_by_position_difference method
TEST_F(GNSSPoserTest, GetQuaternionByPositionDifferenceTest)
{
  geometry_msgs::msg::Point point, prev_point;
  point.x = 1.0; point.y = 1.0; point.z = 0.0;
  prev_point.x = 0.0; prev_point.y = 0.0; prev_point.z = 0.0;

  geometry_msgs::msg::Quaternion orientation = GNSSPoser::get_quaternion_by_position_difference(point, prev_point);
  tf2::Quaternion tf_orientation;
  tf2::fromMsg(orientation, tf_orientation);

  EXPECT_DOUBLE_EQ(tf_orientation.getAngle(), M_PI_4); // 45 degrees in radians
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}