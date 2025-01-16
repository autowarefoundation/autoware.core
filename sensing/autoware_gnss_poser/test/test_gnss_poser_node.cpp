// Copyright(c) 2024 AutoCore Technology (Nanjing) Co., Ltd.
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
#include <chrono>

#include "autoware/gnss_poser/gnss_poser_node.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "autoware_internal_debug_msgs/msg/bool_stamped.hpp"

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

    // Create subscribers to capture published messages
    pose_sub_ = gnss_poser_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "gnss_pose", rclcpp::QoS{1},
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        received_pose_ = *msg;
        pose_received_ = true;
      });

    pose_cov_sub_ = gnss_poser_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "gnss_pose_cov", rclcpp::QoS{1},
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        received_pose_cov_ = *msg;
        pose_cov_received_ = true;
      });

    fixed_sub_ = gnss_poser_->create_subscription<autoware_internal_debug_msgs::msg::BoolStamped>(
      "gnss_fixed", rclcpp::QoS{1},
      [this](const autoware_internal_debug_msgs::msg::BoolStamped::SharedPtr msg) {
        received_fixed_ = *msg;
        fixed_received_ = true;
      });

    // Create publisher to simulate incoming messages
    nav_sat_fix_pub_ = gnss_poser_->create_publisher<sensor_msgs::msg::NavSatFix>(
      "fix", rclcpp::QoS{1});

    executor_.add_node(gnss_poser_);
  }

  void TearDown() override
  {
    executor_.remove_node(gnss_poser_);
  }

  void spin_some()
  {
    executor_.spin_some(std::chrono::milliseconds(100));
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<GNSSPoser> gnss_poser_;

  // Subscribers to capture published messages
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_sub_;
  rclcpp::Subscription<autoware_internal_debug_msgs::msg::BoolStamped>::SharedPtr fixed_sub_;

  // Publisher to simulate incoming messages
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_fix_pub_;

  // Received messages
  geometry_msgs::msg::PoseStamped received_pose_;
  geometry_msgs::msg::PoseWithCovarianceStamped received_pose_cov_;
  autoware_internal_debug_msgs::msg::BoolStamped received_fixed_;

  // Flags to indicate if messages were received
  bool pose_received_ = false;
  bool pose_cov_received_ = false;
  bool fixed_received_ = false;
};

// Test case for callback_nav_sat_fix method
TEST_F(GNSSPoserTest, CallbackNavSatFixTest)
{
  // Create a NavSatFix message
  sensor_msgs::msg::NavSatFix nav_sat_fix_msg;
  nav_sat_fix_msg.header.stamp = rclcpp::Clock().now();
  nav_sat_fix_msg.header.frame_id = "gnss_frame";
  nav_sat_fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  nav_sat_fix_msg.latitude = 35.6895;
  nav_sat_fix_msg.longitude = 139.6917;
  nav_sat_fix_msg.altitude = 35.0;
  nav_sat_fix_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  // Publish the message
  nav_sat_fix_pub_->publish(nav_sat_fix_msg);

  // Spin the executor to process callbacks
  spin_some();

  // Check if messages were received
  EXPECT_TRUE(pose_received_);
  EXPECT_TRUE(pose_cov_received_);
  EXPECT_TRUE(fixed_received_);

  // Check if the fixed message is correct
  EXPECT_TRUE(received_fixed_.data);

  // Check if the pose message is correct
  EXPECT_EQ(received_pose_.header.frame_id, "map");
  EXPECT_EQ(received_pose_.header.stamp, nav_sat_fix_msg.header.stamp);

  // Check if the pose covariance message is correct
  EXPECT_EQ(received_pose_cov_.header.frame_id, "map");
  EXPECT_EQ(received_pose_cov_.header.stamp, nav_sat_fix_msg.header.stamp);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}