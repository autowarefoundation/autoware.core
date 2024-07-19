// Copyright 2024 The Autoware Contributors
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

#include "test_utility.hpp"

#include <control_center_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <autoware_control_center_msgs/msg/heartbeat.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include <gtest/gtest.h>

namespace autoware::control_center
{

class ControlCenterHeartbeatTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_options_.append_parameter_override("deadline_ms", deadline_ms_);
    node_options_.append_parameter_override("report_publish_rate", 1000.0 / node_period_ms_);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    control_center_ = std::make_shared<autoware::control_center::ControlCenter>(node_options_);

    executor_->add_node(control_center_->get_node_base_interface());

    thread_spin_ = std::thread([this]() { executor_->spin(); });
  }

  void TearDown() override
  {
    rclcpp::shutdown();
    if (thread_spin_.joinable()) {
      thread_spin_.join();
    }
  }

  rclcpp::NodeOptions node_options_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  ControlCenter::SharedPtr control_center_;
  std::thread thread_spin_;
  double deadline_ms_ = 50.0;
  double node_period_ms_ = 10.0;
};

TEST_F(ControlCenterHeartbeatTest, HeartbeatHandling)
{
  const auto [uuid, node] = test::register_node("node_test_heartbeat", "");

  // Send a heartbeat and verify node status is updated
  auto hb_healthy_init = test::generate_hb_healthy();
  auto pub_hb = test::send_first_heartbeat(node, deadline_ms_, hb_healthy_init);

  double sleep_duration_ms = node_period_ms_ * 1.5;
  ASSERT_LE(sleep_duration_ms, deadline_ms_);

  std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(sleep_duration_ms));

  autoware_control_center_msgs::msg::NodeReport report;
  ASSERT_TRUE(test::wait_for_node_report(node->get_fully_qualified_name(), report));
  ASSERT_EQ(report.status_activity.status, hb_healthy_init->status_activity.status);
  ASSERT_EQ(report.status_operational.status, hb_healthy_init->status_operational.status);

  test::send_heartbeat(node, pub_hb, test::generate_hb_healthy());
  std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(deadline_ms_ * 0.8));
  test::send_heartbeat(node, pub_hb, test::generate_hb_healthy());

  // Send another heartbeat to make sure it is not marked as dead
  test::send_heartbeat(node, pub_hb, test::generate_hb_healthy());
}

TEST_F(ControlCenterHeartbeatTest, HeartbeatMissedDeadline)
{
  const auto [uuid, node] = test::register_node("node_test_heartbeat", "");

  // Send a heartbeat
  auto pub_hb = test::send_first_heartbeat(node, deadline_ms_, test::generate_hb_healthy());

  // Wait for the heartbeat deadline to be missed
  std::this_thread::sleep_for(
    std::chrono::duration<double, std::milli>(deadline_ms_ + node_period_ms_ * 1.5));

  // Expect the node to be marked as dead
  autoware_control_center_msgs::msg::NodeReport report;
  ASSERT_TRUE(test::wait_for_node_report(node->get_fully_qualified_name(), report));
  ASSERT_FALSE(report.is_alive);
  ASSERT_EQ(
    report.status_activity.status, autoware_control_center_msgs::msg::NodeStatusActivity::UNKNOWN);
  ASSERT_EQ(
    report.status_operational.status,
    autoware_control_center_msgs::msg::NodeStatusOperational::UNKNOWN);

  // Send a heartbeat and verify node status is updated
  auto hb_healthy = test::generate_hb_healthy();
  test::send_heartbeat(node, pub_hb, hb_healthy);

  // Wait shortly and expect the node to be alive
  std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(node_period_ms_ * 2.0));
  ASSERT_TRUE(test::wait_for_node_report(node->get_fully_qualified_name(), report));
  ASSERT_TRUE(report.is_alive);
  ASSERT_EQ(report.status_activity.status, hb_healthy->status_activity.status);
  ASSERT_EQ(report.status_operational.status, hb_healthy->status_operational.status);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}  // namespace autoware::control_center
