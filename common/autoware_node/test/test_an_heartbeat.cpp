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

#include <autoware/control_center/control_center_node.hpp>
#include <autoware/node/node.hpp>
#include <autoware_utils/ros/uuid_helper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <lifecycle_msgs/msg/state.hpp>

#include <gtest/gtest.h>

namespace autoware::node
{
class AutowareNodeHeartbeat : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_options_an_.append_parameter_override("period_timer_register_ms", 100.0);
    node_options_an_.append_parameter_override("period_heartbeat_ms", period_heartbeat_ms_);
    node_options_an_.append_parameter_override("deadline_ms", deadline_ms_);

    node_options_acc_.append_parameter_override("deadline_ms", deadline_ms_);
    node_options_acc_.append_parameter_override(
      "report_publish_rate", 1000.0 / period_report_publish_ms_);
  }

  void TearDown() override { rclcpp::shutdown(); }

  static void validate_heartbeat(
    const std::string & node_full_name,
    const autoware_control_center_msgs::msg::Heartbeat::ConstSharedPtr & hb_healthy)
  {
    autoware_control_center_msgs::msg::NodeReport report;
    ASSERT_TRUE(test::wait_for_node_report(node_full_name, report));
    ASSERT_EQ(report.status_activity.status, hb_healthy->status_activity.status);
    ASSERT_EQ(report.status_operational.status, hb_healthy->status_operational.status);
  }

  rclcpp::NodeOptions node_options_an_;
  rclcpp::NodeOptions node_options_acc_;

  double deadline_ms_{220.0};
  double period_report_publish_ms_{10.0};
  double period_heartbeat_ms_{200.0};
};

TEST_F(AutowareNodeHeartbeat, HeartbeatHealthy)
{
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  autoware::node::Node::SharedPtr autoware_node =
    std::make_shared<autoware::node::Node>("test_node", "test_ns", node_options_an_);
  autoware::node::Node::SharedPtr control_center =
    std::make_shared<autoware::control_center::ControlCenter>(node_options_acc_);

  executor->add_node(control_center->get_node_base_interface());
  std::thread thread_spin = std::thread([&executor]() { executor->spin(); });
  // wait until executor is actually spinning
  while (!executor->is_spinning()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  // let ACC initialize
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  executor->add_node(autoware_node->get_node_base_interface());

  // wait until registered and published at least one heartbeat
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(period_heartbeat_ms_));

  for (int i = 0; i < 3; ++i) {
    {
      SCOPED_TRACE("validate_heartbeat iteration " + std::to_string(i));
      validate_heartbeat(
        autoware_node->get_node_base_interface()->get_fully_qualified_name(),
        test::generate_hb_healthy());
      std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(period_heartbeat_ms_));
    }
  }

  executor->cancel();  // make sure cancel is called after spin
  if (thread_spin.joinable()) {
    thread_spin.join();
  }
}
}  // namespace autoware::node
