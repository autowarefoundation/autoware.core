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
class AutowareNodeRegistering : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    double deadline_ms = 220.0;
    node_options_an_.append_parameter_override("period_timer_register_ms", 100.0);
    node_options_an_.append_parameter_override("period_heartbeat_ms", 100.0);
    node_options_an_.append_parameter_override("deadline_ms", deadline_ms);

    node_options_acc_.append_parameter_override("deadline_ms", deadline_ms);
    node_options_acc_.append_parameter_override("report_publish_rate", 100.0);
  }

  void TearDown() override { rclcpp::shutdown(); }

  rclcpp::NodeOptions node_options_an_;
  rclcpp::NodeOptions node_options_acc_;
};

TEST_F(AutowareNodeRegistering, RegisterSuccessFirstACCThenAN)
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

  // wait until registered
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // Make sure node is registered by subscribing to the node reports
  autoware_control_center_msgs::msg::NodeReport report;
  ASSERT_TRUE(test::wait_for_node_report(
    autoware_node->get_node_base_interface()->get_fully_qualified_name(), report));

  // print uuid
  std::cout << "uuid: " << autoware_utils::to_hex_string(report.uuid) << std::endl;
  ASSERT_EQ(report.count_registered, 1);
  ASSERT_TRUE(report.is_alive);

  executor->cancel();  // make sure cancel is called after spin
  if (thread_spin.joinable()) {
    thread_spin.join();
  }
}

TEST_F(AutowareNodeRegistering, RegisterSuccessFirstANThenACC)
{
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  autoware::node::Node::SharedPtr autoware_node =
    std::make_shared<autoware::node::Node>("test_node", "test_ns", node_options_an_);
  autoware::node::Node::SharedPtr control_center =
    std::make_shared<autoware::control_center::ControlCenter>(node_options_acc_);

  executor->add_node(autoware_node->get_node_base_interface());
  std::thread thread_spin = std::thread([&executor]() { executor->spin(); });
  // wait until executor is actually spinning
  while (!executor->is_spinning()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  // let AN initialize
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  executor->add_node(control_center->get_node_base_interface());

  // wait until registered
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // Make sure node is registered by subscribing to the node reports
  autoware_control_center_msgs::msg::NodeReport report;
  ASSERT_TRUE(test::wait_for_node_report(
    autoware_node->get_node_base_interface()->get_fully_qualified_name(), report));

  // print uuid
  std::cout << "uuid: " << autoware_utils::to_hex_string(report.uuid) << std::endl;
  ASSERT_EQ(report.count_registered, 1);
  ASSERT_TRUE(report.is_alive);

  executor->cancel();  // make sure cancel is called after spin
  if (thread_spin.joinable()) {
    thread_spin.join();
  }
}
}  // namespace autoware::node
