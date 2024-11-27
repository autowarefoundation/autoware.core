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

#include <autoware/control_center/control_center_node.hpp>
#include <autoware/node/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <lifecycle_msgs/msg/state.hpp>

#include <gtest/gtest.h>

class AutowareNodeInitShutdown : public ::testing::Test
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

TEST_F(AutowareNodeInitShutdown, NodeInitShutdown)
{
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  autoware::node::Node::SharedPtr autoware_node =
    std::make_shared<autoware::node::Node>("test_node", "test_ns", node_options_an_);
  autoware::node::Node::SharedPtr control_center =
    std::make_shared<autoware::control_center::ControlCenter>(node_options_acc_);

  executor->add_node(control_center->get_node_base_interface());
  executor->add_node(autoware_node->get_node_base_interface());

  std::thread thread_spin = std::thread([&executor]() { executor->spin(); });

  ASSERT_EQ(
    autoware_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  auto state = autoware_node->shutdown();

  ASSERT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);

  // wait until executor is spinning
  while (!executor->is_spinning()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  executor->cancel();  // make sure cancel is called after spin
  if (thread_spin.joinable()) {
    thread_spin.join();
  }
}

// TEST_F(AutowareNodeTest, Sen)
