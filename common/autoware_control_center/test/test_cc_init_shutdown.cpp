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
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <lifecycle_msgs/msg/state.hpp>

#include <gtest/gtest.h>

class ControlCenterInitShutdownTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_options_.append_parameter_override("deadline_ms", 220.0);
    node_options_.append_parameter_override("report_publish_rate", 1.0);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    control_center_ = std::make_shared<autoware::control_center::ControlCenter>(node_options_);
    executor_->add_node(control_center_->get_node_base_interface());
    executor_->spin_some(std::chrono::milliseconds(50));
  }

  void TearDown() override { rclcpp::shutdown(); }

  rclcpp::NodeOptions node_options_;
  autoware::control_center::ControlCenter::SharedPtr control_center_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

TEST_F(ControlCenterInitShutdownTest, NodeInitShutdown)
{
  ASSERT_EQ(
    control_center_->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  auto state = control_center_->shutdown();
  executor_->spin_some(std::chrono::milliseconds(50));
  ASSERT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
