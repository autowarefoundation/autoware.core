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
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <autoware_control_center_msgs/srv/deregister.hpp>
#include <autoware_control_center_msgs/srv/register.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include <gtest/gtest.h>

namespace autoware::control_center
{

class ControlCenterRegisteringTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_options_.append_parameter_override("deadline_ms", 220.0);
    node_options_.append_parameter_override("report_publish_rate", 100.0);
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
};

TEST_F(ControlCenterRegisteringTest, NodeRegistration)
{
  const std::string node_name = "node_test";
  const std::string ns;
  const auto [uuid, node] = test::register_node(node_name, ns);

  // Make sure node is alive
  ASSERT_EQ(
    control_center_->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  // Make sure node is registered by subscribing to the node reports
  autoware_control_center_msgs::msg::NodeReport report;
  ASSERT_TRUE(test::wait_for_node_report(node->get_fully_qualified_name(), report));
  ASSERT_EQ(report.uuid, uuid);
  ASSERT_EQ(report.count_registered, 1);
  ASSERT_TRUE(report.is_alive);
  ASSERT_EQ(
    report.status_activity.status,
    autoware_control_center_msgs::msg::NodeStatusActivity::INITIALIZING);
  ASSERT_EQ(
    report.status_operational.status,
    autoware_control_center_msgs::msg::NodeStatusOperational::WARNING);
}

TEST_F(ControlCenterRegisteringTest, NodeDuplicateRegistration)
{
  const std::string node_name = "node_test_duplicate";
  const std::string ns;
  const auto [uuid1, node1] = test::register_node(node_name, ns);

  const auto [uuid2, node2] = test::register_node(node_name, ns);

  // Ensure the UUID has changed
  ASSERT_NE(uuid1, uuid2);

  // Ensure the registration count incremented by subscribing to node reports
  autoware_control_center_msgs::msg::NodeReport report;
  ASSERT_TRUE(test::wait_for_node_report(node1->get_fully_qualified_name(), report));
  ASSERT_EQ(report.count_registered, 2);
  ASSERT_TRUE(report.is_alive);
  ASSERT_EQ(
    report.status_activity.status,
    autoware_control_center_msgs::msg::NodeStatusActivity::INITIALIZING);
  ASSERT_EQ(
    report.status_operational.status,
    autoware_control_center_msgs::msg::NodeStatusOperational::WARNING);
}

TEST_F(ControlCenterRegisteringTest, NodeDeregistration)
{
  const std::string node_name = "node_test_deregister";
  const std::string ns;
  const auto [uuid, node] = test::register_node(node_name, ns);
  ASSERT_TRUE(test::deregister_node(uuid));

  // Ensure node is no longer registered by checking the node reports
  ASSERT_FALSE(test::is_node_registered(node->get_fully_qualified_name()));
}

TEST_F(ControlCenterRegisteringTest, DeregisterNonExistentNode)
{
  unique_identifier_msgs::msg::UUID uuid = autoware_utils::generate_uuid();
  ASSERT_FALSE(test::deregister_node(uuid));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}  // namespace autoware::control_center
