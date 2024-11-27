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

#ifndef AUTOWARE__NODE__NODE_HPP_
#define AUTOWARE__NODE__NODE_HPP_

#include "autoware/node/visibility_control.hpp"

#include <rclcpp/message_memory_strategy.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/subscription_options.hpp>
#include <rclcpp/subscription_traits.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <autoware_control_center_msgs/msg/heartbeat.hpp>
#include <autoware_control_center_msgs/msg/node_status_activity.hpp>
#include <autoware_control_center_msgs/msg/node_status_operational.hpp>
#include <autoware_control_center_msgs/srv/deregister.hpp>
#include <autoware_control_center_msgs/srv/register.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <memory>
#include <string>
#include <utility>

namespace autoware::node
{
class Node : public rclcpp_lifecycle::LifecycleNode
{
public:
  AUTOWARE_NODE_PUBLIC
  explicit Node(
    const std::string & node_name, const std::string & ns = "",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using Heartbeat = autoware_control_center_msgs::msg::Heartbeat;
  using NodeStatusActivity = autoware_control_center_msgs::msg::NodeStatusActivity;
  using NodeStatusOperational = autoware_control_center_msgs::msg::NodeStatusOperational;
  using ResultDeregistration = autoware_control_center_msgs::msg::ResultDeregistration;
  using ResultRegistration = autoware_control_center_msgs::msg::ResultRegistration;
  using Deregister = autoware_control_center_msgs::srv::Deregister;
  using Register = autoware_control_center_msgs::srv::Register;

  using FutureRegister = rclcpp::Client<autoware_control_center_msgs::srv::Register>::SharedFuture;

  rclcpp::CallbackGroup::SharedPtr callback_group_mut_ex_;

  rclcpp::Client<autoware_control_center_msgs::srv::Register>::SharedPtr cli_register_;

  rclcpp::Publisher<autoware_control_center_msgs::msg::Heartbeat>::SharedPtr pub_heartbeat_;

  uint16_t sequence_number_;

  bool is_registered_;
  double period_timer_register_ms_;
  double period_heartbeat_ms_;
  double deadline_ms_;

  rclcpp::TimerBase::SharedPtr timer_registration_;
  rclcpp::TimerBase::SharedPtr timer_heartbeat_;

  unique_identifier_msgs::msg::UUID uuid_node_;

  void on_tick_registration();
  void on_tick_heartbeat();
  void on_register(FutureRegister future);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & state) override;

protected:
  void destroy_node();
};
}  // namespace autoware::node

#endif  // AUTOWARE__NODE__NODE_HPP_
