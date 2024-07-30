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

#include "autoware/node/node.hpp"

#include <autoware_utils/ros/uuid_helper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_control_center_msgs/srv/deregister.hpp>
#include <autoware_control_center_msgs/srv/register.hpp>

#include <chrono>

namespace autoware::node
{

Node::Node(
  const std::string & node_name, const std::string & ns, const rclcpp::NodeOptions & options)
: LifecycleNode(node_name, ns, options),
  full_name_{get_node_base_interface()->get_fully_qualified_name()},
  is_registered_{false}
{
  RCLCPP_DEBUG(get_logger(), "Node %s constructor", full_name_.c_str());
  std::chrono::milliseconds period_timer_register(
    declare_parameter<int>("period_timer_register_ms"));

  //  std::chrono::milliseconds heartbeat_period(declare_parameter<int>("heartbeat_period"));

  //  // The granted lease is essentially infinite here, i.e., only reader/watchdog will notify
  //  // violations. XXX causes segfault for cyclone dds, hence pass explicit lease life >
  //  heartbeat. rclcpp::QoS qos_profile(1);
  //  qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC)
  //    .liveliness_lease_duration(heartbeat_period + lease_delta)
  //    .deadline(heartbeat_period + lease_delta);
  //
  //  pub_heartbeat_ = this->create_publisher<autoware_control_center_msgs::msg::Heartbeat>(
  //    "~/heartbeat", qos_profile);
  //  timer_heartbeat_ =
  //    this->create_wall_timer(heartbeat_period, std::bind(&Node::on_tick_heartbeat, this));

  //  callback_group_mut_ex_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  cli_register_ = create_client<autoware_control_center_msgs::srv::Register>(
    "/autoware/control_center/srv/register", rmw_qos_profile_default);

  timer_registration_ =
    this->create_wall_timer(period_timer_register, std::bind(&Node::on_tick_registration, this));
}

void Node::on_tick_registration()
{
  RCLCPP_DEBUG(get_logger(), "on_tick_registration");
  if (is_registered_) {
    RCLCPP_DEBUG(get_logger(), "Already registered.");
    return;
  }

  if (!cli_register_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "%s is unavailable.", cli_register_->get_service_name());
    return;
  }
  autoware_control_center_msgs::srv::Register::Request::SharedPtr req =
    std::make_shared<autoware_control_center_msgs::srv::Register::Request>();
  req->node_name_with_namespace = full_name_;

  cli_register_->async_send_request(
    req, std::bind(&Node::on_register, this, std::placeholders::_1));
  RCLCPP_DEBUG(get_logger(), "Sent registration request.");
}

void Node::on_register(FutureRegister future)
{
  const auto & response = future.get();

  if (
    response->result_registration.result !=
    autoware_control_center_msgs::msg::ResultRegistration::SUCCESS) {
    is_registered_ = false;
    RCLCPP_WARN(get_logger(), "Registration failed.");
    return;
  }

  uuid_node_ = response->uuid_node;
  is_registered_ = true;
  RCLCPP_DEBUG(get_logger(), "Registration succeeded.");
}

}  // namespace autoware::node
