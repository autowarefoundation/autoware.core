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

#include "autoware_control_center_msgs/msg/node_status.hpp"
#include "autoware_control_center_msgs/srv/register.hpp"

#include <chrono>

constexpr std::chrono::milliseconds lease_delta(
  20);  ///< Buffer added to heartbeat to define lease.

namespace autoware::node
{

Node::Node(
  const std::string & node_name, const std::string & ns, const rclcpp::NodeOptions & options)
: LifecycleNode(node_name, ns, options),
  self_name_{ns + "/" + node_name},
  is_registered_{false},
  sequence_number_{0}
{
  RCLCPP_DEBUG(get_logger(), "Node::Node()");
  declare_parameter<int>("heartbeat_period", 200);
  declare_parameter<int>("register_timer_period", 500);
  std::chrono::milliseconds heartbeat_period(get_parameter("heartbeat_period").as_int());
  std::chrono::milliseconds register_timer_period(get_parameter("register_timer_period").as_int());

  // The granted lease is essentially infinite here, i.e., only reader/watchdog will notify
  // violations. XXX causes segfault for cyclone dds, hence pass explicit lease life > heartbeat.
  rclcpp::QoS qos_profile(1);
  qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC)
    .liveliness_lease_duration(heartbeat_period + lease_delta)
    .deadline(heartbeat_period + lease_delta);

  pub_heartbeat_ = this->create_publisher<autoware_control_center_msgs::msg::Heartbeat>(
    "~/heartbeat", qos_profile);
  timer_heartbeat_ =
    this->create_wall_timer(heartbeat_period, std::bind(&Node::on_tick_heartbeat, this));

  callback_group_mut_ex_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  cli_register_ = create_client<autoware_control_center_msgs::srv::Register>(
    "/autoware/control_center/srv/register", rmw_qos_profile_default, callback_group_mut_ex_);

  cli_report_status_ = create_client<autoware_control_center_msgs::srv::ReportState>(
    "/autoware/control_center/srv/report_status", rmw_qos_profile_default, callback_group_mut_ex_);

  timer_registration_ =
    this->create_wall_timer(register_timer_period, std::bind(&Node::on_tick_registration, this));
}

void Node::on_tick_registration()
{
  RCLCPP_DEBUG(get_logger(), "Register callback");
  if (is_registered_) {
    RCLCPP_DEBUG(get_logger(), "It was registered before");
    return;
  }

  if (!cli_register_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "%s is unavailable", cli_register_->get_service_name());
    return;
  }
  autoware_control_center_msgs::srv::Register::Request::SharedPtr req =
    std::make_shared<autoware_control_center_msgs::srv::Register::Request>();
  req->name_node = self_name_;

  cli_register_->async_send_request(
    req, std::bind(&Node::on_register, this, std::placeholders::_1));
  RCLCPP_DEBUG(get_logger(), "Sent request");

  const std::string msg = self_name_ + " node started";
  autoware_control_center_msgs::msg::NodeState node_state;
  node_state.status = autoware_control_center_msgs::msg::NodeState::NORMAL;
  send_state(node_state, msg);
  RCLCPP_DEBUG(get_logger(), "Sent node state");
}

void Node::on_tick_heartbeat()
{
  auto message = autoware_control_center_msgs::msg::Heartbeat();
  message.stamp = this->get_clock()->now();
  message.sequence_number = sequence_number_++;
  RCLCPP_DEBUG(this->get_logger(), "Publishing heartbeat, sent at [%i]", message.stamp.sec);
  pub_heartbeat_->publish(message);
}

void Node::send_state(
  const autoware_control_center_msgs::msg::NodeState & node_state, std::string message)
{
  if (!cli_report_status_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "%s is unavailable", cli_report_status_->get_service_name());
    return;
  }
  autoware_control_center_msgs::srv::ReportState::Request::SharedPtr req =
    std::make_shared<autoware_control_center_msgs::srv::ReportState::Request>();

  req->name_node = self_name_;
  req->state = node_state;
  req->message = std::move(message);

  cli_report_status_->async_send_request(
    req, std::bind(&Node::on_report_status, this, std::placeholders::_1));
  RCLCPP_DEBUG(get_logger(), "Send node state");
}

// performance-unnecessary-value-param
// TODO(xmfcx): add the line above the line below once next cpplint is released (1.7.0 or 2.0.0)
// NOLINTNEXTLINE
void Node::on_register(FutureRegister future)
{
  const auto & response = future.get();
  std::string str_uuid = autoware_utils::to_hex_string(response->uuid_node);
  RCLCPP_DEBUG(get_logger(), "response: %d, %s", response->status.status, str_uuid.c_str());

  if (response->status.status == autoware_control_center_msgs::msg::NodeStatus::SUCCESS) {
    is_registered_ = true;
    self_uuid = response->uuid_node;
    RCLCPP_DEBUG(get_logger(), "Node was registered");
    timer_registration_->cancel();
    RCLCPP_DEBUG(get_logger(), "Register timer was cancelled");
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to register node");
  }
}

// performance-unnecessary-value-param
// TODO(xmfcx): add the line above the line below once next cpplint is released (1.7.0 or 2.0.0)
// NOLINTNEXTLINE
void Node::on_report_status(FutureReportStatus future)
{
  const auto & response = future.get();
  std::string str_uuid = autoware_utils::to_hex_string(response->uuid_node);
  RCLCPP_DEBUG(
    get_logger(), "response: %d, %s, %s", response->status.status, str_uuid.c_str(),
    response->log_response.c_str());

  if (response->status.status == autoware_control_center_msgs::msg::NodeStatus::SUCCESS) {
    RCLCPP_DEBUG(get_logger(), "Node state was received by ACC");
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to send Node state to ACC");
  }
}

}  // namespace autoware::node
