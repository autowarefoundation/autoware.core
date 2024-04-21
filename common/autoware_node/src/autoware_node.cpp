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

#include "autoware_node/autoware_node.hpp"

#include <autoware_utils/ros/uuid_helper.hpp>
#include <rclcpp/rclcpp.hpp>

#include "autoware_control_center_msgs/msg/status.hpp"
#include "autoware_control_center_msgs/srv/autoware_node_register.hpp"

#include <chrono>

constexpr std::chrono::milliseconds lease_delta(
  20);  ///< Buffer added to heartbeat to define lease.

namespace autoware_node
{

AutowareNode::AutowareNode(
  const std::string & node_name, const std::string & ns, const rclcpp::NodeOptions & options)
: LifecycleNode(node_name, ns, options)
{
  RCLCPP_DEBUG(get_logger(), "AutowareNode::AutowareNode()");
  declare_parameter<int>(
    "heartbeat_period", 200);  // TODO(lexavtanke): remove default and add schema
  declare_parameter<int>("register_timer_period", 500);
  std::chrono::milliseconds heartbeat_period(get_parameter("heartbeat_period").as_int());
  std::chrono::milliseconds register_timer_period(get_parameter("register_timer_period").as_int());
  std::string self_namespace(this->get_namespace());
  std::string name(this->get_name());
  if (self_namespace.length() > 1) {
    self_name = self_namespace + "/" + name;
  } else {
    self_name = name;
  }
  sequence_number_ = 0;
  registered = false;

  // The granted lease is essentially infinite here, i.e., only reader/watchdog will notify
  // violations. XXX causes segfault for cyclone dds, hence pass explicit lease life > heartbeat.
  rclcpp::QoS qos_profile(1);
  qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC)
    .liveliness_lease_duration(heartbeat_period + lease_delta)
    .deadline(heartbeat_period + lease_delta);

  // assert liveliness on the 'heartbeat' topic
  heartbeat_pub_ = this->create_publisher<autoware_control_center_msgs::msg::Heartbeat>(
    "~/heartbeat", qos_profile);
  heartbeat_timer_ =
    this->create_wall_timer(heartbeat_period, std::bind(&AutowareNode::heartbeat_callback, this));

  callback_group_mut_ex_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  cli_register_ = create_client<autoware_control_center_msgs::srv::AutowareNodeRegister>(
    "/autoware_control_center/srv/autoware_node_register", rmw_qos_profile_default,
    callback_group_mut_ex_);

  register_timer_ = this->create_wall_timer(
    register_timer_period, std::bind(&AutowareNode::register_callback, this));

  using std::placeholders::_1;
  using std::placeholders::_2;
  srv_deregister_ =
    create_service<autoware_control_center_msgs::srv::AutowareControlCenterDeregister>(
      "~/srv/acc_deregister", std::bind(&AutowareNode::deregister, this, _1, _2),
      rmw_qos_profile_services_default, callback_group_mut_ex_);

  cli_node_error_ = create_client<autoware_control_center_msgs::srv::AutowareNodeError>(
    "/autoware_control_center/srv/autoware_node_error", rmw_qos_profile_default,
    callback_group_mut_ex_);
}

void AutowareNode::register_callback()
{
  RCLCPP_DEBUG(get_logger(), "Register callback");
  if (registered) {
    RCLCPP_DEBUG(get_logger(), "It was registered before");
    return;
  }

  if (!cli_register_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "%s is unavailable", cli_register_->get_service_name());
    return;
  }
  autoware_control_center_msgs::srv::AutowareNodeRegister::Request::SharedPtr req =
    std::make_shared<autoware_control_center_msgs::srv::AutowareNodeRegister::Request>();
  req->name_node = self_name;

  cli_register_->async_send_request(
    req, std::bind(&AutowareNode::node_register_future_callback, this, std::placeholders::_1));
  RCLCPP_DEBUG(get_logger(), "Sent request");

  const std::string msg = self_name + " node started";
  autoware_control_center_msgs::msg::AutowareNodeState node_state;
  node_state.status = autoware_control_center_msgs::msg::AutowareNodeState::NORMAL;
  send_state(node_state, msg);
  RCLCPP_DEBUG(get_logger(), "Sent node state");
}

void AutowareNode::heartbeat_callback()
{
  auto message = autoware_control_center_msgs::msg::Heartbeat();
  message.stamp = this->get_clock()->now();
  message.sequence_number = sequence_number_++;
  RCLCPP_DEBUG(this->get_logger(), "Publishing heartbeat, sent at [%i]", message.stamp.sec);
  heartbeat_pub_->publish(message);
}

void AutowareNode::deregister(
  const autoware_control_center_msgs::srv::AutowareControlCenterDeregister::Request::SharedPtr
    request,
  const autoware_control_center_msgs::srv::AutowareControlCenterDeregister::Response::SharedPtr
    response)
{
  RCLCPP_DEBUG(get_logger(), "Deregister callback");
  std::string str_uuid = autoware_utils::to_hex_string(request->uuid_acc);
  RCLCPP_DEBUG(get_logger(), "Request from %s", str_uuid.c_str());
  response->name_node = self_name;

  if (!registered) {
    RCLCPP_WARN(get_logger(), "Node wasn't registered");
    response->status.status = autoware_control_center_msgs::srv::AutowareControlCenterDeregister::
      Response::_status_type::FAILURE;
  } else {
    RCLCPP_WARN(get_logger(), "Node deregistered");
    registered = false;
    response->status.status = autoware_control_center_msgs::srv::AutowareControlCenterDeregister::
      Response::_status_type::SUCCESS;
    response->log_response = self_name + " was deregistered";
    response->name_node = self_name;
    this->register_timer_->reset();
  }
}

void AutowareNode::send_state(
  const autoware_control_center_msgs::msg::AutowareNodeState & node_state, std::string message)
{
  if (!cli_node_error_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "%s is unavailable", cli_node_error_->get_service_name());
    return;
  }
  autoware_control_center_msgs::srv::AutowareNodeError::Request::SharedPtr req =
    std::make_shared<autoware_control_center_msgs::srv::AutowareNodeError::Request>();

  req->name_node = self_name;
  req->state = node_state;
  req->message = std::move(message);

  cli_node_error_->async_send_request(
    req, std::bind(&AutowareNode::node_error_future_callback, this, std::placeholders::_1));
  RCLCPP_DEBUG(get_logger(), "Send node state");
}
// NOLINTNEXTLINE(performance-unnecessary-value-param)
void AutowareNode::node_register_future_callback(AutowareNodeRegisterServiceResponseFuture future)
{
  const auto & response = future.get();
  std::string str_uuid = autoware_utils::to_hex_string(response->uuid_node);
  RCLCPP_DEBUG(get_logger(), "response: %d, %s", response->status.status, str_uuid.c_str());

  if (response->status.status == autoware_control_center_msgs::msg::Status::SUCCESS) {
    registered = true;
    self_uuid = response->uuid_node;
    RCLCPP_DEBUG(get_logger(), "Node was registered");
    register_timer_->cancel();
    RCLCPP_DEBUG(get_logger(), "Register timer was cancelled");
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to register node");
  }
}
// NOLINTNEXTLINE(performance-unnecessary-value-param)
void AutowareNode::node_error_future_callback(AutowareNodeErrorServiceResponseFuture future)
{
  const auto & response = future.get();
  std::string str_uuid = autoware_utils::to_hex_string(response->uuid_node);
  RCLCPP_DEBUG(
    get_logger(), "response: %d, %s, %s", response->status.status, str_uuid.c_str(),
    response->log_response.c_str());

  if (response->status.status == autoware_control_center_msgs::msg::Status::SUCCESS) {
    RCLCPP_DEBUG(get_logger(), "Node state was received by ACC");
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to send Node state to ACC");
  }
}

}  // namespace autoware_node
