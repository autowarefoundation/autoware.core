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
  sequence_number_{0},
  is_registered_{false},
  period_timer_register_ms_{declare_parameter<double>("period_timer_register_ms")},
  period_heartbeat_ms_{declare_parameter<double>("period_heartbeat_ms")},
  deadline_ms_{declare_parameter<double>("deadline_ms")}
{
  RCLCPP_DEBUG(
    get_logger(), "Node %s constructor", get_node_base_interface()->get_fully_qualified_name());

  rclcpp::QoS qos_profile(1);
  qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC)
    .liveliness_lease_duration(std::chrono::duration<double, std::milli>(deadline_ms_))
    .deadline(std::chrono::duration<double, std::milli>(deadline_ms_));

  pub_heartbeat_ = this->create_publisher<autoware_control_center_msgs::msg::Heartbeat>(
    "~/heartbeat", qos_profile);

  callback_group_mut_ex_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  timer_heartbeat_ = this->create_wall_timer(
    std::chrono::duration<double, std::milli>(period_heartbeat_ms_),
    std::bind(&Node::on_tick_heartbeat, this), callback_group_mut_ex_);

  cli_register_ = create_client<autoware_control_center_msgs::srv::Register>(
    "/autoware/control_center/srv/register", rmw_qos_profile_default, callback_group_mut_ex_);

  timer_registration_ = this->create_wall_timer(
    std::chrono::duration<double, std::milli>(period_timer_register_ms_),
    std::bind(&Node::on_tick_registration, this), callback_group_mut_ex_);
}

void Node::on_tick_heartbeat()
{
  auto heartbeat = std::make_shared<autoware_control_center_msgs::msg::Heartbeat>();
  heartbeat->stamp = rclcpp::Clock().now();
  heartbeat->status_activity.status =
    autoware_control_center_msgs::msg::NodeStatusActivity::PROCESSING;
  heartbeat->status_operational.status =
    autoware_control_center_msgs::msg::NodeStatusOperational::NORMAL;
  heartbeat->sequence_number = sequence_number_;
  pub_heartbeat_->publish(*heartbeat);
  sequence_number_++;
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
  req->node_name_with_namespace = get_node_base_interface()->get_fully_qualified_name();

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

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Node::on_shutdown(
  const rclcpp_lifecycle::State & state)
{
  destroy_node();
  return LifecycleNode::on_shutdown(state);
}

void Node::destroy_node()
{
  timer_registration_.reset();
  timer_heartbeat_.reset();
  cli_register_.reset();
  pub_heartbeat_.reset();
}
}  // namespace autoware::node
