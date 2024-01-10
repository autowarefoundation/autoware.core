// Copyright 2022 The Autoware Contributors
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

#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/ros/uuid_helper.hpp>

#include "autoware_control_center_msgs/srv/autoware_node_register.hpp"

#include <chrono>


using namespace std::chrono_literals;

namespace autoware_node
{

AutowareNode::AutowareNode(
  const std::string & node_name, const std::string & ns, const rclcpp::NodeOptions & options)
: LifecycleNode(node_name, ns, options)
{
  RCLCPP_INFO(get_logger(), "AutowareNode::AutowareNode()");
  self_name = this->get_name();
  callback_group_mut_ex_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  cli_register_ = create_client<autoware_control_center_msgs::srv::AutowareNodeRegister>(
    "/autoware_control_center/srv/autoware_node_register", rmw_qos_profile_default,
    callback_group_mut_ex_);

  register_timer_ = this->create_wall_timer(
    500ms, std::bind(&AutowareNode::register_callback, this));

  using std::placeholders::_1;
  using std::placeholders::_2;
  srv_deregister_ = create_service<autoware_control_center_msgs::srv::AutowareControlCenterDeregister>(
    "~/srv/acc_deregister", std::bind(&AutowareNode::deregister, this, _1, _2), 
    rmw_qos_profile_services_default, callback_group_mut_ex_);  
}

void AutowareNode::register_callback()
{
  RCLCPP_INFO(get_logger(), "Register callback");
  if (registered) {
    RCLCPP_INFO(get_logger(), "It was registered before");
    return;
  }

  if (!cli_register_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "%s is unavailable", cli_register_->get_service_name());
    return;
  }

  autoware_control_center_msgs::srv::AutowareNodeRegister::Request::SharedPtr req =
    std::make_shared<autoware_control_center_msgs::srv::AutowareNodeRegister::Request>();

  req->name_node = self_name;

  using ServiceResponseFuture =
    rclcpp::Client<autoware_control_center_msgs::srv::AutowareNodeRegister>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
      auto response = future.get();
      std::string str_uuid = tier4_autoware_utils::toHexString(response->uuid_node);
      RCLCPP_INFO(get_logger(), "response: %d, %s", response->status.status, str_uuid.c_str());

      if (response->status.status == 1) {
        registered = true;
        self_uuid = response->uuid_node;
        RCLCPP_INFO(get_logger(), "Node was registered");
        register_timer_->cancel();
        RCLCPP_INFO(get_logger(), "Register timer was cancelled");
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to register node");
      }
    };

  auto future_result = cli_register_->async_send_request(req, response_received_callback);

  RCLCPP_INFO(get_logger(), "Sent request");
}

void AutowareNode::deregister(
  const autoware_control_center_msgs::srv::AutowareControlCenterDeregister::Request::SharedPtr request,
  const autoware_control_center_msgs::srv::AutowareControlCenterDeregister::Response::SharedPtr response)
{
  RCLCPP_INFO(get_logger(), "Deregister callback");
  std::string str_uuid = tier4_autoware_utils::toHexString(request->uuid_acc);
  RCLCPP_INFO(get_logger(), "Request from %s", str_uuid.c_str());
  response->name_node = self_name;
  
  if (!registered) {
    RCLCPP_WARN(get_logger(), "Node wasn't registered");
    response->status.status = 
      autoware_control_center_msgs::srv::AutowareControlCenterDeregister::Response::_status_type::FAILURE;
  } else {
    registered = false;
    response->status.status =
      autoware_control_center_msgs::srv::AutowareControlCenterDeregister::Response::_status_type::SUCCESS;
    this->register_timer_->reset();
  }
}

}  // namespace autoware_node
