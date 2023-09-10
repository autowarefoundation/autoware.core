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

namespace autoware_node
{

AutowareNode::AutowareNode(
  const std::string & node_name, const std::string & ns, const rclcpp::NodeOptions & options)
: LifecycleNode(node_name, ns, options)
{
  RCLCPP_INFO(get_logger(), "AutowareNode::AutowareNode()");
  callback_group_mut_ex_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  cli_register_ = create_client<autoware_control_center_msgs::srv::AutowareNodeRegister>(
    "/autoware_control_center/srv/autoware_node_register", rmw_qos_profile_default,
    callback_group_mut_ex_);

  autoware_control_center_msgs::srv::AutowareNodeRegister::Request::SharedPtr req =
    std::make_shared<autoware_control_center_msgs::srv::AutowareNodeRegister::Request>();

  req->name_node = node_name;

  auto fut_and_id_response = cli_register_->async_send_request(req);

  RCLCPP_INFO(get_logger(), "Sent request");

  //  const auto & response = fut_and_id_response.get();
  //  RCLCPP_INFO(get_logger(), "response: %d", response->status.status);

  if (
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), fut_and_id_response) ==
    rclcpp::FutureReturnCode::SUCCESS) {
    auto response = fut_and_id_response.get();
    std::string str_uuid = tier4_autoware_utils::toHexString(response->uuid_node);
    RCLCPP_INFO(
      get_logger(), "response: %d, %s", response->status.status, str_uuid.c_str());
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to call service");
  }
}

}  // namespace autoware_node
