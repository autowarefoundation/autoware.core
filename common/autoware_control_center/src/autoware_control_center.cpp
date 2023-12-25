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

#include "autoware_control_center/autoware_control_center.hpp"

#include "autoware_control_center/node_registry.hpp"

#include <tier4_autoware_utils/ros/uuid_helper.hpp>

#include <unique_identifier_msgs/msg/uuid.hpp>

namespace autoware_control_center
{
unique_identifier_msgs::msg::UUID createDefaultUUID()
{
  unique_identifier_msgs::msg::UUID default_uuid;

  // Use std::generate to fill the UUID with zeros
  std::generate(default_uuid.uuid.begin(), default_uuid.uuid.end(), []() {return 0;});

  return default_uuid;
}

AutowareControlCenter::AutowareControlCenter(const rclcpp::NodeOptions & options)
: LifecycleNode("autoware_control_center", options)
{
  // log info
  RCLCPP_INFO(get_logger(), "AutowareControlCenter is initialized");

  callback_group_mut_ex_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  using std::placeholders::_1;
  using std::placeholders::_2;
  srv_register_ = create_service<autoware_control_center_msgs::srv::AutowareNodeRegister>(
    "~/srv/autoware_node_register", std::bind(&AutowareControlCenter::register_node, this, _1, _2),
    rmw_qos_profile_services_default, callback_group_mut_ex_);
  srv_deregister_ = create_service<autoware_control_center_msgs::srv::AutowareNodeDeregister>(
    "~/srv/autoware_node_deregister",
    std::bind(&AutowareControlCenter::deregister_node, this, _1, _2),
    rmw_qos_profile_services_default, callback_group_mut_ex_);
}

void AutowareControlCenter::register_node(
  const autoware_control_center_msgs::srv::AutowareNodeRegister::Request::SharedPtr request,
  const autoware_control_center_msgs::srv::AutowareNodeRegister::Response::SharedPtr response)
{
  RCLCPP_INFO(get_logger(), "register_node is called from %s", request->name_node.c_str());

  std::optional<unique_identifier_msgs::msg::UUID> node_uuid =
    node_registry_.register_node(request->name_node.c_str(), tier4_autoware_utils::generateUUID());

  if (node_uuid == std::nullopt) {
    response->uuid_node = createDefaultUUID();
    response->status.status =
      autoware_control_center_msgs::srv::AutowareNodeRegister::Response::_status_type::FAILURE;
  } else {
    response->uuid_node = node_uuid.value();
    response->status.status =
      autoware_control_center_msgs::srv::AutowareNodeRegister::Response::_status_type::SUCCESS;
  }
}

void AutowareControlCenter::deregister_node(
  const autoware_control_center_msgs::srv::AutowareNodeDeregister::Request::SharedPtr request,
  const autoware_control_center_msgs::srv::AutowareNodeDeregister::Response::SharedPtr response)
{
  RCLCPP_INFO(get_logger(), "deregister_node is called from %s", request->name_node.c_str());

  std::optional<unique_identifier_msgs::msg::UUID> node_uuid =
    node_registry_.deregister_node(request->name_node.c_str());

  if (node_uuid == std::nullopt) {
    response->uuid_node = createDefaultUUID();
    response->status.status =
      autoware_control_center_msgs::srv::AutowareNodeDeregister::Response::_status_type::FAILURE;
  } else {
    response->uuid_node = node_uuid.value();
    response->status.status =
      autoware_control_center_msgs::srv::AutowareNodeDeregister::Response::_status_type::SUCCESS;
  }
}

}  // namespace autoware_control_center
