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

#ifndef AUTOWARE_CONTROL_CENTER__AUTOWARE_CONTROL_CENTER_HPP_
#define AUTOWARE_CONTROL_CENTER__AUTOWARE_CONTROL_CENTER_HPP_

#include "autoware_control_center/node_registry.hpp"
#include "autoware_control_center/visibility_control.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "autoware_control_center_msgs/srv/autoware_node_register.hpp"
#include "autoware_control_center_msgs/srv/autoware_node_deregister.hpp"

namespace autoware_control_center
{
unique_identifier_msgs::msg::UUID createDefaultUUID();

class AutowareControlCenter : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit AutowareControlCenter(const rclcpp::NodeOptions & options);

private:
  rclcpp::CallbackGroup::SharedPtr callback_group_mut_ex_;

  rclcpp::Service<autoware_control_center_msgs::srv::AutowareNodeRegister>::SharedPtr srv_register_;
  rclcpp::Service<autoware_control_center_msgs::srv::AutowareNodeDeregister>::SharedPtr
    srv_deregister_;

  NodeRegistry node_registry_;

  rclcpp::TimerBase::SharedPtr startup_timer_;
  int countdown;
  unique_identifier_msgs::msg::UUID acc_uuid;

  void register_node(
    const autoware_control_center_msgs::srv::AutowareNodeRegister::Request::SharedPtr request,
    const autoware_control_center_msgs::srv::AutowareNodeRegister::Response::SharedPtr response);

  void deregister_node(
    const autoware_control_center_msgs::srv::AutowareNodeDeregister::Request::SharedPtr request,
    const autoware_control_center_msgs::srv::AutowareNodeDeregister::Response::SharedPtr response);
  
  void startup_callback();

};

}  // namespace autoware_control_center

#endif  // AUTOWARE_CONTROL_CENTER__AUTOWARE_CONTROL_CENTER_HPP_
