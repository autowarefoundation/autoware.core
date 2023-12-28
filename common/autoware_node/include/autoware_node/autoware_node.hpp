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

#ifndef AUTOWARE_NODE__AUTOWARE_NODE_HPP_
#define AUTOWARE_NODE__AUTOWARE_NODE_HPP_

#include "autoware_node/visibility_control.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "autoware_control_center_msgs/srv/autoware_node_register.hpp"
#include "autoware_control_center_msgs/srv/autoware_control_center_deregister.hpp"

#include <string>

namespace autoware_node
{

class AutowareNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  AUTOWARE_NODE_PUBLIC
  explicit AutowareNode(
    const std::string & node_name, const std::string & ns = "",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  rclcpp::CallbackGroup::SharedPtr callback_group_mut_ex_;

  rclcpp::Client<autoware_control_center_msgs::srv::AutowareNodeRegister>::SharedPtr cli_register_;
  rclcpp::Service<autoware_control_center_msgs::srv::AutowareControlCenterDeregister>::SharedPtr srv_deregister_;
  rclcpp::TimerBase::SharedPtr register_timer_;
  bool registered;
  unique_identifier_msgs::msg::UUID self_uuid;
  unique_identifier_msgs::msg::UUID acc_uuid;
  std::string self_name;

private:
  void register_callback();
  void deregister(
    const autoware_control_center_msgs::srv::AutowareControlCenterDeregister::Request::SharedPtr request,
    const autoware_control_center_msgs::srv::AutowareControlCenterDeregister::Response::SharedPtr response
  );
};

}  // namespace autoware_node

#endif  // AUTOWARE_NODE__AUTOWARE_NODE_HPP_
