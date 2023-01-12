// Copyright 2022 The Autoware Contributors
// SPDX-License-Identifier: Apache-2.0
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

#include "autoware_control_center_msgs/srv/autoware_node_register.hpp"

namespace autoware_node
{

AutowareNode::AutowareNode(
  const std::string & node_name, const std::string & ns, const rclcpp::NodeOptions & options)
: LifecycleNode(node_name, ns, options)
{
  callback_group_mut_ex_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  cli_register_ = create_client<autoware_control_center_msgs::srv::AutowareNodeRegister>(
    "~/cli/autoware_node_register", rmw_qos_profile_default, callback_group_mut_ex_);
}

}  // namespace autoware_node
