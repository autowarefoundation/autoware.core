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

#include "autoware_control_center/autoware_control_center.hpp"

namespace autoware_control_center
{
AutowareControlCenter::AutowareControlCenter(const rclcpp::NodeOptions & options)
: LifecycleNode("autoware_control_center", options)
{
  // log info
  RCLCPP_INFO(get_logger(), "AutowareControlCenter is initialized");
}

void AutowareControlCenter::callback_acc_json_global(
  const autoware_control_center_msgs::msg::AutowareControlCenterJson::ConstSharedPtr & msg)
{
  // log message
  RCLCPP_INFO(get_logger(), "AutowareControlCenterJson received %s", msg->json_message.c_str());
}

}  // namespace autoware_control_center
