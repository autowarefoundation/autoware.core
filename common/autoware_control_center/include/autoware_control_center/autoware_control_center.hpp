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

#ifndef AUTOWARE_CONTROL_CENTER__AUTOWARE_CONTROL_CENTER_HPP_
#define AUTOWARE_CONTROL_CENTER__AUTOWARE_CONTROL_CENTER_HPP_

#include "autoware_control_center/visibility_control.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "autoware_control_center_msgs/msg/autoware_control_center_json.hpp"

namespace autoware_control_center
{

class AutowareControlCenter : public rclcpp_lifecycle::LifecycleNode
{
public:
  AutowareControlCenter(const rclcpp::NodeOptions & options);

private:
  using AutowareControlCenterJson = autoware_control_center_msgs::msg::AutowareControlCenterJson;
  rclcpp::Subscription<AutowareControlCenterJson>::SharedPtr sub_global_;

  void callback_acc_json_global(const AutowareControlCenterJson::ConstSharedPtr & msg);
};

}  // namespace autoware_control_center

#endif  // AUTOWARE_CONTROL_CENTER__AUTOWARE_CONTROL_CENTER_HPP_
