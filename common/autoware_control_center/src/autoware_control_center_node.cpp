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

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto autoware_control_center =
    std::make_shared<autoware_control_center::AutowareControlCenter>(rclcpp::NodeOptions());
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(autoware_control_center->get_node_base_interface());
  exec.spin();
  rclcpp::shutdown();

  return 0;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware_control_center::AutowareControlCenter)
