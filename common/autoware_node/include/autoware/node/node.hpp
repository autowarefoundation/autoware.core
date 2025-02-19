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

#ifndef AUTOWARE__NODE__NODE_HPP_
#define AUTOWARE__NODE__NODE_HPP_

#include "autoware/node/visibility_control.hpp"

#include <rclcpp/node.hpp>

#include <string>

namespace autoware::node
{
class Node : public rclcpp::Node
{
public:
  AUTOWARE_NODE_PUBLIC
  explicit Node(
    const std::string & node_name, const std::string & ns = "",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
};
}  // namespace autoware::node

#endif  // AUTOWARE__NODE__NODE_HPP_
