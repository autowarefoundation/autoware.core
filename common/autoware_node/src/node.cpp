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

#include <autoware/node/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>

namespace autoware::node
{
Node::Node(
  const std::string & node_name, const std::string & ns, const rclcpp::NodeOptions & options)
: LifecycleNode(node_name, ns, options)
{
  RCLCPP_DEBUG(
    get_logger(), "Node %s constructor was called.",
    get_node_base_interface()->get_fully_qualified_name());
}

CallbackReturn Node::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_DEBUG(
    get_logger(), "Node %s shutdown was called with state %s.",
    get_node_base_interface()->get_fully_qualified_name(), state.label().c_str());
  return CallbackReturn::SUCCESS;
}
}  // namespace autoware::node
