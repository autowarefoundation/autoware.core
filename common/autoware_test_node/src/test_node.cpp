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

#include "include/test_node.hpp"

#include <autoware/node/node.hpp>

namespace autoware::test_node
{

TestNode::TestNode(const rclcpp::NodeOptions & options)
: autoware::node::Node("test_node", "", options)
{
  RCLCPP_INFO(
    get_logger(), "TestNode constructor with name %s", autoware::node::Node::self_name.c_str());

  using std::placeholders::_1;
  monitored_subscription_ =
    autoware::node::Node::create_monitored_subscription<std_msgs::msg::String>(
      "topic", 10, 10, std::bind(&TestNode::topic_callback, this, _1));
}

void TestNode::topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}

}  // namespace autoware::test_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::test_node::TestNode)
