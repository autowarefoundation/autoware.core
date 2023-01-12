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

#ifndef TEST_NODE__TEST_NODE_HPP_
#define TEST_NODE__TEST_NODE_HPP_

#include "autoware_node/autoware_node.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "test_node/visibility_control.hpp"

namespace test_node
{

class TestNode : public autoware_node::AutowareNode
{
public:
  TEST_NODE_PUBLIC
  explicit TestNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
};

}  // namespace test_node

#endif  // TEST_NODE__TEST_NODE_HPP_
