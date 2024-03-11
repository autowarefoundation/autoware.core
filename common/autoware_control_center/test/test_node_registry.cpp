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

#include <gtest/gtest.h>

#include <autoware_utils/ros/uuid_helper.hpp>

#include "autoware_control_center/node_registry.hpp"

// Test fixture for NodeRegistry
class NodeRegistryTest : public ::testing::Test
{
protected:
  autoware_control_center::NodeRegistry registry;
};

TEST_F(NodeRegistryTest, RegisterNodeTest)
{
  std::string nodeName = "test_node";
  unique_identifier_msgs::msg::UUID uuid;
  uuid = autoware_utils::generate_uuid();

  auto result = registry.register_node(nodeName, uuid);

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result.value(), uuid);
  EXPECT_TRUE(registry.is_registered(nodeName));
}

TEST_F(NodeRegistryTest, SameNameRegister)
{
  std::string nodeName = "test_node";
  unique_identifier_msgs::msg::UUID first_uuid;
  first_uuid = autoware_utils::generate_uuid();

  auto first_result = registry.register_node(nodeName, first_uuid);
  ASSERT_TRUE(first_result.has_value());

  unique_identifier_msgs::msg::UUID second_uuid;
  second_uuid = autoware_utils::generate_uuid();
  auto second_result = registry.register_node(nodeName, second_uuid);

  ASSERT_TRUE(second_result.has_value());
  EXPECT_EQ(second_result.value(), second_uuid);
  EXPECT_TRUE(registry.is_registered(nodeName));
}

TEST_F(NodeRegistryTest, DeregisterNodeTest)
{
  std::string nodeName = "test_node";
  unique_identifier_msgs::msg::UUID uuid;
  uuid = autoware_utils::generate_uuid();

  registry.register_node(nodeName, uuid);
  auto result = registry.deregister_node(nodeName);

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result.value(), uuid);
  EXPECT_FALSE(registry.is_registered(nodeName));
}

TEST_F(NodeRegistryTest, GetUUIDTest)
{
  std::string nodeName = "test_node";
  unique_identifier_msgs::msg::UUID uuid;
  uuid = autoware_utils::generate_uuid();

  registry.register_node(nodeName, uuid);
  auto result = registry.get_uuid(nodeName);

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result.value(), uuid);
}

TEST_F(NodeRegistryTest, IsEmptyTest)
{
  EXPECT_TRUE(registry.is_empty());

  registry.register_node("test_node", unique_identifier_msgs::msg::UUID());

  EXPECT_FALSE(registry.is_empty());
}
