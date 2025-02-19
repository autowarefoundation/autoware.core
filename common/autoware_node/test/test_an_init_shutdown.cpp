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

#include <gtest/gtest.h>

#include <memory>
#include <string>

class AutowareNodeInitShutdown : public ::testing::Test
{
public:
  void SetUp() override { rclcpp::init(0, nullptr); }

  void TearDown() override { rclcpp::shutdown(); }

  rclcpp::NodeOptions node_options_an_;
};

// Helper function to wait until the executor starts spinning with a timeout.
bool wait_for_executor_spinning(
  const std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> & executor,
  std::chrono::milliseconds timeout)
{
  auto start_time = std::chrono::steady_clock::now();
  while (!executor->is_spinning()) {
    if (std::chrono::steady_clock::now() - start_time > timeout) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return true;
}

TEST_F(AutowareNodeInitShutdown, NodeInitShutdown)
{
  const std::string node_name = "test_node";
  const std::string node_ns = "test_ns";

  auto autoware_node = std::make_shared<autoware::node::Node>(node_name, node_ns, node_options_an_);

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(autoware_node->get_node_base_interface());

  // Start the executor in a separate thread.
  std::thread executor_thread([executor]() { executor->spin(); });

  ASSERT_EQ(autoware_node->get_fully_qualified_name(), "/" + node_ns + "/" + node_name);

  // Wait until the executor starts spinning (timeout after 2 seconds).
  ASSERT_TRUE(wait_for_executor_spinning(executor, std::chrono::milliseconds(2000)))
    << "Executor did not start spinning within the expected timeout.";

  executor->cancel();
  if (executor_thread.joinable()) {
    executor_thread.join();
  }

  ASSERT_FALSE(executor->is_spinning());
}
