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

#include "test_autoware_control_center.hpp"

#include "autoware_control_center/autoware_control_center.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "autoware_control_center_msgs/srv/autoware_node_register.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class AutowareControlCenterTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    autoware_control_center_ =
      std::make_shared<autoware_control_center::AutowareControlCenter>(rclcpp::NodeOptions());
  }

  void TearDown() override {rclcpp::shutdown();}
  autoware_control_center::AutowareControlCenter::SharedPtr autoware_control_center_;
};

TEST_F(AutowareControlCenterTest, RegisterNode)
{
  auto client = autoware_control_center_
    ->create_client<autoware_control_center_msgs::srv::AutowareNodeRegister>(
    "/autoware_control_center/srv/autoware_node_register");
  if (!client->wait_for_service(20s)) {
    ASSERT_TRUE(false) << "Node register service not available after waiting";
  }

  auto request =
    std::make_shared<autoware_control_center_msgs::srv::AutowareNodeRegister::Request>();
  request->name_node = "test_node";

  auto result = client->async_send_request(request);
  auto ret = rclcpp::spin_until_future_complete(
    autoware_control_center_, result, 5s);  // Wait for the result.

  ASSERT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);

  EXPECT_EQ(1, result.get()->status.status);
}

TEST_F(AutowareControlCenterTest, DeregisterNode)
{
  std::string node_name = "test_node";
  auto client_reg = autoware_control_center_
    ->create_client<autoware_control_center_msgs::srv::AutowareNodeRegister>(
    "/autoware_control_center/srv/autoware_node_register");

  if (!client_reg->wait_for_service(20s)) {
    ASSERT_TRUE(false) << "Node register service not available after waiting";
  }
  // Register node first
  auto request_reg =
    std::make_shared<autoware_control_center_msgs::srv::AutowareNodeRegister::Request>();
  request_reg->name_node = node_name;

  auto result_reg = client_reg->async_send_request(request_reg);
  auto ret = rclcpp::spin_until_future_complete(
    autoware_control_center_, result_reg, 5s);  // Wait for the result.

  ASSERT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(1, result_reg.get()->status.status) << "Node register request fail";
  // TODO(lexavtanke) add uuid check

  // Deregister node
  auto client_dereg = autoware_control_center_
    ->create_client<autoware_control_center_msgs::srv::AutowareNodeDeregister>(
    "/autoware_control_center/srv/autoware_node_deregister");
  if (!client_dereg->wait_for_service(20s)) {
    ASSERT_TRUE(false) << "Node deregister service not available after waiting";
  }

  auto request_dereg =
    std::make_shared<autoware_control_center_msgs::srv::AutowareNodeDeregister::Request>();
  request_dereg->name_node = node_name;

  auto result_dereg = client_dereg->async_send_request(request_dereg);
  ret = rclcpp::spin_until_future_complete(autoware_control_center_, result_dereg, 5s);

  ASSERT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(1, result_dereg.get()->status.status) << "Node deregister request fail";
  // TODO(lexavtanke) add uuid check
}
