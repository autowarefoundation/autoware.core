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

#include <autoware_utils/ros/uuid_helper.hpp>

#include "autoware_control_center_msgs/srv/autoware_node_register.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <string>

using std::chrono::operator""ms;
using std::chrono::operator""s;

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
  auto result_reg_payload = result_reg.get();
  EXPECT_EQ(1, result_reg_payload->status.status) << "Node register request fail";
  unique_identifier_msgs::msg::UUID uuid_node = result_reg_payload->uuid_node;

  // cspell:ignore dereg
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
  auto result_dereg_payload = result_dereg.get();
  EXPECT_EQ(1, result_dereg_payload->status.status) << "Node deregister request fail";
  EXPECT_EQ(uuid_node, result_dereg_payload->uuid_node) << "ACC return wrong uuid";
}

TEST_F(AutowareControlCenterTest, NodeErrorServiceNotRegistered)
{
  std::string node_name = "test_node";
  auto client =
    autoware_control_center_->create_client<autoware_control_center_msgs::srv::AutowareNodeError>(
    "/autoware_control_center/srv/autoware_node_error");

  if (!client->wait_for_service(20s)) {
    ASSERT_TRUE(false) << "Node error service not available after waiting";
  }

  auto error_request =
    std::make_shared<autoware_control_center_msgs::srv::AutowareNodeError::Request>();
  error_request->name_node = node_name;
  autoware_control_center_msgs::msg::AutowareNodeState node_state;
  node_state.status = autoware_control_center_msgs::msg::AutowareNodeState::NORMAL;
  error_request->state = node_state;
  error_request->message = "test message";

  auto result_error = client->async_send_request(error_request);
  auto ret = rclcpp::spin_until_future_complete(autoware_control_center_, result_error, 5s);

  ASSERT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
  auto result_error_payload = result_error.get();
  EXPECT_EQ(0, result_error_payload->status.status) << "Node error request wrong status";
  std::string log_msg = node_name + " node was not registered.";
  EXPECT_EQ(log_msg, result_error_payload->log_response) << "Received wrong log response";
  EXPECT_EQ(autoware_utils::generate_default_uuid(), result_error_payload->uuid_node)
    << "Received wrong uuid";
}

TEST_F(AutowareControlCenterTest, NodeErrorService)
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
  auto ret_reg = rclcpp::spin_until_future_complete(
    autoware_control_center_, result_reg, 5s);  // Wait for the result.

  ASSERT_EQ(ret_reg, rclcpp::FutureReturnCode::SUCCESS);
  auto result_reg_payload = result_reg.get();
  EXPECT_EQ(1, result_reg_payload->status.status) << "Node register request fail";
  unique_identifier_msgs::msg::UUID uuid_node = result_reg_payload->uuid_node;

  auto error_client =
    autoware_control_center_->create_client<autoware_control_center_msgs::srv::AutowareNodeError>(
    "/autoware_control_center/srv/autoware_node_error");

  if (!error_client->wait_for_service(20s)) {
    ASSERT_TRUE(false) << "Node error service not available after waiting";
  }

  auto error_request =
    std::make_shared<autoware_control_center_msgs::srv::AutowareNodeError::Request>();
  error_request->name_node = node_name;
  autoware_control_center_msgs::msg::AutowareNodeState node_state;
  node_state.status = autoware_control_center_msgs::msg::AutowareNodeState::NORMAL;
  error_request->state = node_state;
  error_request->message = "test message";

  auto result_error = error_client->async_send_request(error_request);
  auto ret_err = rclcpp::spin_until_future_complete(autoware_control_center_, result_error, 5s);

  ASSERT_EQ(ret_err, rclcpp::FutureReturnCode::SUCCESS);
  auto result_error_payload = result_error.get();
  EXPECT_EQ(1, result_error_payload->status.status) << "Node error request wrong status";
  EXPECT_EQ(uuid_node, result_error_payload->uuid_node) << "Received wrong uuid";
}
