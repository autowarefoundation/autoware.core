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

#include <autoware_utils/ros/uuid_helper.hpp>
#include <control_center_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_control_center_msgs/srv/register.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>

class ControlCenterTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions node_options;
    node_options.append_parameter_override("lease_duration_ms", 220.0);
    node_options.append_parameter_override("report_publish_rate", 1.0);
    control_center_ = std::make_shared<autoware::control_center::ControlCenter>(node_options);
  }

  void TearDown() override { rclcpp::shutdown(); }
  autoware::control_center::ControlCenter::SharedPtr control_center_;
};

TEST_F(ControlCenterTest, RegisterNode)
{
  auto client = control_center_->create_client<autoware_control_center_msgs::srv::Register>(
    "/autoware/control_center/srv/register");
  if (!client->wait_for_service(std::chrono::seconds(20))) {
    ASSERT_TRUE(false) << "Node register service not available after waiting";
  }

  auto request = std::make_shared<autoware_control_center_msgs::srv::Register::Request>();
  request->name_node = "test_node";

  auto result = client->async_send_request(request);
  auto ret = rclcpp::spin_until_future_complete(
    control_center_, result, std::chrono::seconds(5));  // Wait for the result.

  ASSERT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);

  EXPECT_EQ(1, result.get()->status.status);
}

TEST_F(ControlCenterTest, DeregisterNode)
{
  std::string node_name = "test_node";
  auto client_reg = control_center_->create_client<autoware_control_center_msgs::srv::Register>(
    "/autoware/control_center/srv/register");

  if (!client_reg->wait_for_service(std::chrono::seconds(20))) {
    ASSERT_TRUE(false) << "Node register service not available after waiting";
  }
  // Register node first
  auto request_reg = std::make_shared<autoware_control_center_msgs::srv::Register::Request>();
  request_reg->name_node = node_name;

  auto result_reg = client_reg->async_send_request(request_reg);
  auto ret = rclcpp::spin_until_future_complete(
    control_center_, result_reg, std::chrono::seconds(5));  // Wait for the result.

  ASSERT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
  auto result_reg_payload = result_reg.get();
  EXPECT_EQ(1, result_reg_payload->status.status) << "Node register request fail";
  unique_identifier_msgs::msg::UUID uuid_node = result_reg_payload->uuid_node;

  // cspell:ignore dereg
  // Deregister node
  auto client_dereg = control_center_->create_client<autoware_control_center_msgs::srv::Deregister>(
    "/autoware/control_center/srv/deregister");
  if (!client_dereg->wait_for_service(std::chrono::seconds(20))) {
    ASSERT_TRUE(false) << "Node deregister service not available after waiting";
  }

  auto request_dereg = std::make_shared<autoware_control_center_msgs::srv::Deregister::Request>();
  request_dereg->name_node = node_name;

  auto result_dereg = client_dereg->async_send_request(request_dereg);
  ret = rclcpp::spin_until_future_complete(control_center_, result_dereg, std::chrono::seconds(5));

  ASSERT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
  auto result_dereg_payload = result_dereg.get();
  EXPECT_EQ(1, result_dereg_payload->status.status) << "Node deregister request fail";
  EXPECT_EQ(uuid_node, result_dereg_payload->uuid_node) << "ACC return wrong uuid";
}

TEST_F(ControlCenterTest, NodeErrorServiceNotRegistered)
{
  std::string node_name = "test_node";
  auto client = control_center_->create_client<autoware_control_center_msgs::srv::ReportState>(
    "/autoware/control_center/srv/report_state");

  if (!client->wait_for_service(std::chrono::seconds(20))) {
    ASSERT_TRUE(false) << "Node error service not available after waiting";
  }

  auto error_request = std::make_shared<autoware_control_center_msgs::srv::ReportState::Request>();
  error_request->name_node = node_name;
  autoware_control_center_msgs::msg::NodeState node_state;
  node_state.status = autoware_control_center_msgs::msg::NodeState::NORMAL;
  error_request->state = node_state;
  error_request->message = "test message";

  auto result_error = client->async_send_request(error_request);
  auto ret =
    rclcpp::spin_until_future_complete(control_center_, result_error, std::chrono::seconds(5));

  ASSERT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
  auto result_error_payload = result_error.get();
  EXPECT_EQ(0, result_error_payload->status.status) << "Node error request wrong status";
  std::string log_msg = node_name + " node was not registered.";
  EXPECT_EQ(log_msg, result_error_payload->log_response) << "Received wrong log response";
  EXPECT_EQ(autoware_utils::generate_default_uuid(), result_error_payload->uuid_node)
    << "Received wrong uuid";
}

TEST_F(ControlCenterTest, NodeErrorService)
{
  std::string node_name = "test_node";
  auto client_reg = control_center_->create_client<autoware_control_center_msgs::srv::Register>(
    "/autoware/control_center/srv/register");

  if (!client_reg->wait_for_service(std::chrono::seconds(20))) {
    ASSERT_TRUE(false) << "Node register service not available after waiting";
  }
  // Register node first
  auto request_reg = std::make_shared<autoware_control_center_msgs::srv::Register::Request>();
  request_reg->name_node = node_name;

  auto result_reg = client_reg->async_send_request(request_reg);
  auto ret_reg = rclcpp::spin_until_future_complete(
    control_center_, result_reg, std::chrono::seconds(5));  // Wait for the result.

  ASSERT_EQ(ret_reg, rclcpp::FutureReturnCode::SUCCESS);
  auto result_reg_payload = result_reg.get();
  EXPECT_EQ(1, result_reg_payload->status.status) << "Node register request fail";
  unique_identifier_msgs::msg::UUID uuid_node = result_reg_payload->uuid_node;

  auto error_client =
    control_center_->create_client<autoware_control_center_msgs::srv::ReportState>(
      "/autoware/control_center/srv/report_state");

  if (!error_client->wait_for_service(std::chrono::seconds(20))) {
    ASSERT_TRUE(false) << "Node error service not available after waiting";
  }

  auto error_request = std::make_shared<autoware_control_center_msgs::srv::ReportState::Request>();
  error_request->name_node = node_name;
  autoware_control_center_msgs::msg::NodeState node_state;
  node_state.status = autoware_control_center_msgs::msg::NodeState::NORMAL;
  error_request->state = node_state;
  error_request->message = "test message";

  auto result_error = error_client->async_send_request(error_request);
  auto ret_err =
    rclcpp::spin_until_future_complete(control_center_, result_error, std::chrono::seconds(5));

  ASSERT_EQ(ret_err, rclcpp::FutureReturnCode::SUCCESS);
  auto result_error_payload = result_error.get();
  EXPECT_EQ(1, result_error_payload->status.status) << "Node error request wrong status";
  EXPECT_EQ(uuid_node, result_error_payload->uuid_node) << "Received wrong uuid";
}