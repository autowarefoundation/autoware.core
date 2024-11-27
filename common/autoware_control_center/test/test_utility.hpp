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

#ifndef TEST_UTILITY_HPP_
#define TEST_UTILITY_HPP_

#include "test_constants.hpp"

#include <autoware_utils/ros/uuid_helper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_control_center_msgs/msg/heartbeat.hpp>
#include <autoware_control_center_msgs/msg/node_report.hpp>
#include <autoware_control_center_msgs/msg/node_reports.hpp>
#include <autoware_control_center_msgs/srv/deregister.hpp>
#include <autoware_control_center_msgs/srv/register.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <tuple>

namespace autoware::control_center::test
{

inline std::tuple<unique_identifier_msgs::msg::UUID, rclcpp::Node::SharedPtr> register_node(
  const std::string & node_name, const std::string & ns)
{
  auto test_node = std::make_shared<rclcpp::Node>(node_name, ns);

  auto client =
    test_node->create_client<autoware_control_center_msgs::srv::Register>(topic_register_service);
  auto request = std::make_shared<autoware_control_center_msgs::srv::Register::Request>();
  request->node_name_with_namespace = test_node->get_fully_qualified_name();

  RCLCPP_INFO(rclcpp::get_logger("registering"), "Initializing service");
  if (!client->wait_for_service(std::chrono::seconds(5))) {
    throw std::runtime_error("Service initialization timeout");
  }

  RCLCPP_INFO(rclcpp::get_logger("registering"), "Waiting for response");
  auto fut_result = client->async_send_request(request);
  if (
    rclcpp::spin_until_future_complete(test_node, fut_result, std::chrono::seconds(5)) !=
    rclcpp::FutureReturnCode::SUCCESS) {
    throw std::runtime_error("Service call timeout or failed");
  }

  auto result = fut_result.get();
  if (
    result->result_registration.result !=
    autoware_control_center_msgs::msg::ResultRegistration::SUCCESS) {
    throw std::runtime_error("Node registration failed");
  }
  RCLCPP_INFO(rclcpp::get_logger("registering"), "Received the response");

  return {result->uuid_node, test_node};
}

inline bool deregister_node(const unique_identifier_msgs::msg::UUID & uuid)
{
  auto test_node = std::make_shared<rclcpp::Node>("test_node");

  auto client = test_node->create_client<autoware_control_center_msgs::srv::Deregister>(
    topic_deregister_service);
  auto request = std::make_shared<autoware_control_center_msgs::srv::Deregister::Request>();
  request->uuid_node = uuid;

  if (!client->wait_for_service(std::chrono::seconds(5))) {
    return false;
  }

  auto fut_result = client->async_send_request(request);
  if (
    rclcpp::spin_until_future_complete(test_node, fut_result, std::chrono::seconds(5)) !=
    rclcpp::FutureReturnCode::SUCCESS) {
    return false;
  }

  auto result = fut_result.get();
  return result->result_deregistration.result ==
         autoware_control_center_msgs::msg::ResultDeregistration::SUCCESS;
}

inline bool wait_for_node_report(
  const std::string & node_full_name, autoware_control_center_msgs::msg::NodeReport & report,
  const std::int64_t timeout_ms = 5000)
{
  std::promise<bool> report_received;
  auto future_report = report_received.get_future();

  auto node_temp = std::make_shared<rclcpp::Node>("node_report_listener", "");

  auto node_reports_sub =
    node_temp->create_subscription<autoware_control_center_msgs::msg::NodeReports>(
      topic_node_reports, rclcpp::QoS(rclcpp::KeepLast(0)),
      [&report_received, &report,
       &node_full_name](const autoware_control_center_msgs::msg::NodeReports::SharedPtr msg) {
        for (const auto & node_report : msg->reports) {
          if (node_report.name == node_full_name) {
            report = node_report;
            report_received.set_value(true);
            return;
          }
        }
        report_received.set_value(false);
      });

  if (
    rclcpp::spin_until_future_complete(
      node_temp, future_report, std::chrono::milliseconds(timeout_ms)) !=
    rclcpp::FutureReturnCode::SUCCESS) {
    return false;
  }
  return future_report.get();
}

inline bool is_node_registered(const std::string & node_full_name)
{
  autoware_control_center_msgs::msg::NodeReport report;
  return wait_for_node_report(node_full_name, report);
}

inline void send_heartbeat(
  const rclcpp::Node::SharedPtr & node,
  const rclcpp::Publisher<autoware_control_center_msgs::msg::Heartbeat>::SharedPtr & pub,
  const autoware_control_center_msgs::msg::Heartbeat::SharedPtr & heartbeat)
{
  pub->publish(*heartbeat);
  rclcpp::spin_some(node);
}

inline rclcpp::Publisher<autoware_control_center_msgs::msg::Heartbeat>::SharedPtr
send_first_heartbeat(
  const rclcpp::Node::SharedPtr & node, const double deadline_ms,
  const autoware_control_center_msgs::msg::Heartbeat::SharedPtr & heartbeat)
{
  const std::string fully_qualified_name = node->get_fully_qualified_name();

  rclcpp::QoS qos_profile(1);
  qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC)
    .liveliness_lease_duration(std::chrono::duration<double, std::milli>(deadline_ms))
    .deadline(std::chrono::duration<double, std::milli>(deadline_ms));

  auto pub = node->create_publisher<autoware_control_center_msgs::msg::Heartbeat>(
    fully_qualified_name + test::topic_heartbeat_suffix, qos_profile);
  send_heartbeat(node, pub, heartbeat);
  return pub;
}

autoware_control_center_msgs::msg::Heartbeat::SharedPtr generate_hb_healthy()
{
  auto heartbeat = std::make_shared<autoware_control_center_msgs::msg::Heartbeat>();
  heartbeat->stamp = rclcpp::Clock().now();
  heartbeat->status_activity.status =
    autoware_control_center_msgs::msg::NodeStatusActivity::PROCESSING;
  heartbeat->status_operational.status =
    autoware_control_center_msgs::msg::NodeStatusOperational::NORMAL;
  return heartbeat;
}
}  // namespace autoware::control_center::test

#endif  // TEST_UTILITY_HPP_
