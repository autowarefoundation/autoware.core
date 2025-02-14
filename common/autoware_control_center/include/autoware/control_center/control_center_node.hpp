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

#ifndef AUTOWARE__CONTROL_CENTER__CONTROL_CENTER_NODE_HPP_
#define AUTOWARE__CONTROL_CENTER__CONTROL_CENTER_NODE_HPP_

#include <rclcpp/node.hpp>

#include <autoware_control_center_msgs/msg/node_report.hpp>
#include <autoware_control_center_msgs/msg/node_reports.hpp>
#include <autoware_control_center_msgs/msg/node_status_activity.hpp>
#include <autoware_control_center_msgs/msg/node_status_operational.hpp>
#include <autoware_control_center_msgs/srv/deregister.hpp>
#include <autoware_control_center_msgs/srv/register.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace autoware::control_center
{

class ControlCenter : public rclcpp::Node
{
public:
  using SharedPtr = std::shared_ptr<ControlCenter>;
  using NodeReport = autoware_control_center_msgs::msg::NodeReport;
  explicit ControlCenter(const rclcpp::NodeOptions & options);

  struct NodeInfo
  {
    using SharedPtr = std::shared_ptr<NodeInfo>;
    using ConstSharedPtr = std::shared_ptr<const NodeInfo>;

    NodeReport report;
  };

private:
  using NodeReports = autoware_control_center_msgs::msg::NodeReports;
  using NodeStatusActivity = autoware_control_center_msgs::msg::NodeStatusActivity;
  using NodeStatusOperational = autoware_control_center_msgs::msg::NodeStatusOperational;
  using ResultDeregistration = autoware_control_center_msgs::msg::ResultDeregistration;
  using ResultRegistration = autoware_control_center_msgs::msg::ResultRegistration;
  using Deregister = autoware_control_center_msgs::srv::Deregister;
  using Register = autoware_control_center_msgs::srv::Register;

  std::tuple<ResultRegistration, std::optional<unique_identifier_msgs::msg::UUID>> register_node(
    const std::string & node_name);
  ControlCenter::ResultDeregistration deregister_node(
    const unique_identifier_msgs::msg::UUID & uuid);

  bool is_registered(const unique_identifier_msgs::msg::UUID & uuid) const;
  bool is_registered(const std::string & name) const;

  /// @brief Retrieves the UUID of a registered node.
  /// @note Ensure the node is registered using `is_registered(node_name)` before calling this
  /// function.
  unique_identifier_msgs::msg::UUID get_uuid(const std::string & node_name) const;

  /// @brief Retrieves the NodeInfo of a registered node.
  /// @note Ensure the node is registered using `is_registered(uuid)` before calling this function.
  ControlCenter::NodeInfo::ConstSharedPtr get_node_info(
    const unique_identifier_msgs::msg::UUID & uuid) const;

  void publish_node_reports();

  void on_register_node(
    const std::shared_ptr<Register::Request> request, std::shared_ptr<Register::Response> response);

  void on_deregister_node(
    const std::shared_ptr<Deregister::Request> request,
    std::shared_ptr<Deregister::Response> response);

  rclcpp::Service<Register>::SharedPtr srv_register_;
  rclcpp::Service<Deregister>::SharedPtr srv_deregister_;
  rclcpp::Publisher<NodeReports>::SharedPtr pub_reports_;
  rclcpp::TimerBase::SharedPtr timer_publish_reports_;

  std::map<std::string, unique_identifier_msgs::msg::UUID> node_name_to_uuid_;
  std::map<std::array<uint8_t, 16>, NodeInfo::SharedPtr> uuid_to_info_;

  double report_publish_rate_;
};

}  // namespace autoware::control_center

#endif  // AUTOWARE__CONTROL_CENTER__CONTROL_CENTER_NODE_HPP_
