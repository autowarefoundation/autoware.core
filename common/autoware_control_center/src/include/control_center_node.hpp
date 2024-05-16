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

#ifndef CONTROL_CENTER_NODE_HPP_
#define CONTROL_CENTER_NODE_HPP_

#include "node_registry.hpp"

#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <autoware_control_center_msgs/msg/heartbeat.hpp>
#include <autoware_control_center_msgs/msg/node_reports.hpp>
#include <autoware_control_center_msgs/msg/node_state.hpp>
#include <autoware_control_center_msgs/srv/deregister.hpp>
#include <autoware_control_center_msgs/srv/register.hpp>
#include <autoware_control_center_msgs/srv/report_state.hpp>

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::control_center
{

class ControlCenter : public rclcpp_lifecycle::LifecycleNode
{
public:
  using SharedPtr = std::shared_ptr<ControlCenter>;
  explicit ControlCenter(const rclcpp::NodeOptions & options);

private:
  using Register = autoware_control_center_msgs::srv::Register;
  using Deregister = autoware_control_center_msgs::srv::Deregister;
  using ReportState = autoware_control_center_msgs::srv::ReportState;
  using NodeReports = autoware_control_center_msgs::msg::NodeReports;
  using Heartbeat = autoware_control_center_msgs::msg::Heartbeat;
  using NodeState = autoware_control_center_msgs::msg::NodeState;

  enum class HealthState { Unknown = 0, Healthy = 1, Warning = 2, Error = 3 };

  struct AutowareNodeStatus
  {
    bool alive;
    rclcpp::Time last_heartbeat;
    std::string node_report;
    NodeState state;
  };

  const std::chrono::duration<double, std::milli> lease_duration_ms_;
  const double report_publish_rate_;

  rclcpp::CallbackGroup::SharedPtr callback_group_mut_ex_;

  rclcpp::Service<Register>::SharedPtr srv_register_;
  rclcpp::Service<Deregister>::SharedPtr srv_deregister_;
  rclcpp::Service<ReportState>::SharedPtr srv_report_state_;

  rclcpp::Publisher<NodeReports>::SharedPtr pub_reports_;

  rclcpp::TimerBase::SharedPtr timer_publish_reports_;

  NodeRegistry node_registry_;
  std::unordered_map<std::string, rclcpp::Subscription<Heartbeat>::SharedPtr> heartbeat_sub_map_;
  std::unordered_map<std::string, AutowareNodeStatus> node_status_map_;
  unique_identifier_msgs::msg::UUID acc_uuid_;

  void on_register_node(
    Register::Request::SharedPtr request, Register::Response::SharedPtr response);

  void on_deregister_node(
    Deregister::Request::SharedPtr request, Deregister::Response::SharedPtr response);

  void on_report_state(
    ReportState::Request::SharedPtr request, ReportState::Response::SharedPtr response);

  void on_tick_publish_reports();

  void liveliness_callback(rclcpp::QOSLivelinessChangedInfo & event, const std::string & node_name);

  void heartbeat_callback(Heartbeat::SharedPtr msg, const std::string & node_name);

  rclcpp::Subscription<Heartbeat>::SharedPtr create_heartbeat_sub(const std::string & node_name);
};

}  // namespace autoware::control_center

#endif  // CONTROL_CENTER_NODE_HPP_
