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

#ifndef AUTOWARE_CONTROL_CENTER__AUTOWARE_CONTROL_CENTER_HPP_
#define AUTOWARE_CONTROL_CENTER__AUTOWARE_CONTROL_CENTER_HPP_

#include "autoware_control_center/node_registry.hpp"
#include "autoware_control_center/visibility_control.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "autoware_control_center_msgs/msg/autoware_node_reports.hpp"
#include "autoware_control_center_msgs/msg/heartbeat.hpp"
#include "autoware_control_center_msgs/srv/autoware_node_deregister.hpp"
#include "autoware_control_center_msgs/srv/autoware_node_error.hpp"
#include "autoware_control_center_msgs/srv/autoware_node_register.hpp"

#include <string>
#include <unordered_map>

namespace autoware_control_center
{

enum class HealthState { Unknown = 0, Healthy = 1, Warning = 2, Error = 3 };

struct AutowareNodeStatus
{
  bool alive;
  rclcpp::Time last_heartbeat;
  std::string node_report;
  autoware_control_center_msgs::msg::AutowareNodeState state;
};

class AutowareControlCenter : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit AutowareControlCenter(const rclcpp::NodeOptions & options);

private:
  rclcpp::CallbackGroup::SharedPtr callback_group_mut_ex_;

  rclcpp::Service<autoware_control_center_msgs::srv::AutowareNodeRegister>::SharedPtr srv_register_;
  rclcpp::Service<autoware_control_center_msgs::srv::AutowareNodeDeregister>::SharedPtr
    srv_deregister_;
  rclcpp::Service<autoware_control_center_msgs::srv::AutowareNodeError>::SharedPtr srv_node_error_;
  rclcpp::Publisher<autoware_control_center_msgs::msg::AutowareNodeReports>::SharedPtr
    node_reports_pub_;
  NodeRegistry node_registry_;
  std::unordered_map<
    std::string, rclcpp::Subscription<autoware_control_center_msgs::msg::Heartbeat>::SharedPtr>
    heartbeat_sub_map_;
  std::unordered_map<std::string, AutowareNodeStatus> node_status_map_;

  rclcpp::TimerBase::SharedPtr startup_timer_;
  rclcpp::TimerBase::SharedPtr node_reports_timer_;
  int countdown;
  unique_identifier_msgs::msg::UUID acc_uuid;
  /// The lease duration granted to the remote (heartbeat) publisher
  std::chrono::milliseconds lease_duration_;
  bool startup;

  void register_node(
    const autoware_control_center_msgs::srv::AutowareNodeRegister::Request::SharedPtr request,
    const autoware_control_center_msgs::srv::AutowareNodeRegister::Response::SharedPtr response);

  void deregister_node(
    const autoware_control_center_msgs::srv::AutowareNodeDeregister::Request::SharedPtr request,
    const autoware_control_center_msgs::srv::AutowareNodeDeregister::Response::SharedPtr response);

  void autoware_node_error(
    const autoware_control_center_msgs::srv::AutowareNodeError::Request::SharedPtr request,
    const autoware_control_center_msgs::srv::AutowareNodeError::Response::SharedPtr response);

  void startup_callback();

  rclcpp::Subscription<autoware_control_center_msgs::msg::Heartbeat>::SharedPtr
  create_heartbeat_sub(const std::string & node_name);

  void node_reports_callback();
  void liveliness_callback(rclcpp::QOSLivelinessChangedInfo & event, const std::string & node_name);
  void heartbeat_callback(
    const typename autoware_control_center_msgs::msg::Heartbeat::SharedPtr msg,
    const std::string & node_name);
  void filter_deregister_services(std::map<std::string, std::vector<std::string>> & srv_list);
};

}  // namespace autoware_control_center

#endif  // AUTOWARE_CONTROL_CENTER__AUTOWARE_CONTROL_CENTER_HPP_
