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

#include <map>
#include <string>
#include <unordered_map>
#include <vector>

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
  /*!
  MutuallyExclusive callback group for services provided by the Autoware Control Center.
  */
  rclcpp::CallbackGroup::SharedPtr callback_group_mut_ex_;

  /*!
  Service for registering an Autoware Node to the Autoware Control Service.
  */
  rclcpp::Service<autoware_control_center_msgs::srv::AutowareNodeRegister>::SharedPtr srv_register_;
  /*!
  Service for deregistering an Autoware Node from the Autoware Contol Service.
  */
  rclcpp::Service<autoware_control_center_msgs::srv::AutowareNodeDeregister>::SharedPtr
    srv_deregister_;
  /*!
  Service to receive a status report from an Autoware Node.
  */
  rclcpp::Service<autoware_control_center_msgs::srv::AutowareNodeError>::SharedPtr srv_node_error_;
  /*!
  Publisher for the autoware_node_reports topic.
  */
  rclcpp::Publisher<autoware_control_center_msgs::msg::AutowareNodeReports>::SharedPtr
    node_reports_pub_;
  /*!
  Store registered nodes names and uuid. Provide methods for registration, de-registration and others.
  */
  NodeRegistry node_registry_;
  /*!
  Store subscribers to all heartbeat topics of registered Autoware Nodes. 
  */
  std::unordered_map<
    std::string, rclcpp::Subscription<autoware_control_center_msgs::msg::Heartbeat>::SharedPtr>
    heartbeat_sub_map_;
  /*!
  Store information about registered Autoware Nodes statuses.
  */
  std::unordered_map<std::string, AutowareNodeStatus> node_status_map_;
  /*!
  Control startup procedure of the  Autoware Control Center.
  */
  rclcpp::TimerBase::SharedPtr startup_timer_;
  /*!
  Control publishing to the autoware_node_reports topic.
  */
  rclcpp::TimerBase::SharedPtr node_reports_timer_;
  /*!
  UUID of the Autoware Control Center instance.
  */
  unique_identifier_msgs::msg::UUID acc_uuid_;
  /*!
  The lease duration granted to the remote (heartbeat) publisher.
  */
  std::chrono::milliseconds lease_duration_;
  /*!
  The period for node registration during the startup of the Autoware Control Center.
  */
  double startup_duration_;
  /*!
  The timestamp of the Autoware Control Center startup.
  */
  rclcpp::Time startup_timestamp_;
  /*!
  The flag of startup for deregistration of Autoware Nodes.
  */
  bool startup_;

  /*!
  Callback for the register node service.  
  */
  void register_node(
    const autoware_control_center_msgs::srv::AutowareNodeRegister::Request::SharedPtr request,
    const autoware_control_center_msgs::srv::AutowareNodeRegister::Response::SharedPtr response);
  
  /*!
  Callback for the deregister node service.  
  */
  void deregister_node(
    const autoware_control_center_msgs::srv::AutowareNodeDeregister::Request::SharedPtr request,
    const autoware_control_center_msgs::srv::AutowareNodeDeregister::Response::SharedPtr response);
  
  /*!
  Callback for the autoware error node service. 
  It reseives status message from Autoware node and store information to the node_status_map_. 
  */
  void autoware_node_error(
    const autoware_control_center_msgs::srv::AutowareNodeError::Request::SharedPtr request,
    const autoware_control_center_msgs::srv::AutowareNodeError::Response::SharedPtr response);

  /*!
  Callback for the startup timer. Check if there is no registered node for the particular period of time. 
  After it will list all nodes in the system and call the deregister service of them.  
  */
  void startup_callback();

  /*!
  Create subscription with the proper QoS to the heartbeat topic of the node with node_name.
  \param[node_name] The name of the node to which you want to create a subscription.
  */
  rclcpp::Subscription<autoware_control_center_msgs::msg::Heartbeat>::SharedPtr
  create_heartbeat_sub(const std::string & node_name);

  /*!
  Publish node reports to autoware_node_reports topic. 
  */
  void node_reports_callback();

  /*!
  Update node_status_map_ info for the node with node name if liveliness of the heartbeat topic publisher has changed.
  \param[node_name] The name of the node which info should be updated.
  */
  void liveliness_callback(rclcpp::QOSLivelinessChangedInfo & event, const std::string & node_name);

  /*!
  Update node_status_map_ info for the node with node name  then heartbeat message is reseived.
  \param[node_name] The name of the node which info should be updated.
  */
  void heartbeat_callback(
    const typename autoware_control_center_msgs::msg::Heartbeat::SharedPtr msg,
    const std::string & node_name);
  
  /*!
  Filter out deregister services from list of all services in the system by type.
  \param[srv_list] The list of services available in the system.
  */
  void filter_deregister_services(std::map<std::string, std::vector<std::string>> & srv_list);
};

}  // namespace autoware_control_center

#endif  // AUTOWARE_CONTROL_CENTER__AUTOWARE_CONTROL_CENTER_HPP_
