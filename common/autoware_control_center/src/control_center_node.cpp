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

#include "autoware/control_center/control_center_node.hpp"

#include <autoware_utils/ros/uuid_helper.hpp>

#include <unique_identifier_msgs/msg/uuid.hpp>

#include <chrono>

namespace autoware::control_center
{

ControlCenter::ControlCenter(const rclcpp::NodeOptions & options)
: LifecycleNode("control_center", "autoware", options),
  deadline_ms_{declare_parameter<double>("deadline_ms")},
  report_publish_rate_{declare_parameter<double>("report_publish_rate")}
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  const rclcpp::QoS qos_profile =
    rclcpp::QoS(rclcpp::KeepLast(999)).reliable().durability_volatile();
  const rmw_qos_profile_t rmw_qos_profile = qos_profile.get_rmw_qos_profile();

  srv_register_ = create_service<Register>(
    "~/srv/register", std::bind(&ControlCenter::on_register_node, this, _1, _2), rmw_qos_profile);
  srv_deregister_ = create_service<Deregister>(
    "~/srv/deregister", std::bind(&ControlCenter::on_deregister_node, this, _1, _2),
    rmw_qos_profile);

  timer_publish_reports_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / report_publish_rate_),
    std::bind(&ControlCenter::publish_node_reports, this));

  pub_reports_ = create_publisher<NodeReports>("~/node_reports", 10);
}

std::tuple<ControlCenter::ResultRegistration, std::optional<unique_identifier_msgs::msg::UUID>>
ControlCenter::register_node(const std::string & node_name)
{
  RCLCPP_DEBUG(this->get_logger(), "Registering node %s", node_name.c_str());

  auto node_info = std::make_shared<NodeInfo>();

  auto & report = node_info->report;

  if (is_registered(node_name)) {
    const auto & uuid_old = get_uuid(node_name);
    if (!is_registered(uuid_old)) {
      // This should not be possible
      throw std::logic_error("Node name is registered but UUID is not");
    }
    const auto & node_info_old = get_node_info(uuid_old);
    report.count_registered = node_info_old->report.count_registered;
    deregister_node(uuid_old);
  }

  report.count_registered++;
  report.name = node_name;
  report.uuid = autoware_utils::generate_uuid();
  report.stamp_registration = rclcpp::Clock().now();
  report.is_alive = true;
  report.status_activity.status = NodeStatusActivity::INITIALIZING;
  report.status_operational.status = NodeStatusOperational::WARNING;
  report.stamp_last_heartbeat = rclcpp::Clock().now();
  node_name_to_uuid_[node_name] = node_info->report.uuid;
  uuid_to_info_[node_info->report.uuid.uuid] = node_info;

  const std::chrono::milliseconds deadline_duration{static_cast<int>(deadline_ms_)};
  std::string topic_name = node_name + "/heartbeat";
  //  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).deadline(deadline_duration);
  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10))
                       .liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC)
                       .liveliness_lease_duration(deadline_duration);

  std::function<void(const autoware_control_center_msgs::msg::Heartbeat::SharedPtr)>
    on_heartbeat_wrapped =
      std::bind(&ControlCenter::on_heartbeat, this, std::placeholders::_1, node_info->report.uuid);

  rclcpp::SubscriptionOptions heartbeat_sub_options;
  //  auto bound_on_deadline = std::bind(
  //    &ControlCenter::on_deadline_missed, this, std::placeholders::_1, node_info->report.uuid);
  //  heartbeat_sub_options.event_callbacks.deadline_callback = bound_on_deadline;
  auto bound_on_liveliness = std::bind(
    &ControlCenter::on_liveliness_changed, this, std::placeholders::_1, node_info->report.uuid);
  heartbeat_sub_options.event_callbacks.liveliness_callback = bound_on_liveliness;

  node_info->heartbeat_sub =
    this->create_subscription<autoware_control_center_msgs::msg::Heartbeat>(
      topic_name, qos_profile, on_heartbeat_wrapped, heartbeat_sub_options);

  ResultRegistration result_registration;
  result_registration.result = ResultRegistration::SUCCESS;
  return std::make_tuple(result_registration, node_info->report.uuid);
}

ControlCenter::ResultDeregistration ControlCenter::deregister_node(
  const unique_identifier_msgs::msg::UUID & uuid)
{
  ResultDeregistration result;
  if (!is_registered(uuid)) {
    RCLCPP_WARN(this->get_logger(), "Attempt to deregister non-existent node with UUID");
    result.result = ResultDeregistration::FAILURE_NOT_REGISTERED;
    return result;
  }

  auto node_info = uuid_to_info_.at(uuid.uuid);
  const auto & node_name = node_info->report.name;
  RCLCPP_DEBUG(this->get_logger(), "Deregistering node %s", node_name.c_str());

  uuid_to_info_.erase(uuid.uuid);

  if (!is_registered(node_name)) {
    // This should not be possible
    throw std::logic_error("Node name is registered but UUID is not");
  }
  node_name_to_uuid_.erase(node_name);

  result.result = ResultDeregistration::SUCCESS;
  return result;
}

bool ControlCenter::is_registered(const unique_identifier_msgs::msg::UUID & uuid) const
{
  return uuid_to_info_.find(uuid.uuid) != uuid_to_info_.end();
}

bool ControlCenter::is_registered(const std::string & name) const
{
  return node_name_to_uuid_.find(name) != node_name_to_uuid_.end();
}

ControlCenter::NodeInfo::ConstSharedPtr ControlCenter::get_node_info(
  const unique_identifier_msgs::msg::UUID & uuid) const
{
  return uuid_to_info_.at(uuid.uuid);
}

unique_identifier_msgs::msg::UUID ControlCenter::get_uuid(const std::string & node_name) const
{
  return node_name_to_uuid_.at(node_name);
}

void ControlCenter::on_heartbeat(
  const autoware_control_center_msgs::msg::Heartbeat::SharedPtr heartbeat,
  const unique_identifier_msgs::msg::UUID & uuid)
{
  if (!is_registered(uuid)) {
    RCLCPP_WARN(this->get_logger(), "Received heartbeat for unregistered node");
    return;
  }
  auto node_info = uuid_to_info_[uuid.uuid];
  node_info->report.stamp_last_heartbeat = heartbeat->stamp;
  node_info->report.status_activity = heartbeat->status_activity;
  node_info->report.status_operational = heartbeat->status_operational;
  node_info->report.is_alive = true;
}

void ControlCenter::on_deadline_missed(
  rclcpp::QOSDeadlineRequestedInfo & event, const unique_identifier_msgs::msg::UUID & uuid)
{
  if (!is_registered(uuid)) {
    RCLCPP_WARN(this->get_logger(), "Deadline missed for unregistered node");
    return;
  }

  auto & node_report = uuid_to_info_.at(uuid.uuid)->report;
  node_report.is_alive = false;
  node_report.status_activity.status = NodeStatusActivity::UNKNOWN;
  node_report.status_operational.status = NodeStatusOperational::UNKNOWN;
  RCLCPP_WARN(
    this->get_logger(), "Deadline missed for node %s: total_count=%d, total_count_change=%d",
    node_report.name.c_str(), event.total_count, event.total_count_change);
}

void ControlCenter::on_liveliness_changed(
  rclcpp::QOSLivelinessChangedInfo & event, const unique_identifier_msgs::msg::UUID & uuid)
{
  if (!is_registered(uuid)) {
    RCLCPP_WARN(this->get_logger(), "Liveliness changed for node for unregistered node");
    return;
  }

  auto & node_report = uuid_to_info_.at(uuid.uuid)->report;
  RCLCPP_WARN(
    this->get_logger(), "Liveliness changed for node %s: alive_count=%d, not_alive_count=%d",
    node_report.name.c_str(), event.alive_count, event.not_alive_count);

  // Only consider the alive -> not alive transition
  if (event.alive_count != 0) return;

  RCLCPP_DEBUG(this->get_logger(), "Node %s is not alive", node_report.name.c_str());
  node_report.is_alive = false;
  node_report.status_activity.status = NodeStatusActivity::UNKNOWN;
  node_report.status_operational.status = NodeStatusOperational::UNKNOWN;
}

void ControlCenter::publish_node_reports()
{
  NodeReports node_reports_msg;
  node_reports_msg.stamp = rclcpp::Clock().now();
  node_reports_msg.reports.resize(uuid_to_info_.size());

  std::transform(
    uuid_to_info_.begin(), uuid_to_info_.end(), node_reports_msg.reports.begin(),
    [](const auto & entry) { return entry.second->report; });

  pub_reports_->publish(node_reports_msg);
}

void ControlCenter::on_register_node(
  const std::shared_ptr<Register::Request> request, std::shared_ptr<Register::Response> response)
{
  const auto & node_name = request->node_name_with_namespace;
  auto [result_registration, node_uuid] = register_node(node_name);

  response->result_registration = result_registration;

  if (result_registration.result != ResultRegistration::SUCCESS) {
    RCLCPP_WARN(get_logger(), "Node registration failed for %s", node_name.c_str());
    return;
  }

  response->uuid_node = node_uuid.value();
}

void ControlCenter::on_deregister_node(
  const std::shared_ptr<Deregister::Request> request,
  std::shared_ptr<Deregister::Response> response)
{
  const auto & result_deregistration = deregister_node(request->uuid_node);
  response->result_deregistration.result = result_deregistration.result;

  if (result_deregistration.result != ResultDeregistration::SUCCESS) {
    RCLCPP_WARN(
      get_logger(), "Node deregistration failed for %s",
      autoware_utils::to_hex_string(request->uuid_node).c_str());
  }
}

}  // namespace autoware::control_center
