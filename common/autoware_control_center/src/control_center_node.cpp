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

#include "include/control_center_node.hpp"

#include "include/node_registry.hpp"

#include <autoware_utils/ros/uuid_helper.hpp>

#include <autoware_control_center_msgs/msg/node_report.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <chrono>

namespace autoware::control_center
{
ControlCenter::ControlCenter(const rclcpp::NodeOptions & options)
: LifecycleNode("control_center", "autoware", options),
  lease_duration_ms_{declare_parameter<double>("lease_duration_ms")},
  report_publish_rate_{declare_parameter<double>("report_publish_rate")}
{
  RCLCPP_DEBUG(get_logger(), "ControlCenter is initialized");

  timer_publish_reports_ = create_wall_timer(
    std::chrono::duration<double, std::milli>(1.0 / report_publish_rate_),
    std::bind(&ControlCenter::on_tick_publish_reports, this));

  callback_group_mut_ex_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  using std::placeholders::_1;
  using std::placeholders::_2;
  srv_register_ = create_service<Register>(
    "~/srv/register", std::bind(&ControlCenter::on_register_node, this, _1, _2),
    rmw_qos_profile_services_default, callback_group_mut_ex_);
  srv_deregister_ = create_service<Deregister>(
    "~/srv/deregister", std::bind(&ControlCenter::on_deregister_node, this, _1, _2),
    rmw_qos_profile_services_default, callback_group_mut_ex_);
  srv_report_state_ = create_service<ReportState>(
    "~/srv/report_state", std::bind(&ControlCenter::on_report_state, this, _1, _2),
    rmw_qos_profile_services_default, callback_group_mut_ex_);

  pub_reports_ = create_publisher<autoware_control_center_msgs::msg::NodeReports>("~/reports", 1);

  acc_uuid_ = autoware_utils::generate_uuid();
}

void ControlCenter::on_register_node(
  const Register::Request::SharedPtr request, const Register::Response::SharedPtr response)
{
  RCLCPP_DEBUG(get_logger(), "on_register_node is called from %s", request->name_node.c_str());

  std::optional<unique_identifier_msgs::msg::UUID> node_uuid =
    node_registry_.register_node(request->name_node, autoware_utils::generate_uuid());

  if (node_uuid == std::nullopt) {
    response->uuid_node = autoware_utils::generate_default_uuid();
    response->status.status = Register::Response::_status_type::FAILURE;
  } else {
    NodeState un_state;
    un_state.status = NodeState::UNKNOWN;
    // alive, last_heartbeat, node_report, state
    node_status_map_.insert({request->name_node, {false, this->now(), "", un_state}});
    // Create heartbeat sub
    rclcpp::Subscription<autoware_control_center_msgs::msg::Heartbeat>::SharedPtr
      heartbeat_node_sub = create_heartbeat_sub(request->name_node);
    heartbeat_sub_map_.insert({request->name_node, heartbeat_node_sub});
    RCLCPP_DEBUG(get_logger(), "Subscribed to topic %s", heartbeat_node_sub->get_topic_name());
    response->uuid_node = node_uuid.value();
    response->status.status = Register::Response::_status_type::SUCCESS;
  }
}

void ControlCenter::on_deregister_node(
  const Deregister::Request::SharedPtr request, const Deregister::Response::SharedPtr response)
{
  RCLCPP_DEBUG(get_logger(), "on_deregister_node is called from %s", request->name_node.c_str());

  std::optional<unique_identifier_msgs::msg::UUID> node_uuid =
    node_registry_.deregister_node(request->name_node);

  if (node_uuid == std::nullopt) {
    response->uuid_node = autoware_utils::generate_default_uuid();
    response->status.status = Deregister::Response::_status_type::FAILURE;
  } else {
    response->uuid_node = node_uuid.value();
    response->status.status = Deregister::Response::_status_type::SUCCESS;
  }
}

rclcpp::Subscription<ControlCenter::Heartbeat>::SharedPtr ControlCenter::create_heartbeat_sub(
  const std::string & node_name)
{
  RCLCPP_DEBUG(get_logger(), "Create heart sub is called.");
  auto qos_profile = rclcpp::QoS(10);
  qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC)
    .liveliness_lease_duration(lease_duration_ms_);

  rclcpp::SubscriptionOptions heartbeat_sub_options;
  std::function<void(rclcpp::QOSLivelinessChangedInfo & event)> bound_liveliness_callback_func =
    std::bind(&ControlCenter::liveliness_callback, this, std::placeholders::_1, node_name);
  heartbeat_sub_options.event_callbacks.liveliness_callback = bound_liveliness_callback_func;

  const std::string topic_name = node_name + "/heartbeat";
  RCLCPP_DEBUG(get_logger(), "Topic to subscribe is %s", topic_name.c_str());
  rclcpp::Subscription<Heartbeat>::SharedPtr heartbeat_sub;
  std::function<void(const typename Heartbeat::SharedPtr msg)> bound_heartbeat_callback_func =
    std::bind(&ControlCenter::heartbeat_callback, this, std::placeholders::_1, node_name);
  heartbeat_sub = create_subscription<Heartbeat>(
    topic_name, qos_profile, bound_heartbeat_callback_func, heartbeat_sub_options);

  return heartbeat_sub;
}

void ControlCenter::on_tick_publish_reports()
{
  NodeReports msg;
  const rclcpp::Time stamp = this->now();
  msg.stamp = stamp;
  for (auto const & [name, info] : node_status_map_) {
    autoware_control_center_msgs::msg::NodeReport report;
    report.uuid_node = node_registry_.get_uuid(name).value();
    report.name_node = name;
    report.alive = info.alive;
    report.last_node_state = info.state;
    report.node_report = info.node_report;
    report.last_heartbeat = stamp - info.last_heartbeat;
    msg.nodes.push_back(report);
  }
  pub_reports_->publish(msg);
}

void ControlCenter::on_report_state(
  const ReportState::Request::SharedPtr request, const ReportState::Response::SharedPtr response)
{
  RCLCPP_DEBUG(get_logger(), "on_report_state is called from %s", request->name_node.c_str());
  // node_report
  if (node_registry_.is_registered(request->name_node)) {
    node_status_map_[request->name_node].node_report = request->message;
    node_status_map_[request->name_node].state = request->state;

    response->status.status = ReportState::Response::_status_type::SUCCESS;
    response->uuid_node = node_registry_.get_uuid(request->name_node).value();
  } else {
    response->status.status = ReportState::Response::_status_type::FAILURE;
    response->uuid_node = autoware_utils::generate_default_uuid();
    response->log_response = request->name_node + " node was not registered.";
  }
}

void ControlCenter::liveliness_callback(
  rclcpp::QOSLivelinessChangedInfo & event, const std::string & node_name)
{
  RCLCPP_DEBUG(get_logger(), "Reader Liveliness changed event:");
  RCLCPP_DEBUG(get_logger(), "  alive_count: %d", event.alive_count);
  RCLCPP_DEBUG(get_logger(), "  not_alive_count: %d", event.not_alive_count);
  RCLCPP_DEBUG(get_logger(), "  alive_count_change: %d", event.alive_count_change);
  RCLCPP_DEBUG(get_logger(), "  not_alive_count_change: %d", event.not_alive_count_change);
  if (event.alive_count == 0) {
    RCLCPP_ERROR(get_logger(), "Heartbeat was not received");
    node_status_map_[node_name].alive = false;
  }
}

void ControlCenter::heartbeat_callback(
  const Heartbeat::SharedPtr msg, const std::string & node_name)
{
  RCLCPP_DEBUG(get_logger(), "Watchdog raised, heartbeat sent at [%d.x]", msg->stamp.sec);
  node_status_map_[node_name].alive = true;
  node_status_map_[node_name].last_heartbeat = msg->stamp;
}

}  // namespace autoware::control_center

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::control_center::ControlCenter)
