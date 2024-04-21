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

#include "autoware_control_center/autoware_control_center.hpp"

#include "autoware_control_center/node_registry.hpp"

#include <autoware_utils/ros/uuid_helper.hpp>

#include "autoware_control_center_msgs/msg/autoware_node_report.hpp"
#include "autoware_control_center_msgs/msg/heartbeat.hpp"
#include "autoware_control_center_msgs/srv/autoware_control_center_deregister.hpp"
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <chrono>

namespace autoware_control_center
{

AutowareControlCenter::AutowareControlCenter(const rclcpp::NodeOptions & options)
: LifecycleNode("autoware_control_center", options), lease_duration_{0}
{
  // log info
  RCLCPP_DEBUG(get_logger(), "AutowareControlCenter is initialized");
  declare_parameter<int>("lease_duration", 220);  // TODO(lexavtanke): remove default and add schema
  declare_parameter<int>("node_report_period", 1000);
  lease_duration_ = std::chrono::milliseconds(get_parameter("lease_duration").as_int());
  std::chrono::milliseconds node_report_period(get_parameter("node_report_period").as_int());

  callback_group_mut_ex_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  using std::placeholders::_1;
  using std::placeholders::_2;
  srv_register_ = create_service<autoware_control_center_msgs::srv::AutowareNodeRegister>(
    "~/srv/autoware_node_register", std::bind(&AutowareControlCenter::register_node, this, _1, _2),
    rmw_qos_profile_services_default, callback_group_mut_ex_);
  srv_deregister_ = create_service<autoware_control_center_msgs::srv::AutowareNodeDeregister>(
    "~/srv/autoware_node_deregister",
    std::bind(&AutowareControlCenter::deregister_node, this, _1, _2),
    rmw_qos_profile_services_default, callback_group_mut_ex_);
  srv_node_error_ = create_service<autoware_control_center_msgs::srv::AutowareNodeError>(
    "~/srv/autoware_node_error",
    std::bind(&AutowareControlCenter::autoware_node_error, this, _1, _2),
    rmw_qos_profile_services_default, callback_group_mut_ex_);

  node_reports_pub_ = create_publisher<autoware_control_center_msgs::msg::AutowareNodeReports>(
    "~/autoware_node_reports", 1);
  node_reports_timer_ = create_wall_timer(
    node_report_period, std::bind(&AutowareControlCenter::node_reports_callback, this));

  acc_uuid_ = autoware_utils::generate_uuid();
  on_startup();
}

void AutowareControlCenter::register_node(
  const autoware_control_center_msgs::srv::AutowareNodeRegister::Request::SharedPtr request,
  const autoware_control_center_msgs::srv::AutowareNodeRegister::Response::SharedPtr response)
{
  RCLCPP_DEBUG(get_logger(), "register_node is called from %s", request->name_node.c_str());

  std::optional<unique_identifier_msgs::msg::UUID> node_uuid =
    node_registry_.register_node(request->name_node, autoware_utils::generate_uuid());

  if (node_uuid == std::nullopt) {
    response->uuid_node = autoware_utils::generate_default_uuid();
    response->status.status =
      autoware_control_center_msgs::srv::AutowareNodeRegister::Response::_status_type::FAILURE;
  } else {
    autoware_control_center_msgs::msg::AutowareNodeState un_state;
    un_state.status = autoware_control_center_msgs::msg::AutowareNodeState::UNKNOWN;
    // alive, last_heartbeat, node_report, state
    node_status_map_.insert({request->name_node, {false, this->now(), "", un_state}});
    // Create heartbeat sub
    rclcpp::Subscription<autoware_control_center_msgs::msg::Heartbeat>::SharedPtr
      heartbeat_node_sub = create_heartbeat_sub(request->name_node);
    heartbeat_sub_map_.insert({request->name_node, heartbeat_node_sub});
    RCLCPP_DEBUG(get_logger(), "Subscribed to topic %s", heartbeat_node_sub->get_topic_name());
    response->uuid_node = node_uuid.value();
    response->status.status =
      autoware_control_center_msgs::srv::AutowareNodeRegister::Response::_status_type::SUCCESS;
  }
}

void AutowareControlCenter::deregister_node(
  const autoware_control_center_msgs::srv::AutowareNodeDeregister::Request::SharedPtr request,
  const autoware_control_center_msgs::srv::AutowareNodeDeregister::Response::SharedPtr response)
{
  RCLCPP_DEBUG(get_logger(), "deregister_node is called from %s", request->name_node.c_str());

  std::optional<unique_identifier_msgs::msg::UUID> node_uuid =
    node_registry_.deregister_node(request->name_node);

  if (node_uuid == std::nullopt) {
    response->uuid_node = autoware_utils::generate_default_uuid();
    response->status.status =
      autoware_control_center_msgs::srv::AutowareNodeDeregister::Response::_status_type::FAILURE;
  } else {
    response->uuid_node = node_uuid.value();
    response->status.status =
      autoware_control_center_msgs::srv::AutowareNodeDeregister::Response::_status_type::SUCCESS;
  }
}

void AutowareControlCenter::on_startup()
{
  RCLCPP_DEBUG(get_logger(), "ACC startup called");
  std::map<std::string, std::vector<std::string>> srv_list = this->get_service_names_and_types();
  filter_deregister_services(srv_list);

  RCLCPP_DEBUG(get_logger(), "Filtered service list:");
  if (srv_list.empty()) {
    RCLCPP_DEBUG(get_logger(), "Empty.");
  }
  for (auto const & pair : srv_list) {
    RCLCPP_DEBUG(get_logger(), "Service Name: %s", pair.first.c_str());
    rclcpp::Client<autoware_control_center_msgs::srv::AutowareControlCenterDeregister>::SharedPtr
      // cspell:ignore dereg
      dereg_client =
        create_client<autoware_control_center_msgs::srv::AutowareControlCenterDeregister>(
          pair.first);
    autoware_control_center_msgs::srv::AutowareControlCenterDeregister::Request::SharedPtr req =
      std::make_shared<
        autoware_control_center_msgs::srv::AutowareControlCenterDeregister::Request>();

    req->uuid_acc = acc_uuid_;

    using ServiceResponseFuture = rclcpp::Client<
      autoware_control_center_msgs::srv::AutowareControlCenterDeregister>::SharedFuture;
    // lambda for async request
    // NOLINTNEXTLINE(performance-unnecessary-value-param)
    auto response_received_callback = [this](const ServiceResponseFuture future) {
      const auto & response = future.get();
      RCLCPP_DEBUG(
        get_logger(), "Deregister response: %d, %s", response->status.status,
        response->name_node.c_str());

      if (response->status.status == autoware_control_center_msgs::msg::Status::SUCCESS) {
        RCLCPP_DEBUG(get_logger(), "Node was deregistered");
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to deregister node");
      }
    };

    if (dereg_client->service_is_ready()) {
      auto future_result = dereg_client->async_send_request(req, response_received_callback);
      RCLCPP_DEBUG(get_logger(), "Sent request to %s", pair.first.c_str());
    }
  }  
}

rclcpp::Subscription<autoware_control_center_msgs::msg::Heartbeat>::SharedPtr
AutowareControlCenter::create_heartbeat_sub(const std::string & node_name)
{
  RCLCPP_DEBUG(get_logger(), "Create heart sub is called.");
  auto qos_profile = rclcpp::QoS(10);
  qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC)
    .liveliness_lease_duration(lease_duration_);

  rclcpp::SubscriptionOptions heartbeat_sub_options;
  std::function<void(rclcpp::QOSLivelinessChangedInfo & event)> bound_liveliness_callback_func =
    std::bind(&AutowareControlCenter::liveliness_callback, this, std::placeholders::_1, node_name);
  heartbeat_sub_options.event_callbacks.liveliness_callback = bound_liveliness_callback_func;

  const std::string topic_name = node_name + "/heartbeat";
  RCLCPP_DEBUG(get_logger(), "Topic to subscribe is %s", topic_name.c_str());
  rclcpp::Subscription<autoware_control_center_msgs::msg::Heartbeat>::SharedPtr heartbeat_sub;
  std::function<void(const typename autoware_control_center_msgs::msg::Heartbeat::SharedPtr msg)>
    bound_heartbeat_callback_func =
      std::bind(&AutowareControlCenter::heartbeat_callback, this, std::placeholders::_1, node_name);
  heartbeat_sub = create_subscription<autoware_control_center_msgs::msg::Heartbeat>(
    topic_name, qos_profile, bound_heartbeat_callback_func, heartbeat_sub_options);

  return heartbeat_sub;
}

void AutowareControlCenter::node_reports_callback()
{
  autoware_control_center_msgs::msg::AutowareNodeReports msg;
  const rclcpp::Time stamp = this->now();
  msg.stamp = stamp;
  for (auto const & [name, info] : node_status_map_) {
    autoware_control_center_msgs::msg::AutowareNodeReport report;
    report.uuid_node = node_registry_.get_uuid(name).value();
    report.name_node = name;
    report.alive = info.alive;
    report.last_node_state = info.state;
    report.node_report = info.node_report;
    report.last_heartbeat = stamp - info.last_heartbeat;
    msg.nodes.push_back(report);
  }
  node_reports_pub_->publish(msg);
}

void AutowareControlCenter::autoware_node_error(
  const autoware_control_center_msgs::srv::AutowareNodeError::Request::SharedPtr request,
  const autoware_control_center_msgs::srv::AutowareNodeError::Response::SharedPtr response)
{
  RCLCPP_DEBUG(get_logger(), "autoware_node_error is called from %s", request->name_node.c_str());
  // node_report
  if (node_registry_.is_registered(request->name_node)) {
    node_status_map_[request->name_node].node_report = request->message;
    node_status_map_[request->name_node].state = request->state;

    response->status.status =
      autoware_control_center_msgs::srv::AutowareNodeError::Response::_status_type::SUCCESS;
    response->uuid_node = node_registry_.get_uuid(request->name_node).value();
  } else {
    response->status.status =
      autoware_control_center_msgs::srv::AutowareNodeError::Response::_status_type::FAILURE;
    response->uuid_node = autoware_utils::generate_default_uuid();
    response->log_response = request->name_node + " node was not registered.";
  }
}

void AutowareControlCenter::liveliness_callback(
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

void AutowareControlCenter::heartbeat_callback(
  const typename autoware_control_center_msgs::msg::Heartbeat::SharedPtr msg,
  const std::string & node_name)
{
  RCLCPP_DEBUG(get_logger(), "Watchdog raised, heartbeat sent at [%d.x]", msg->stamp.sec);
  node_status_map_[node_name].alive = true;
  node_status_map_[node_name].last_heartbeat = msg->stamp;
}

void AutowareControlCenter::filter_deregister_services(
  std::map<std::string, std::vector<std::string>> & srv_list)
{
  auto it = srv_list.begin();
  // filter out srv with type autoware_control_center_msgs/srv/AutowareControlCenterDeregister
  while (it != srv_list.end()) {
    if (it->second[0] != "autoware_control_center_msgs/srv/AutowareControlCenterDeregister") {
      srv_list.erase(it++);
    } else {
      ++it;
    }
  }
}

}  // namespace autoware_control_center
