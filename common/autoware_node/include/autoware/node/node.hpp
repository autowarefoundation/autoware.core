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

#ifndef AUTOWARE__NODE__NODE_HPP_
#define AUTOWARE__NODE__NODE_HPP_

#include "autoware/node/visibility_control.hpp"

#include <rclcpp/message_memory_strategy.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/subscription_options.hpp>
#include <rclcpp/subscription_traits.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <autoware_control_center_msgs/msg/heartbeat.hpp>
#include <autoware_control_center_msgs/msg/node_status_activity.hpp>
#include <autoware_control_center_msgs/msg/node_status_operational.hpp>
#include <autoware_control_center_msgs/srv/deregister.hpp>
#include <autoware_control_center_msgs/srv/register.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <memory>
#include <string>
#include <utility>

namespace autoware::node
{
class Node : public rclcpp_lifecycle::LifecycleNode
{
public:
  AUTOWARE_NODE_PUBLIC
  explicit Node(
    const std::string & node_name, const std::string & ns = "",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  //  /// @brief Create a subscription with a monitored deadline and liveliness
  //  template <
  //    typename MessageT, typename CallbackT, typename AllocatorT = std::allocator<void>,
  //    typename SubscriptionT = rclcpp::Subscription<MessageT, AllocatorT>,
  //    typename MessageMemoryStrategyT = typename SubscriptionT::MessageMemoryStrategyType>
  //  std::shared_ptr<SubscriptionT> create_monitored_subscription(
  //    const std::string & topic_name, const float hz, const rclcpp::QoS & qos, CallbackT &&
  //    callback, const rclcpp::SubscriptionOptions & options = rclcpp::SubscriptionOptions(),
  //    typename MessageMemoryStrategyT::SharedPtr msg_mem_strategy =
  //      (MessageMemoryStrategyT::create_default()))
  //  {
  //    // create proper qos based on input parameter
  //    // update lease duration and deadline in qos
  //    RCLCPP_DEBUG(get_logger(), "Create monitored subscription to topic %s", topic_name.c_str());
  //    std::chrono::milliseconds lease_duration{
  //      static_cast<int>(1.0 / hz * 1000 * 1.1)};  // add 10 % gap to lease duration (buffer)
  //    rclcpp::QoS qos_profile = qos;
  //    qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC)
  //      .liveliness_lease_duration(lease_duration)
  //      .deadline(lease_duration);
  //
  //    rclcpp::SubscriptionOptions sub_options = options;
  //    sub_options.event_callbacks.deadline_callback =
  //      [=](rclcpp::QOSDeadlineRequestedInfo & event) -> void {
  //      RCLCPP_ERROR(
  //        get_logger(), "Requested deadline missed - total %d delta %d", event.total_count,
  //        event.total_count_change);
  //      // NodeError service call
  //      std::string msg = "Deadline for topic " + topic_name + " was missed.";
  //      autoware_control_center_msgs::msg::NodeState node_state;
  //      node_state.status = autoware_control_center_msgs::msg::NodeState::ERROR;
  //      send_state(node_state, msg);
  //    };
  //
  //    sub_options.event_callbacks.liveliness_callback =
  //      [=](rclcpp::QOSLivelinessChangedInfo & event) -> void {
  //      RCLCPP_DEBUG(get_logger(), "%s topic liveliness info changed", topic_name.c_str());
  //      RCLCPP_DEBUG(get_logger(), "  alive_count: %d", event.alive_count);
  //      RCLCPP_DEBUG(get_logger(), "  not_alive_count: %d", event.not_alive_count);
  //      RCLCPP_DEBUG(get_logger(), "  alive_count_change: %d", event.alive_count_change);
  //      RCLCPP_DEBUG(get_logger(), "  not_alive_count_change: %d", event.not_alive_count_change);
  //      if (event.alive_count == 0) {
  //        RCLCPP_ERROR(get_logger(), "%s topic publisher is not alive.", topic_name.c_str());
  //        // NodeError service call
  //        std::string msg = topic_name + " topic publisher is not alive.";
  //        autoware_control_center_msgs::msg::NodeState node_state;
  //        node_state.status = autoware_control_center_msgs::msg::NodeState::ERROR;
  //        send_state(node_state, msg);
  //      }
  //    };
  //
  //    return create_subscription<MessageT>(
  //      topic_name, qos_profile, std::forward<CallbackT>(callback), sub_options,
  //      msg_mem_strategy);
  //  }

private:
  using Heartbeat = autoware_control_center_msgs::msg::Heartbeat;
  using NodeStatusActivity = autoware_control_center_msgs::msg::NodeStatusActivity;
  using NodeStatusOperational = autoware_control_center_msgs::msg::NodeStatusOperational;
  using ResultDeregistration = autoware_control_center_msgs::msg::ResultDeregistration;
  using ResultRegistration = autoware_control_center_msgs::msg::ResultRegistration;
  using Deregister = autoware_control_center_msgs::srv::Deregister;
  using Register = autoware_control_center_msgs::srv::Register;

  using FutureRegister = rclcpp::Client<autoware_control_center_msgs::srv::Register>::SharedFuture;

  //  rclcpp::CallbackGroup::SharedPtr callback_group_mut_ex_;

  rclcpp::Client<autoware_control_center_msgs::srv::Register>::SharedPtr cli_register_;

  //  rclcpp::Publisher<autoware_control_center_msgs::msg::Heartbeat>::SharedPtr pub_heartbeat_;

  rclcpp::TimerBase::SharedPtr timer_registration_;
  //  rclcpp::TimerBase::SharedPtr timer_heartbeat_;

  const std::string full_name_;
  bool is_registered_;
  unique_identifier_msgs::msg::UUID uuid_node_;
  //  uint16_t sequence_number_;
  //  unique_identifier_msgs::msg::UUID self_uuid;
  //  unique_identifier_msgs::msg::UUID acc_uuid;

  void on_tick_registration();
  //  void on_tick_heartbeat();
  void on_register(FutureRegister future);
};

}  // namespace autoware::node

#endif  // AUTOWARE__NODE__NODE_HPP_
