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

#ifndef AUTOWARE_NODE__AUTOWARE_NODE_HPP_
#define AUTOWARE_NODE__AUTOWARE_NODE_HPP_

#include "autoware_node/visibility_control.hpp"
#include "rclcpp/message_memory_strategy.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/subscription_traits.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "autoware_control_center_msgs/msg/autoware_node_state.hpp"
#include "autoware_control_center_msgs/msg/heartbeat.hpp"
#include "autoware_control_center_msgs/srv/autoware_control_center_deregister.hpp"
#include "autoware_control_center_msgs/srv/autoware_node_error.hpp"
#include "autoware_control_center_msgs/srv/autoware_node_register.hpp"

#include <memory>
#include <string>
#include <utility>

namespace autoware_node
{

class AutowareNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  AUTOWARE_NODE_PUBLIC
  explicit AutowareNode(
    const std::string & node_name, const std::string & ns = "",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /*!
  Create subscription to provided topic with topic_name and monitors period of received messages.
  If desired period is violated the Autoware Node informs the Autoware Control Center.
  \param[topic_name] The name of the monitored topic.
  \param[hz] The desired message frequency of the topic.
  \param[qos] The desired QoS for the topic.
  */
  // cspell:ignore strat
  template <
    typename MessageT, typename CallbackT, typename AllocatorT = std::allocator<void>,
    typename SubscriptionT = rclcpp::Subscription<MessageT, AllocatorT>,
    typename MessageMemoryStrategyT = typename SubscriptionT::MessageMemoryStrategyType>
  std::shared_ptr<SubscriptionT> create_monitored_subscription(
    const std::string & topic_name, const float hz, const rclcpp::QoS & qos, CallbackT && callback,
    const rclcpp::SubscriptionOptions & options = rclcpp::SubscriptionOptions(),
    typename MessageMemoryStrategyT::SharedPtr msg_mem_strat =
      (MessageMemoryStrategyT::create_default()))
  {
    // create proper qos based on input parameter
    // update lease duration and deadline in qos
    RCLCPP_INFO(get_logger(), "Create monitored subscription to topic %s", topic_name.c_str());
    std::chrono::milliseconds lease_duration{
      static_cast<int>(1.0 / hz * 1000 * 1.1)};  // add 10 % gap to lease duration (buffer)
    rclcpp::QoS qos_profile = qos;
    qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC)
      .liveliness_lease_duration(lease_duration)
      .deadline(lease_duration);

    rclcpp::SubscriptionOptions sub_options = options;
    sub_options.event_callbacks.deadline_callback =
      [=](rclcpp::QOSDeadlineRequestedInfo & event) -> void {
      RCLCPP_ERROR(
        get_logger(), "Requested deadline missed - total %d delta %d", event.total_count,
        event.total_count_change);
      // NodeError service call
      std::string msg = "Deadline for topic " + topic_name + " was missed.";
      autoware_control_center_msgs::msg::AutowareNodeState node_state;
      node_state.status = autoware_control_center_msgs::msg::AutowareNodeState::ERROR;
      send_state(node_state, msg);
    };

    sub_options.event_callbacks.liveliness_callback =
      [=](rclcpp::QOSLivelinessChangedInfo & event) -> void {
      RCLCPP_INFO(get_logger(), "%s topic liveliness info changed", topic_name.c_str());
      RCLCPP_INFO(get_logger(), "  alive_count: %d", event.alive_count);
      RCLCPP_INFO(get_logger(), "  not_alive_count: %d", event.not_alive_count);
      RCLCPP_INFO(get_logger(), "  alive_count_change: %d", event.alive_count_change);
      RCLCPP_INFO(get_logger(), "  not_alive_count_change: %d", event.not_alive_count_change);
      if (event.alive_count == 0) {
        RCLCPP_ERROR(get_logger(), "%s topic publisher is not alive.", topic_name.c_str());
        // NodeError service call
        std::string msg = topic_name + " topic publisher is not alive.";
        autoware_control_center_msgs::msg::AutowareNodeState node_state;
        node_state.status = autoware_control_center_msgs::msg::AutowareNodeState::ERROR;
        send_state(node_state, msg);
      }
    };

    return create_subscription<MessageT>(
      topic_name, qos_profile, std::forward<CallbackT>(callback), sub_options, msg_mem_strat);
  }

  /*!
  The mutually exclusive callback group for Autoware Node services.
  */
  rclcpp::CallbackGroup::SharedPtr callback_group_mut_ex_;
  /*!
  The client of the Autoware Node register service.
  */
  rclcpp::Client<autoware_control_center_msgs::srv::AutowareNodeRegister>::SharedPtr cli_register_;
  /*!
  The service of the Autoware Control Center de-register type.
  */
  rclcpp::Service<autoware_control_center_msgs::srv::AutowareControlCenterDeregister>::SharedPtr
    srv_deregister_;
  /*!
  The client of the Autoware Node error service.
  */
  rclcpp::Client<autoware_control_center_msgs::srv::AutowareNodeError>::SharedPtr cli_node_error_;
  /*!
  The publisher of the heartbeat topic.
  */
  rclcpp::Publisher<autoware_control_center_msgs::msg::Heartbeat>::SharedPtr heartbeat_pub_;
  /*!
  Controls heartbeat publishing.
  */
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  /*!
  Controls the node register callback.
  */
  rclcpp::TimerBase::SharedPtr register_timer_;
  /*!
  The flag if the node registered to Autoware Control Center or not.
  */
  bool registered;
  /*!
  The UUID of the Autoware Node.
  */
  unique_identifier_msgs::msg::UUID self_uuid;
  /*!
  The UUID of the Autoware Control Center.
  */
  unique_identifier_msgs::msg::UUID acc_uuid;
  /*!
  The name of the Autoware Node.
  */
  std::string self_name;

private:
  /*!
  Manage Autoware Node registering procedure.
  */
  void register_callback();
  /*!
  Publish heartbeat topic of the Autoware Node.
  */
  void heartbeat_callback();
  using AutowareNodeRegisterServiceResponseFuture =
    rclcpp::Client<autoware_control_center_msgs::srv::AutowareNodeRegister>::SharedFuture;
  /*!
  Manage a response from the Autoware Node register service.
  */
  void node_register_future_callback(AutowareNodeRegisterServiceResponseFuture future);
  using AutowareNodeErrorServiceResponseFuture =
    rclcpp::Client<autoware_control_center_msgs::srv::AutowareNodeError>::SharedFuture;
  /*!
  Manage a response from the Autoware Node error service.
  */
  void node_error_future_callback(AutowareNodeErrorServiceResponseFuture future);
  /*!
  Send the Autoware Node state to the Autoware Control Center.
  \param[node_state] The state of the Autoware Node to send to the Autoware Control Center.
  \param[message] The log message to send to the Autoware Control Center.
  */
  void send_state(
    const autoware_control_center_msgs::msg::AutowareNodeState & node_state, std::string message);
  /*!
  The callback for the  Autoware Control Center deregister service of the Autoware Node.
  */
  void deregister(
    const autoware_control_center_msgs::srv::AutowareControlCenterDeregister::Request::SharedPtr
      request,
    const autoware_control_center_msgs::srv::AutowareControlCenterDeregister::Response::SharedPtr
      response);
  /*!
  The sequential number of the Heartbeat message.
  */
  uint16_t sequence_number;
};

}  // namespace autoware_node

#endif  // AUTOWARE_NODE__AUTOWARE_NODE_HPP_
