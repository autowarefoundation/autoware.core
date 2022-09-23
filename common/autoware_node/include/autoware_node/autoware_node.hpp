#ifndef AUTOWARE_NODE__AUTOWARE_NODE_HPP_
#define AUTOWARE_NODE__AUTOWARE_NODE_HPP_

#include "autoware_node/visibility_control.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace autoware_node
{

class AutowareNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  AutowareNode(
    const std::string & nodeName, const rclcpp::NodeOptions & options,
    bool enable_communication_interface = true);

  virtual ~AutowareNode();
};

}  // namespace autoware_node

#endif  // AUTOWARE_NODE__AUTOWARE_NODE_HPP_
