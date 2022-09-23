#include "autoware_node/autoware_node.hpp"

namespace autoware_node
{

AutowareNode::~AutowareNode() {}

AutowareNode::AutowareNode(
  const std::string & nodeName, const rclcpp::NodeOptions & options,
  bool enableCommunicationInterface)
: LifecycleNode(nodeName, options, enableCommunicationInterface)
{
}

}  // namespace autoware_node
