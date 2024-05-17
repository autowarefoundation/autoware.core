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

#include <rclcpp/rclcpp.hpp>

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <memory>

const char * LOCK_FILE = "/tmp/autoware_control_center_node.lock";

bool lock_file()
{
  int fd = open(LOCK_FILE, O_CREAT | O_RDWR, 0666);
  if (fd == -1) {
    RCLCPP_FATAL(rclcpp::get_logger(""), "Failed to open lock file");
    return false;
  }

  if (flock(fd, LOCK_EX | LOCK_NB) == -1) {
    if (errno == EWOULDBLOCK) {
      RCLCPP_FATAL(rclcpp::get_logger(""), "Another instance is already running");
    } else {
      RCLCPP_FATAL(rclcpp::get_logger(""), "Failed to lock file");
    }
    close(fd);
    return false;
  }

  // Keep the file descriptor open to maintain the lock
  return true;
}

void unlock_file(int fd)
{
  if (fd != -1) {
    flock(fd, LOCK_UN);
    close(fd);
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Lock file to prevent multiple instances on the same machine
  int lock_fd = open(LOCK_FILE, O_CREAT | O_RDWR, 0666);
  if (!lock_file()) {
    return 1;
  }

  // Check if node already exists to prevent multiple instances across the network
  // It's a bit slow but better than nothing
  const auto node_name_with_namespace = std::string("/autoware/control_center");

  auto node_already_exists = [](const std::string & node_name) {
    auto temp_node = rclcpp::Node::make_shared("temp_node");
    auto all_nodes = temp_node->get_node_names();
    return std::find(all_nodes.begin(), all_nodes.end(), node_name) != all_nodes.end();
  };

  if (node_already_exists(node_name_with_namespace)) {
    RCLCPP_FATAL(
      rclcpp::get_logger(""), "Node %s already exists", node_name_with_namespace.c_str());
    unlock_file(lock_fd);
    throw std::runtime_error("Node already exists");
  }

  // Instantiate the control center node, hopefully the only instance
  auto control_center =
    std::make_shared<autoware::control_center::ControlCenter>(rclcpp::NodeOptions());
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(control_center->get_node_base_interface());
  exec.spin();
  unlock_file(lock_fd);
  rclcpp::shutdown();
  return 0;
}
