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

#include <rclcpp/rclcpp.hpp>

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <memory>
#include <string>

const char * lock_file_path = "/tmp/autoware_control_center_node.lock";

int open_lock_file()
{
  // Open or create the lock file with read/write permissions
  int fd = open(lock_file_path, O_CREAT | O_RDWR, 0666);

  // Check if the file descriptor is valid
  if (fd == -1) {
    RCLCPP_FATAL(rclcpp::get_logger(""), "Failed to open lock file");
  }
  return fd;
}

bool lock_file(int fd)
{
  struct flock fl = {F_WRLCK, SEEK_SET, 0, 0, 0};
  // F_WRLCK: Write lock
  // SEEK_SET: Base the lock offset from the beginning of the file
  // 0, 0: Lock the entire file (start offset, length)

  // Attempt to set the file lock using fcntl
  if (fcntl(fd, F_SETLK, &fl) == -1) {
    // Check if the file is already locked by another process
    if (errno == EWOULDBLOCK) {
      RCLCPP_FATAL(rclcpp::get_logger(""), "Another instance is already running");
    } else {
      // Handle other locking errors
      RCLCPP_FATAL(rclcpp::get_logger(""), "Failed to lock file");
    }
    // Close the file descriptor on failure
    close(fd);
    return false;
  }
  return true;
}

void unlock_file(int fd)
{
  // Ensure the file descriptor is valid before closing
  if (fd != -1) {
    close(fd);
  }
}

bool node_already_exists(const std::string & node_name)
{
  auto temp_node = rclcpp::Node::make_shared("temp_node");
  auto all_nodes = temp_node->get_node_names();
  return std::find(all_nodes.begin(), all_nodes.end(), node_name) != all_nodes.end();
}

int main(int argc, char * argv[])
{
  bool use_lock_file = true;
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--dont-use-lock-file") {
      use_lock_file = false;
      break;
    }
  }
  rclcpp::init(argc, argv);

  // Lock file to prevent multiple instances on the same machine
  int lock_fd = -1;
  if (use_lock_file) {
    lock_fd = open_lock_file();
    if (lock_fd == -1 || !lock_file(lock_fd)) {
      return 1;
    }
  }

  // Check if node already exists to prevent multiple instances across the network
  // It's a bit slow but better than nothing
  const std::string node_name_with_namespace = "/autoware/control_center";
  if (node_already_exists(node_name_with_namespace)) {
    RCLCPP_FATAL(
      rclcpp::get_logger(""), "Node %s already exists", node_name_with_namespace.c_str());
    if (use_lock_file) {
      unlock_file(lock_fd);
    }
    return 2;
  }

  // Instantiate the control center node, hopefully the only instance
  auto control_center =
    std::make_shared<autoware::control_center::ControlCenter>(rclcpp::NodeOptions());
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(control_center->get_node_base_interface());
  exec.spin();

  if (use_lock_file) {
    unlock_file(lock_fd);
  }
  rclcpp::shutdown();
  return 0;
}
