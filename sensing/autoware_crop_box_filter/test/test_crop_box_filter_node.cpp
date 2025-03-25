// Copyright(c) 2025 AutoCore Technology (Nanjing) Co., Ltd. All rights reserved.
//
// Copyright 2025 TIER IV, Inc.
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

#include "autoware/crop_box_filter/crop_box_filter_node.hpp"

#include <gtest/gtest.h>

#include <memory>

TEST(CropBoxFilterTest, checkOutputPointcloud)
{
  rclcpp::init(0, nullptr);

  // prepare parameters for the node
  const double min_x = -5.0;
  const double max_x = 5.0;
  const double min_y = -5.0;
  const double max_y = 5.0;
  const double min_z = -5.0;
  const double max_z = 5.0;
  const int64_t max_queue_size = 5;
  const bool negative = true;

  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides({
    {"min_x", min_x},
    {"min_y", min_y},
    {"min_z", min_z},
    {"max_x", max_x},
    {"max_y", max_y},
    {"max_z", max_z},
    {"negative", negative},
    {"input_pointcloud_frame", "base_link"},
    {"input_frame", "base_link"},
    {"output_frame", "base_link"},
    {"max_queue_size", max_queue_size},
  });

  // Create the node with the specified options
  autoware::crop_box_filter::CropBoxFilter node(node_options);

  // construct input pointcloud
  pcl::PointCloud<pcl::PointXYZ> input_pointcloud;
  input_pointcloud.push_back(pcl::PointXYZ(0.5, 0.5, 0.1));  // point inside the polygon
  input_pointcloud.push_back(pcl::PointXYZ(1.5, 1.5, 1.1));  // point inside the polygon
  input_pointcloud.push_back(pcl::PointXYZ(2.5, 2.5, 2.1));  // point inside the polygon
  input_pointcloud.push_back(pcl::PointXYZ(3.5, 3.5, 3.1));  // point inside the polygon
  input_pointcloud.push_back(pcl::PointXYZ(4.5, 4.5, 4.1));  // point inside the polygon

  input_pointcloud.push_back(pcl::PointXYZ(5.5, 5.5, 5.1));  // point outside the polygon
  input_pointcloud.push_back(pcl::PointXYZ(6.5, 6.5, 6.1));  // point outside the polygon
  input_pointcloud.push_back(pcl::PointXYZ(7.5, 7.5, 7.1));  // point outside the polygon
  input_pointcloud.push_back(pcl::PointXYZ(8.5, 8.5, 8.1));  // point outside the polygon
  input_pointcloud.push_back(pcl::PointXYZ(9.5, 9.5, 9.1));  // point outside the polygon

  input_pointcloud.push_back(pcl::PointXYZ(-5.5, -5.5, -5.1));  // point outside the polygon
  input_pointcloud.push_back(pcl::PointXYZ(-6.5, -6.5, -6.1));  // point outside the polygon
  input_pointcloud.push_back(pcl::PointXYZ(-7.5, -7.5, -7.1));  // point outside the polygon
  input_pointcloud.push_back(pcl::PointXYZ(-8.5, -8.5, -8.1));  // point outside the polygon
  input_pointcloud.push_back(pcl::PointXYZ(-9.5, -9.5, -9.1));  // point outside the polygon

  // convert the point cloud to a ROS message
  sensor_msgs::msg::PointCloud2 pointcloud;
  pcl::toROSMsg(input_pointcloud, pointcloud);
  pointcloud.header.frame_id = "base_link";

  const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_msg =
    std::make_shared<sensor_msgs::msg::PointCloud2>(pointcloud);

  auto output = sensor_msgs::msg::PointCloud2();
  // filtering
  node.filter_pointcloud(pointcloud_msg, output);

  // check the size of the output point cloud
  EXPECT_EQ((output.data.size() / output.point_step), 10);

  // check every point in the output pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(output, *cloud);

  // check if the points are inside the polygon
  for (const auto & point : cloud->points) {
    bool point_is_inside = point.z > min_z && point.z < max_z && point.y > min_y &&
                           point.y < max_y && point.x > min_x && point.x < max_x;
    EXPECT_TRUE((!negative && point_is_inside) || (negative && !point_is_inside));
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
