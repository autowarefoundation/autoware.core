// Copyright 2024 TIER IV, Inc.
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

#include "autoware/trajectory/pose.hpp"

#include "autoware/trajectory/detail/helpers.hpp"
#include "autoware/trajectory/forward.hpp"
#include "autoware/trajectory/interpolator/spherical_linear.hpp"

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.h>

#include <memory>
#include <utility>
#include <vector>
namespace autoware::trajectory
{
using PointType = geometry_msgs::msg::Pose;

Trajectory<PointType>::Trajectory()
{
  Builder::defaults(this);
}

Trajectory<PointType>::Trajectory(const Trajectory & rhs)
: BaseClass(rhs), orientation_interpolator_(rhs.orientation_interpolator_->clone())
{
}

Trajectory<PointType> & Trajectory<PointType>::operator=(const Trajectory & rhs)
{
  if (this != &rhs) {
    BaseClass::operator=(rhs);
    orientation_interpolator_ = rhs.orientation_interpolator_->clone();
  }
  return *this;
}

interpolator::InterpolationResult Trajectory<PointType>::build(
  const std::vector<PointType> & points)
{
  std::vector<geometry_msgs::msg::Point> path_points;
  std::vector<geometry_msgs::msg::Quaternion> orientations;
  path_points.reserve(points.size());
  orientations.reserve(points.size());
  for (const auto & point : points) {
    path_points.emplace_back(point.position);
    orientations.emplace_back(point.orientation);
  }

  if (const auto result = BaseClass::build(path_points); !result) {
    return tl::unexpected(
      interpolator::InterpolationFailure{"failed to interpolate Pose::points"} + result.error());
  }
  if (const auto result = orientation_interpolator_->build(bases_, std::move(orientations));
      !result) {
    return tl::unexpected(
      interpolator::InterpolationFailure{"failed to interpolate Pose::orientation"} +
      result.error());
  }
  return interpolator::InterpolationSuccess{};
}

std::vector<double> Trajectory<PointType>::get_underlying_bases() const
{
  auto bases = detail::crop_bases(bases_, start_, end_);
  std::transform(
    bases.begin(), bases.end(), bases.begin(), [this](const double s) { return s - start_; });
  return bases;
}

PointType Trajectory<PointType>::compute(const double s) const
{
  PointType result;
  result.position = BaseClass::compute(s);
  const auto s_clamp = clamp(s);
  // NOTE(soblin): Ideally azimuth() should be used? But okay if the interpolation is not rough
  result.orientation = orientation_interpolator_->compute(s_clamp);
  return result;
}

std::vector<PointType> Trajectory<PointType>::compute(const std::vector<double> & ss) const
{
  std::vector<PointType> points;
  points.reserve(ss.size());
  for (const auto s : ss) {
    points.emplace_back(compute(s));
  }
  return points;
}

void Trajectory<PointType>::align_orientation_with_trajectory_direction()
{
  std::vector<geometry_msgs::msg::Quaternion> aligned_orientations;
  for (const auto & s : bases_) {
    const double azimuth = this->azimuth(s);
    const double elevation = this->elevation(s);
    const geometry_msgs::msg::Quaternion current_orientation =
      orientation_interpolator_->compute(s);
    tf2::Quaternion current_orientation_tf2(
      current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w);
    current_orientation_tf2.normalize();

    // Calculate the current x-axis of the orientation (local x-axis)
    const tf2::Vector3 current_x_axis =
      tf2::quatRotate(current_orientation_tf2, tf2::Vector3(1, 0, 0));

    // Calculate the desired x-axis direction based on the trajectory's azimuth and elevation
    const tf2::Vector3 desired_x_axis(
      std::cos(elevation) * std::cos(azimuth), std::cos(elevation) * std::sin(azimuth),
      std::sin(elevation));

    const double dot_product = current_x_axis.dot(desired_x_axis);
    const double rotation_angle = std::acos(dot_product);

    const tf2::Vector3 rotation_axis = [&]() {
      // If the rotation angle is nearly 0 or 180 degrees, choose a default rotation axis
      if (std::abs(rotation_angle) < 1e-6 || std::abs(rotation_angle - M_PI) < 1e-6) {
        // Use the rotated z-axis as the fallback axis
        return tf2::quatRotate(current_orientation_tf2, tf2::Vector3(0, 0, 1));
      }
      // Otherwise, compute the rotation axis using the cross product of the current and desired
      // x-axes
      tf2::Vector3 cross = current_x_axis.cross(desired_x_axis);
      return cross.normalized();
    }();

    // Create a quaternion representing the rotation required to align the x-axis
    tf2::Quaternion delta_q = tf2::Quaternion(rotation_axis, rotation_angle);

    // Apply the rotation delta to the current orientation and normalize the result
    const tf2::Quaternion aligned_orientation_tf2 =
      (delta_q * current_orientation_tf2).normalized();

    geometry_msgs::msg::Quaternion aligned_orientation;
    aligned_orientation.x = aligned_orientation_tf2.x();
    aligned_orientation.y = aligned_orientation_tf2.y();
    aligned_orientation.z = aligned_orientation_tf2.z();
    aligned_orientation.w = aligned_orientation_tf2.w();

    aligned_orientations.emplace_back(aligned_orientation);
  }
  const auto success = orientation_interpolator_->build(bases_, std::move(aligned_orientations));
  if (!success) {
    throw std::runtime_error(
      "Failed to build orientation interpolator.");  // This exception should not be thrown.
  }
}

std::vector<PointType> Trajectory<PointType>::restore(const size_t min_points) const
{
  auto bases = get_underlying_bases();
  bases = detail::fill_bases(bases, min_points);
  std::vector<PointType> points;
  points.reserve(bases.size());
  for (const auto & s : bases) {
    points.emplace_back(compute(s));
  }
  return points;
}

Trajectory<PointType>::Builder::Builder() : trajectory_(std::make_unique<Trajectory<PointType>>())
{
  defaults(trajectory_.get());
}

void Trajectory<PointType>::Builder::defaults(Trajectory<PointType> * trajectory)
{
  BaseClass::Builder::defaults(trajectory);
  trajectory->orientation_interpolator_ = std::make_shared<interpolator::SphericalLinear>();
}

tl::expected<Trajectory<PointType>, interpolator::InterpolationFailure>
Trajectory<PointType>::Builder::build(const std::vector<PointType> & points)
{
  auto trajectory_result = trajectory_->build(points);
  if (trajectory_result) {
    auto result = Trajectory(std::move(*trajectory_));
    trajectory_.reset();
    return result;
  }
  return tl::unexpected(trajectory_result.error());
}

}  // namespace autoware::trajectory
