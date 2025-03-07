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

#include "autoware/trajectory/trajectory_point.hpp"

#include "autoware/trajectory/detail/helpers.hpp"
#include "autoware/trajectory/detail/interpolated_array.hpp"
#include "autoware/trajectory/forward.hpp"
#include "autoware/trajectory/interpolator/stairstep.hpp"
#include "autoware/trajectory/pose.hpp"

#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <memory>
#include <vector>

namespace autoware::trajectory
{

using PointType = autoware_planning_msgs::msg::TrajectoryPoint;

Trajectory<PointType>::Trajectory()
: longitudinal_velocity_mps_(std::make_shared<detail::InterpolatedArray<double>>(
    std::make_shared<interpolator::Stairstep<double>>())),
  lateral_velocity_mps_(std::make_shared<detail::InterpolatedArray<double>>(
    std::make_shared<interpolator::Stairstep<double>>())),
  heading_rate_rps_(std::make_shared<detail::InterpolatedArray<double>>(
    std::make_shared<interpolator::Stairstep<double>>())),
  acceleration_mps2_(std::make_shared<detail::InterpolatedArray<double>>(
    std::make_shared<interpolator::Stairstep<double>>())),
  front_wheel_angle_rad_(std::make_shared<detail::InterpolatedArray<double>>(
    std::make_shared<interpolator::Stairstep<double>>())),
  rear_wheel_angle_rad_(std::make_shared<detail::InterpolatedArray<double>>(
    std::make_shared<interpolator::Stairstep<double>>()))
{
}

Trajectory<PointType>::Trajectory(const Trajectory & rhs)
: BaseClass(rhs),
  longitudinal_velocity_mps_(
    std::make_shared<detail::InterpolatedArray<double>>(*rhs.longitudinal_velocity_mps_)),
  lateral_velocity_mps_(
    std::make_shared<detail::InterpolatedArray<double>>(*rhs.lateral_velocity_mps_)),
  heading_rate_rps_(std::make_shared<detail::InterpolatedArray<double>>(*rhs.heading_rate_rps_)),
  acceleration_mps2_(std::make_shared<detail::InterpolatedArray<double>>(*rhs.acceleration_mps2_)),
  front_wheel_angle_rad_(
    std::make_shared<detail::InterpolatedArray<double>>(*rhs.front_wheel_angle_rad_)),
  rear_wheel_angle_rad_(
    std::make_shared<detail::InterpolatedArray<double>>(*rhs.rear_wheel_angle_rad_))
{
}

Trajectory<PointType> & Trajectory<PointType>::operator=(const Trajectory & rhs)
{
  if (this != &rhs) {
    BaseClass::operator=(rhs);
    *longitudinal_velocity_mps_ = *rhs.longitudinal_velocity_mps_;
    *lateral_velocity_mps_ = *rhs.lateral_velocity_mps_;
    *heading_rate_rps_ = *rhs.heading_rate_rps_;
    *acceleration_mps2_ = *rhs.acceleration_mps2_;
    *front_wheel_angle_rad_ = *rhs.front_wheel_angle_rad_;
    *rear_wheel_angle_rad_ = *rhs.rear_wheel_angle_rad_;
  }
  return *this;
}

bool Trajectory<PointType>::build(const std::vector<PointType> & points)
{
  std::vector<geometry_msgs::msg::Pose> poses;
  std::vector<double> longitudinal_velocity_mps_values;
  std::vector<double> lateral_velocity_mps_values;
  std::vector<double> heading_rate_rps_values;
  std::vector<double> acceleration_mps2_values;
  std::vector<double> front_wheel_angle_rad_values;
  std::vector<double> rear_wheel_angle_rad_values;

  for (const auto & point : points) {
    poses.emplace_back(point.pose);
    longitudinal_velocity_mps_values.emplace_back(point.longitudinal_velocity_mps);
    lateral_velocity_mps_values.emplace_back(point.lateral_velocity_mps);
    heading_rate_rps_values.emplace_back(point.heading_rate_rps);
    acceleration_mps2_values.emplace_back(point.acceleration_mps2);
    front_wheel_angle_rad_values.emplace_back(point.front_wheel_angle_rad);
    rear_wheel_angle_rad_values.emplace_back(point.rear_wheel_angle_rad);
  }

  bool is_valid = true;

  is_valid &= Trajectory<geometry_msgs::msg::Pose>::build(poses);
  is_valid &= this->longitudinal_velocity_mps().build(bases_, longitudinal_velocity_mps_values);
  is_valid &= this->lateral_velocity_mps().build(bases_, lateral_velocity_mps_values);
  is_valid &= this->heading_rate_rps().build(bases_, heading_rate_rps_values);
  is_valid &= this->acceleration_mps2().build(bases_, acceleration_mps2_values);
  is_valid &= this->front_wheel_angle_rad().build(bases_, front_wheel_angle_rad_values);
  is_valid &= this->rear_wheel_angle_rad().build(bases_, rear_wheel_angle_rad_values);

  return is_valid;
}

std::vector<double> Trajectory<PointType>::get_internal_bases() const
{
  auto get_bases = [](const auto & interpolated_array) {
    auto [bases, values] = interpolated_array.get_data();
    return bases;
  };

  auto bases = detail::merge_vectors(
    bases_, get_bases(this->longitudinal_velocity_mps()), get_bases(this->lateral_velocity_mps()),
    get_bases(this->heading_rate_rps()), get_bases(this->acceleration_mps2()),
    get_bases(this->front_wheel_angle_rad()), get_bases(this->rear_wheel_angle_rad()));

  bases = detail::crop_bases(bases, start_, end_);
  std::transform(
    bases.begin(), bases.end(), bases.begin(), [this](const double & s) { return s - start_; });
  return bases;
}

PointType Trajectory<PointType>::compute(double s) const
{
  PointType result;
  result.pose = Trajectory<geometry_msgs::msg::Pose>::compute(s);
  s = clamp(s);
  result.longitudinal_velocity_mps =
    static_cast<float>(this->longitudinal_velocity_mps().compute(s));
  result.lateral_velocity_mps = static_cast<float>(this->lateral_velocity_mps().compute(s));
  result.heading_rate_rps = static_cast<float>(this->heading_rate_rps().compute(s));
  result.acceleration_mps2 = static_cast<float>(this->acceleration_mps2().compute(s));
  result.front_wheel_angle_rad = static_cast<float>(this->front_wheel_angle_rad().compute(s));
  result.rear_wheel_angle_rad = static_cast<float>(this->rear_wheel_angle_rad().compute(s));
  return result;
}

std::vector<PointType> Trajectory<PointType>::restore(const size_t & min_points) const
{
  std::vector<double> bases = get_internal_bases();
  bases = detail::fill_bases(bases, min_points);

  std::vector<PointType> points;
  points.reserve(bases.size());
  for (const auto & s : bases) {
    points.emplace_back(compute(s));
  }
  return points;
}

}  // namespace autoware::trajectory
