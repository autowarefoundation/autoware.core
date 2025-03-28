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

#ifndef AUTOWARE__TRAJECTORY__POINT_HPP_
#define AUTOWARE__TRAJECTORY__POINT_HPP_

#include "autoware/trajectory/forward.hpp"
#include "autoware/trajectory/interpolator/interpolator.hpp"

#include <Eigen/Dense>

#include <geometry_msgs/msg/point.hpp>

#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::trajectory
{
/**
 * @brief Trajectory class for geometry_msgs::msg::Point
 */
template <>
class Trajectory<geometry_msgs::msg::Point>
{
  using PointType = geometry_msgs::msg::Point;

protected:
  std::shared_ptr<interpolator::InterpolatorInterface<double>> x_interpolator_{
    nullptr};  //!< Interpolator for x
  std::shared_ptr<interpolator::InterpolatorInterface<double>> y_interpolator_{
    nullptr};  //!< Interpolator for y
  std::shared_ptr<interpolator::InterpolatorInterface<double>> z_interpolator_{
    nullptr};  //!< Interpolator for z

  std::vector<double> bases_;  //!< Axis of the trajectory

  double start_{0.0}, end_{0.0};  //!< Start and end of the arc length of the trajectory

  /**
   * @brief Validate the arc length is within the trajectory
   * @param s Arc length
   */
  double clamp(const double s, bool show_warning = false) const;

public:
  Trajectory();
  virtual ~Trajectory() = default;
  Trajectory(const Trajectory & rhs);
  Trajectory(Trajectory && rhs) = default;
  Trajectory & operator=(const Trajectory & rhs);
  Trajectory & operator=(Trajectory && rhs) = default;

  /**
   * @brief Get the underlying arc lengths of the trajectory
   * @return Vector of bases(arc lengths)
   */
  virtual std::vector<double> get_underlying_bases() const;
  /**
   * @brief Get the length of the trajectory
   * @return Length of the trajectory
   */
  double length() const;

  /**
   * @brief Compute the point on the trajectory at a given s value
   * @param s Arc length
   * @return Point on the trajectory
   */
  PointType compute(const double s) const;

  /**
   * @brief Compute the points on the trajectory at given s values
   * @param ss Arc lengths
   * @return Points on the trajectory
   */
  std::vector<PointType> compute(const std::vector<double> & ss) const;

  /**
   * @brief Build the trajectory from the points
   * @param points Vector of points
   * @return True if the build is successful
   */
  interpolator::InterpolationResult build(const std::vector<PointType> & points);

  /**
   * @brief Get the azimuth angle at a given s value
   * @param s Arc length
   * @return Azimuth in radians
   */
  double azimuth(const double s) const;

  /**
   * @brief Get the azimuth angles at given s values
   * @param ss Arc lengths
   * @return Azimuth in radians
   */
  std::vector<double> azimuth(const std::vector<double> & ss) const;

  /**
   * @brief Get the elevation angle at a given s value
   * @param s Arc length
   * @return Elevation in radians
   */
  double elevation(const double s) const;

  /**
   * @brief Get the curvature at a given s value
   * @param s Arc length
   * @return Curvature
   */
  double curvature(const double s) const;

  /**
   * @brief Get the curvature at a given s values
   * @param ss Arc lengths
   * @return Curvature
   */
  std::vector<double> curvature(const std::vector<double> & ss) const;

  /**
   * @brief Restore the trajectory points
   * @param min_points Minimum number of points
   * @return Vector of points
   */
  std::vector<PointType> restore(const size_t min_points = 4) const;

  void crop(const double start, const double length);

  /**
   * @brief return the list of base values from start_ to end_ with the given interval
   * @param tick the length of interval
   * @return array of double from start_ to end_ including the end_
   */
  std::vector<double> base_arange(const double tick) const
  {
    std::vector<double> ss;
    for (double s = start_; s < end_; s += tick) {
      ss.push_back(s);
    }
    if (ss.back() != end_) {
      ss.push_back(end_);
    }
    return ss;
  }

  class Builder
  {
  private:
    std::unique_ptr<Trajectory> trajectory_;

  public:
    Builder();

    /**
     * @brief create the default interpolator setting
     * @note CubicSpline for x, y and Linear for z
     */
    static void defaults(Trajectory * trajectory);

    template <class InterpolatorType, class... Args>
    Builder & set_xy_interpolator(Args &&... args)
    {
      trajectory_->x_interpolator_ =
        std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
      trajectory_->y_interpolator_ =
        std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
      return *this;
    }

    template <class InterpolatorType, class... Args>
    Builder & set_z_interpolator(Args &&... args)
    {
      trajectory_->z_interpolator_ =
        std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
      return *this;
    }

    tl::expected<Trajectory, interpolator::InterpolationFailure> build(
      const std::vector<PointType> & points);
  };
};

}  // namespace autoware::trajectory

#endif  // AUTOWARE__TRAJECTORY__POINT_HPP_
