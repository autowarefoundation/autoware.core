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

#include "autoware/trajectory/interpolator/akima_spline.hpp"
#include "autoware/trajectory/interpolator/cubic_spline.hpp"
#include "autoware/trajectory/interpolator/linear.hpp"
#include "autoware/trajectory/interpolator/stairstep.hpp"
#include "autoware/trajectory/pose.hpp"

#include <autoware/pyplot/pyplot.hpp>
#include <range/v3/all.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>

#include <autoware_planning_msgs/msg/path_point.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <random>
#include <vector>

// NOLINTBEGIN
int main_cubic_normal()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-0.3, 0.3);
  auto noise = [&]() { return dis(gen); };
  auto plt = autoware::pyplot::import();
  auto [fig, ax] = plt.subplots();

  using autoware::trajectory::interpolator::CubicSpline;
  using autoware::trajectory::interpolator::InterpolationFailure;

  std::vector<double> xs = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
  std::vector<double> ys = {0.0 + noise(), 1.0 + noise(), 2.0 + noise(),
                            3.0 + noise(), 4.0 + noise(), 5.0 + noise()};

  // You need to use the `Builder` to try to construct the interpolator
  const tl::expected<CubicSpline, InterpolationFailure> result =
    CubicSpline::Builder()
      .set_bases(xs)   // set the base
      .set_values(ys)  // set the value
      .build();        // finally, call build()

  // The result may be `InterpolationFailure` type if the input is not suitable
  // for this interpolator
  if (!result) {
    return 0;
  }

  // If successful, the interpolator is available.
  // In this case the cubic spline coefficients are calculated.
  const CubicSpline & trajectory = result.value();

  // arbitrary value can be computed continuously
  std::vector<double> x;
  std::vector<double> y;
  for (double s = xs.front(); s <= xs.back(); s += 0.05) {
    x.push_back(s);
    y.push_back(trajectory.compute(s));
  }
  ax.plot(Args(x, y), Kwargs("color"_a = "purple", "label"_a = "interpolated(cubic spline)"));

  // interpolate multiple values
  const auto ys2 = trajectory.compute(xs);
  ax.scatter(Args(xs, ys2), Kwargs("color"_a = "red", "label"_a = "underlying points"));

  ax.scatter(
    Args(1.5, trajectory.compute(1.5)),
    Kwargs("color"_a = "green", "marker"_a = "o", "label"_a = "x=1.5"));
  plt.axis(Args("equal"));
  plt.grid();
  plt.legend();
  fig.tight_layout();
  plt.savefig(Args("cubic_spline.svg"));

  return 0;
}

int main_cubic_error()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-0.3, 0.3);
  auto noise = [&]() { return dis(gen); };

  using autoware::trajectory::interpolator::CubicSpline;
  using autoware::trajectory::interpolator::InterpolationFailure;

  // Give only 3 points for CubicSpline, which is infeasible
  std::vector<double> xs = {0.0, 1.0, 2.0};
  std::vector<double> ys = {0.0 + noise(), 1.0 + noise(), 2.0 + noise()};

  // You need to use the `Builder` to try to construct the interpolator
  const tl::expected<CubicSpline, InterpolationFailure> result =
    CubicSpline::Builder()
      .set_bases(xs)   // set the base
      .set_values(ys)  // set the value
      .build();        // finally, call build()

  // Access to the error reason
  if (!result) {
    // says "base size 3 is less than minimum required 4"
    std::cout << result.error().what << std::endl;
  }
  return 0;
}

int main_akima()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-0.3, 0.3);
  auto noise = [&]() { return dis(gen); };
  auto plt = autoware::pyplot::import();
  auto [fig, axes] = plt.subplots(1, 2);
  auto & ax1 = axes[0];
  auto & ax2 = axes[1];

  using autoware::trajectory::interpolator::AkimaSpline;
  using autoware::trajectory::interpolator::InterpolationFailure;

  std::vector<double> xs = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
  std::vector<double> ys = {0.0 + noise(), 1.0 + noise(), 2.0 + noise(),
                            3.0 + noise(), 4.0 + noise(), 5.0 + noise()};

  // You need to use the `Builder` to try to construct the interpolator
  const tl::expected<AkimaSpline, InterpolationFailure> result =
    AkimaSpline::Builder()
      .set_bases(xs)   // set the base
      .set_values(ys)  // set the value
      .build();        // finally, call build()
  const auto & trajectory = result.value();
  const auto x = trajectory.base_arange(0.05);
  const auto y = trajectory.compute(x);
  const auto dy = trajectory.compute_first_derivative(x);
  const auto ddy = trajectory.compute_second_derivative(x);
  ax1.plot(Args(x, y), Kwargs("color"_a = "purple", "label"_a = "akima spline"));
  ax1.scatter(Args(xs, ys), Kwargs("color"_a = "red", "marker"_a = "o", "label"_a = "underlying"));
  ax1.legend();
  ax1.grid();
  ax1.set_aspect(Args("equal"));
  ax1.set_xlim(Args(-1.0, 6.0));
  ax1.set_ylim(Args(-1.0, 6.0));

  ax2.plot(Args(x, dy), Kwargs("color"_a = "magenta", "label"_a = "1st derivative"));
  ax2.plot(Args(x, ddy), Kwargs("color"_a = "purple", "label"_a = "2nd derivative"));
  ax2.legend();
  ax2.grid();
  ax2.set_aspect(Args("equal"));
  ax2.set_xlim(Args(-1.0, 6.0));
  ax2.set_ylim(Args(-3.0, 4.0));

  fig.tight_layout();
  plt.savefig(Args("akima_spline.svg"));
  return 0;
}

int main_cubic()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-0.3, 0.3);
  auto noise = [&]() { return dis(gen); };
  auto plt = autoware::pyplot::import();
  auto [fig, axes] = plt.subplots(1, 2);
  auto & ax1 = axes[0];
  auto & ax2 = axes[1];

  using autoware::trajectory::interpolator::CubicSpline;
  using autoware::trajectory::interpolator::InterpolationFailure;

  std::vector<double> xs = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
  std::vector<double> ys = {0.0 + noise(), 1.0 + noise(), 2.0 + noise(),
                            3.0 + noise(), 4.0 + noise(), 5.0 + noise()};

  // You need to use the `Builder` to try to construct the interpolator
  const tl::expected<CubicSpline, InterpolationFailure> result =
    CubicSpline::Builder()
      .set_bases(xs)   // set the base
      .set_values(ys)  // set the value
      .build();        // finally, call build()
  const auto & trajectory = result.value();
  const auto x = trajectory.base_arange(0.05);
  const auto y = trajectory.compute(x);
  const auto dy = trajectory.compute_first_derivative(x);
  const auto ddy = trajectory.compute_second_derivative(x);
  ax1.plot(Args(x, y), Kwargs("color"_a = "purple", "label"_a = "cubic spline"));
  ax1.scatter(Args(xs, ys), Kwargs("color"_a = "red", "marker"_a = "o", "label"_a = "underlying"));
  ax1.legend();
  ax1.grid();
  ax1.set_aspect(Args("equal"));
  ax1.set_xlim(Args(-1.0, 6.0));
  ax1.set_ylim(Args(-1.0, 6.0));

  ax2.plot(Args(x, dy), Kwargs("color"_a = "magenta", "label"_a = "1st derivative"));
  ax2.plot(Args(x, ddy), Kwargs("color"_a = "purple", "label"_a = "2nd derivative"));
  ax2.legend();
  ax2.grid();
  ax2.set_aspect(Args("equal"));
  ax2.set_xlim(Args(-1.0, 6.0));
  ax2.set_ylim(Args(-3.0, 4.0));

  fig.tight_layout();
  plt.savefig(Args("cubic_spline.svg"));
  return 0;
}

int main_linear()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-0.3, 0.3);
  auto noise = [&]() { return dis(gen); };
  auto plt = autoware::pyplot::import();
  auto [fig, axes] = plt.subplots(1, 2);
  auto & ax1 = axes[0];
  auto & ax2 = axes[1];

  using autoware::trajectory::interpolator::InterpolationFailure;
  using autoware::trajectory::interpolator::Linear;

  std::vector<double> xs = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
  std::vector<double> ys = {0.0 + noise(), 1.0 + noise(), 2.0 + noise(),
                            3.0 + noise(), 4.0 + noise(), 5.0 + noise()};

  // You need to use the `Builder` to try to construct the interpolator
  const tl::expected<Linear, InterpolationFailure> result = Linear::Builder()
                                                              .set_bases(xs)   // set the base
                                                              .set_values(ys)  // set the value
                                                              .build();  // finally, call build()
  const auto & trajectory = result.value();
  const auto x = trajectory.base_arange(0.05);
  const auto y = trajectory.compute(x);
  const auto dy = trajectory.compute_first_derivative(x);
  const auto ddy = trajectory.compute_second_derivative(x);
  ax1.plot(Args(x, y), Kwargs("color"_a = "purple", "label"_a = "linear"));
  ax1.scatter(Args(xs, ys), Kwargs("color"_a = "red", "marker"_a = "o", "label"_a = "underlying"));
  ax1.legend();
  ax1.grid();
  ax1.set_aspect(Args("equal"));
  ax1.set_xlim(Args(-1.0, 6.0));
  ax1.set_ylim(Args(-1.0, 6.0));

  ax2.plot(Args(x, dy), Kwargs("color"_a = "magenta", "label"_a = "1st derivative"));
  ax2.plot(Args(x, ddy), Kwargs("color"_a = "purple", "label"_a = "2nd derivative"));
  ax2.legend();
  ax2.grid();
  ax2.set_aspect(Args("equal"));
  ax2.set_xlim(Args(-1.0, 6.0));
  ax2.set_ylim(Args(-3.0, 4.0));

  fig.tight_layout();
  plt.savefig(Args("linear.svg"));
  return 0;
}

int main_stairstep()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-0.3, 0.3);
  auto noise = [&]() { return dis(gen); };
  auto plt = autoware::pyplot::import();
  auto [fig, axes] = plt.subplots(1, 2);
  auto & ax1 = axes[0];
  auto & ax2 = axes[1];

  using autoware::trajectory::interpolator::InterpolationFailure;
  using autoware::trajectory::interpolator::Stairstep;

  std::vector<double> xs = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
  std::vector<double> ys = {0.0 + noise(), 1.0 + noise(), 2.0 + noise(),
                            3.0 + noise(), 4.0 + noise(), 5.0 + noise()};

  // You need to use the `Builder` to try to construct the interpolator
  const tl::expected<Stairstep<double>, InterpolationFailure> result =
    Stairstep<double>::Builder()
      .set_bases(xs)   // set the base
      .set_values(ys)  // set the value
      .build();        // finally, call build()
  const auto & trajectory = result.value();
  const auto x = trajectory.base_arange(0.05);
  const auto y = trajectory.compute(x);
  const auto dy = trajectory.compute_first_derivative(x);
  const auto ddy = trajectory.compute_second_derivative(x);
  ax1.plot(Args(x, y), Kwargs("color"_a = "purple", "label"_a = "stairstep"));
  ax1.scatter(Args(xs, ys), Kwargs("color"_a = "red", "marker"_a = "o", "label"_a = "underlying"));
  ax1.legend();
  ax1.grid();
  ax1.set_aspect(Args("equal"));
  ax1.set_xlim(Args(-1.0, 6.0));
  ax1.set_ylim(Args(-1.0, 6.0));

  ax2.plot(Args(x, dy), Kwargs("color"_a = "magenta", "label"_a = "1st derivative"));
  ax2.plot(Args(x, ddy), Kwargs("color"_a = "purple", "label"_a = "2nd derivative"));
  ax2.legend();
  ax2.grid();
  ax2.set_aspect(Args("equal"));
  ax2.set_xlim(Args(-1.0, 6.0));
  ax2.set_ylim(Args(-3.0, 4.0));

  fig.tight_layout();
  plt.savefig(Args("stairstep.svg"));
  return 0;
}

int main_coordinate_approximation()
{
  auto plt = autoware::pyplot::import();
  auto fig = plt.figure();
  auto ax = fig.add_subplot(Kwargs("projection"_a = "3d"));

  auto pose = [](const double x, const double y, const double z) -> geometry_msgs::msg::Point {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
  };

  using autoware::trajectory::Trajectory;
  using autoware::trajectory::interpolator::CubicSpline;
  using autoware::trajectory::interpolator::InterpolationFailure;
  using autoware::trajectory::interpolator::Linear;

  const double root2 = std::sqrt(2.0);
  const double root3 = std::sqrt(3.0);
  const double root6 = std::sqrt(6.0);

  // underlying
  std::vector<geometry_msgs::msg::Point> points = {
    pose(0.0, 0.0, 0.0),                                                            // P0
    pose(1.0 / root2, 1.0 / root2, 0.0),                                            // P1
    pose(1.0 / root2 + 2.0, 1.0 / root2, 0.0),                                      // P2
    pose(2.0 / root2 + 2.0, 2.0 / root2, 0.0),                                      // P3
    pose(2.0 / root2 + 2.0 + 1.0 / root6, 2.0 / root2 + 1.0 / root3, 1.0 / root2),  // P4
  };
  const auto points_x = points | ranges::views::transform([&](const auto & p) { return p.x; }) |
                        ranges::to<std::vector>();
  const auto points_y = points | ranges::views::transform([&](const auto & p) { return p.y; }) |
                        ranges::to<std::vector>();
  const auto points_z = points | ranges::views::transform([&](const auto & p) { return p.z; }) |
                        ranges::to<std::vector>();
  ax.scatter(
    Args(points_x, points_y, points_z),
    Kwargs("color"_a = "red", "marker"_a = "o", "label"_a = "underlying"));

  // cubic
  const auto cubic = Trajectory<geometry_msgs::msg::Point>::Builder().build(points).value();
  const auto s = cubic.base_arange(0.05);
  const auto C = cubic.compute(s);
  const auto Cx =
    C | ranges::views::transform([&](const auto & p) { return p.x; }) | ranges::to<std::vector>();
  const auto Cy =
    C | ranges::views::transform([&](const auto & p) { return p.y; }) | ranges::to<std::vector>();
  const auto Cz =
    C | ranges::views::transform([&](const auto & p) { return p.z; }) | ranges::to<std::vector>();
  ax.plot(
    Args(Cx, Cy, Cz),
    Kwargs("color"_a = "purple", "label"_a = "cubic interpolation", "zdir"_a = "z"));

  // linear(for description)
  const auto linear = Trajectory<geometry_msgs::msg::Point>::Builder()
                        .set_xy_interpolator<Linear>()
                        .set_z_interpolator<Linear>()
                        .build(points)
                        .value();
  const auto C_linear = linear.compute(s);
  const auto Cx_linear = C_linear | ranges::views::transform([&](const auto & p) { return p.x; }) |
                         ranges::to<std::vector>();
  const auto Cy_linear = C_linear | ranges::views::transform([&](const auto & p) { return p.y; }) |
                         ranges::to<std::vector>();
  const auto Cz_linear = C_linear | ranges::views::transform([&](const auto & p) { return p.z; }) |
                         ranges::to<std::vector>();
  ax.plot(
    Args(Cx_linear, Cy_linear, Cz_linear),
    Kwargs("color"_a = "k", "label"_a = "arc length", "zdir"_a = "z", "linestyle"_a = "dotted"));
  ax.set_zlim(Args(0.0, 0.7));

  // test point
  const auto p_base = linear.compute(0.5);
  const auto p_cubic = cubic.compute(0.5);
  ax.scatter(
    Args(p_base.x, p_base.y, p_base.z),
    Kwargs("color"_a = "black", "label"_a = "s=0.5", "marker"_a = "x"));
  ax.scatter(
    Args(p_cubic.x, p_cubic.y, p_cubic.z), Kwargs("color"_a = "purple", "label"_a = "C(s=0.5)"));

  ax.text(Args(points[0].x, points[0].y, points[0].z + 0.1, "P0"), Kwargs("fontsize"_a = 12));
  ax.text(Args(points[1].x, points[1].y, points[1].z + 0.1, "P1"), Kwargs("fontsize"_a = 12));
  ax.text(Args(points[2].x, points[2].y, points[2].z + 0.1, "P2"), Kwargs("fontsize"_a = 12));
  ax.text(Args(points[3].x, points[3].y, points[3].z + 0.1, "P3"), Kwargs("fontsize"_a = 12));
  ax.text(Args(points[4].x, points[4].y, points[4].z + 0.1, "P4"), Kwargs("fontsize"_a = 12));
  fig.tight_layout();
  ax.legend();
  ax.set_aspect(Args("equal"));
  ax.view_init(Args(), Kwargs("elev"_a = 25, "azim"_a = -69, "roll"_a = 0));
  plt.show();
  return 0;
}

int main_curvature()
{
  auto plt = autoware::pyplot::import();
  auto [fig, axes] = plt.subplots(1, 2);
  auto & ax = axes[0];
  auto & ax1 = axes[1];

  auto pose = [](const double x, const double y, const double z) -> geometry_msgs::msg::Pose {
    geometry_msgs::msg::Pose p;
    p.position.x = x;
    p.position.y = y;
    p.position.z = z;
    p.orientation.w = 1.0;
    return p;
  };

  using autoware::trajectory::Trajectory;
  using autoware::trajectory::interpolator::CubicSpline;
  using autoware::trajectory::interpolator::InterpolationFailure;
  using autoware::trajectory::interpolator::Linear;

  const double root2 = std::sqrt(2.0);
  const double root3 = std::sqrt(3.0);
  const double root6 = std::sqrt(6.0);

  // underlying
  std::vector<geometry_msgs::msg::Pose> points = {
    pose(0.0, 0.0, 0.0),                                                            // P0
    pose(1.0 / root2, 1.0 / root2, 0.0),                                            // P1
    pose(1.0 / root2 + 2.0, 1.0 / root2, 0.0),                                      // P2
    pose(2.0 / root2 + 2.0, 2.0 / root2, 0.0),                                      // P3
    pose(2.0 / root2 + 2.0 + 1.0 / root6, 2.0 / root2 + 1.0 / root3, 1.0 / root2),  // P4
  };
  const auto points_x = points |
                        ranges::views::transform([&](const auto & p) { return p.position.x; }) |
                        ranges::to<std::vector>();
  const auto points_y = points |
                        ranges::views::transform([&](const auto & p) { return p.position.y; }) |
                        ranges::to<std::vector>();
  ax.scatter(
    Args(points_x, points_y),
    Kwargs("color"_a = "red", "marker"_a = "o", "label"_a = "underlying"));

  // cubic
  auto cubic = Trajectory<geometry_msgs::msg::Pose>::Builder{}.build(points).value();
  cubic.align_orientation_with_trajectory_direction();
  const auto s = cubic.base_arange(0.05);
  const auto C = cubic.compute(s);
  const auto Cx = C | ranges::views::transform([&](const auto & p) { return p.position.x; }) |
                  ranges::to<std::vector>();
  const auto Cy = C | ranges::views::transform([&](const auto & p) { return p.position.y; }) |
                  ranges::to<std::vector>();
  ax.plot(Args(Cx, Cy), Kwargs("color"_a = "purple", "label"_a = "cubic interpolation"));

  // linear(for description)
  const auto linear = Trajectory<geometry_msgs::msg::Pose>::Builder()
                        .set_xy_interpolator<Linear>()
                        .set_z_interpolator<Linear>()
                        .build(points)
                        .value();
  const auto C_linear = linear.compute(s);
  const auto Cx_linear = C_linear |
                         ranges::views::transform([&](const auto & p) { return p.position.x; }) |
                         ranges::to<std::vector>();
  const auto Cy_linear = C_linear |
                         ranges::views::transform([&](const auto & p) { return p.position.y; }) |
                         ranges::to<std::vector>();
  ax.plot(
    Args(Cx_linear, Cy_linear),
    Kwargs("color"_a = "k", "label"_a = "arc length", "linestyle"_a = "dotted"));

  // curvature
  const auto base_ss = std::vector<double>({0.0, 1.0, 3.0, 4.0, 5.0});
  const auto cos_yaw =
    base_ss | ranges::views::transform([&](const auto ss) {
      const auto p = cubic.compute(ss);
      tf2::Vector3 x_axis(1.0, 0.0, 0.0);
      tf2::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
      tf2::Vector3 direction = tf2::quatRotate(q, x_axis);
      return direction.x();
    }) |
    ranges::to<std::vector>();
  const auto sin_yaw =
    base_ss | ranges::views::transform([&](const auto ss) {
      const auto p = cubic.compute(ss);
      tf2::Vector3 x_axis(1.0, 0.0, 0.0);
      tf2::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
      tf2::Vector3 direction = tf2::quatRotate(q, x_axis);
      return direction.y();
    }) |
    ranges::to<std::vector>();

  ax1.plot(Args(s, cubic.curvature(s)), Kwargs("color"_a = "navy", "label"_a = "curvature"));
  ax.quiver(
    Args(points_x, points_y, cos_yaw, sin_yaw), Kwargs("color"_a = "green", "label"_a = "yaw"));

  fig.tight_layout();
  for (auto & a : axes) {
    a.legend();
    a.grid();
    a.set_aspect(Args("equal"));
  }
  plt.show();
  return 0;
}

int main()
{
  pybind11::scoped_interpreter guard{};

  /*
  main_cubic_normal();
  main_cubic_error();
  main_akima();
  main_cubic();
  main_linear();
  main_stairstep();
  */
  // main_coordinate_approximation();
  main_curvature();
}
// NOLINTEND
