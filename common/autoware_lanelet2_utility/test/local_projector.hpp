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

#ifndef LOCAL_PROJECTOR_HPP_
#define LOCAL_PROJECTOR_HPP_

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/Projection.h>

#include <string>
#include <utility>

class LocalProjector : public lanelet::Projector
{
public:
  LocalProjector() : Projector(lanelet::Origin(lanelet::GPSPoint{})) {}

  lanelet::BasicPoint3d forward(const lanelet::GPSPoint & gps) const override  // NOLINT
  {
    return lanelet::BasicPoint3d{0.0, 0.0, gps.ele};
  }

  [[nodiscard]] lanelet::GPSPoint reverse(const lanelet::BasicPoint3d & point) const override
  {
    return lanelet::GPSPoint{0.0, 0.0, point.z()};
  }
};

inline lanelet::LaneletMapConstPtr load_local_coordinate_map(const std::string & path)
{
  lanelet::ErrorMessages errors{};
  LocalProjector projector;
  auto lanelet_map_ptr_mut = lanelet::load(path, projector, &errors);
  for (auto point : lanelet_map_ptr_mut->pointLayer) {
    point.x() = point.attribute("local_x").asDouble().value();
    point.y() = point.attribute("local_y").asDouble().value();
  }

  return lanelet::LaneletMapConstPtr{std::move(lanelet_map_ptr_mut)};
}

#endif  // LOCAL_PROJECTOR_HPP_
