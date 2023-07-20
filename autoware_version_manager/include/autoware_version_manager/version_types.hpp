// Copyright 2023 The Autoware Foundation
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

#ifndef AUTOWARE_VERSION_MANAGER__VERSION_TYPES_HPP_
#define AUTOWARE_VERSION_MANAGER__VERSION_TYPES_HPP_

#include <cstdint>

namespace autoware_version_manager
{
// Autoware version (CalVer with YYYY.0M.MICRO)
// https://calver.org/#scheme
struct VersionAutoware
{
  using uint16_t = std::uint16_t;
  uint16_t year;   // year of release
  uint16_t month;  // month of release
  uint16_t micro;  // increments for bug fixes or patches
};

// Autoware component interface version (SemVer)
// https://semver.org/
struct VersionInterface
{
  using uint16_t = std::uint16_t;
  uint16_t major;  // increments for breaking changes
  uint16_t minor;  // increments for non-breaking changes
  uint16_t patch;  // increments for bug fixes or patches
};

}  // namespace autoware_version_manager

#endif  // AUTOWARE_VERSION_MANAGER__VERSION_TYPES_HPP_
