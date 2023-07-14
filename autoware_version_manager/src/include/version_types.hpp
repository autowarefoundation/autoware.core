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

#ifndef VERSION_TYPES_HPP_
#define VERSION_TYPES_HPP_

namespace autoware_version_manager
{
// Autoware version (CalVer with YYYY.MINOR.MICRO)
// https://calver.org/#scheme
struct VersionAutoware
{
  int year;   // year of release
  int minor;  // increments for non-breaking changes
  int micro;  // increments for bug fixes or patches
};

// Autoware component interface version (SemVer)
// https://semver.org/
struct VersionInterface
{
  int major;  // increments for breaking changes
  int minor;  // increments for non-breaking changes
  int patch;  // increments for bug fixes or patches
};

}  // namespace autoware_version_manager

#endif  // VERSION_TYPES_HPP_
