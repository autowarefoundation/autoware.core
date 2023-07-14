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

#ifndef PARSE_VERSION_HPP_
#define PARSE_VERSION_HPP_

#include "version_types.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>

namespace autoware_version_manager
{
namespace parse_version
{

VersionAutoware parse_autoware_version(const YAML::Node & yaml_node);
VersionAutoware parse_autoware_version(const std::filesystem::path & path);

VersionInterface parse_interface_version(const YAML::Node & yaml_node);
VersionInterface parse_interface_version(const std::filesystem::path & path);

}  // namespace parse_version
}  // namespace autoware_version_manager

#endif  // PARSE_VERSION_HPP_
