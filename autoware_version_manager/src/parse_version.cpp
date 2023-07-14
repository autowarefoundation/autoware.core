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

#include "include/parse_version.hpp"

#include "include/version_types.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>

namespace autoware_version_manager
{
namespace parse_version
{

VersionAutoware parse_autoware_version(const YAML::Node & yaml_node)
{
  VersionAutoware version{};
  version.year = static_cast<uint16_t>(yaml_node["year"].as<int>());
  version.month = static_cast<uint16_t>(yaml_node["month"].as<int>());
  version.micro = static_cast<uint16_t>(yaml_node["micro"].as<int>());
  return version;
}

VersionAutoware parse_autoware_version(const std::filesystem::path & path)
{
  YAML::Node yaml_node = YAML::LoadFile(path.string());
  return parse_autoware_version(yaml_node);
}

VersionInterface parse_interface_version(const YAML::Node & yaml_node)
{
  VersionInterface version{};
  version.major = static_cast<uint16_t>(yaml_node["major"].as<int>());
  version.minor = static_cast<uint16_t>(yaml_node["minor"].as<int>());
  version.patch = static_cast<uint16_t>(yaml_node["patch"].as<int>());
  return version;
}

VersionInterface parse_interface_version(const std::filesystem::path & path)
{
  YAML::Node yaml_node = YAML::LoadFile(path.string());
  return parse_interface_version(yaml_node);
}

}  // namespace parse_version
}  // namespace autoware_version_manager
