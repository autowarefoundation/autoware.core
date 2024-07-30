// Copyright 2024 The Autoware Contributors
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

#ifndef TEST_CONSTANTS_HPP_
#define TEST_CONSTANTS_HPP_

namespace autoware::control_center::test
{
constexpr char const * topic_register_service = "/autoware/control_center/srv/register";
constexpr char const * topic_deregister_service = "/autoware/control_center/srv/deregister";
constexpr char const * topic_node_reports = "/autoware/control_center/node_reports";
constexpr char const * topic_heartbeat_suffix = "/heartbeat";
}  // namespace autoware::control_center::test

#endif  // TEST_CONSTANTS_HPP_
