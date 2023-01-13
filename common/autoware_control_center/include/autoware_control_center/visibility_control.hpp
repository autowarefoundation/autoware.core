// Copyright 2022 The Autoware Contributors
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

#ifndef AUTOWARE_CONTROL_CENTER__VISIBILITY_CONTROL_HPP_
#define AUTOWARE_CONTROL_CENTER__VISIBILITY_CONTROL_HPP_

#include "rcutils/visibility_control_macros.h"

#ifdef AUTOWARE_CONTROL_CENTER_BUILDING_DLL
#define AUTOWARE_CONTROL_CENTER_PUBLIC RCUTILS_EXPORT
#else
#define AUTOWARE_CONTROL_CENTER_PUBLIC RCUTILS_IMPORT
#endif  // !AUTOWARE_CONTROL_CENTER_BUILDING_DLL
#define AUTOWARE_CONTROL_CENTER_LOCAL RCUTILS_LOCAL

#endif  // AUTOWARE_CONTROL_CENTER__VISIBILITY_CONTROL_HPP_
