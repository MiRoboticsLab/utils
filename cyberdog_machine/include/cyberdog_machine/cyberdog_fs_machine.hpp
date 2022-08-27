// Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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
/**
 * @brief Finite state machine, which is specially designed for cyberdog rop.
 *
 */
#ifndef CYBERDOG_MACHINE__CYBERDOG_FS_MACHINE_HPP_
#define CYBERDOG_MACHINE__CYBERDOG_FS_MACHINE_HPP_
#include <string>
#include <memory>
#include <chrono>
#include "rclcpp/node.hpp"
#include "cyberdog_common/cyberdog_json.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
namespace cyberdog
{
namespace machine
{
class MachineController
{};  // class MachineController

class MachineActuator
{};  // class MachineActuator
}  // namespace machine
}  // namespace cyberdog
#endif  // CYBERDOG_MACHINE__CYBERDOG_FS_MACHINE_HPP_
