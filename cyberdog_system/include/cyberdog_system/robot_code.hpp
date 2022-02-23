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
#ifndef CYBERDOG_SYSTEM__ROBOT_CODE_HPP_
#define CYBERDOG_SYSTEM__ROBOT_CODE_HPP_
#include <string>
#include <memory>

namespace cyberdog
{
namespace system
{
/**
 * @brief Cyberdog global code for module.
 *        Regulations are tough on hundreds place.
 */
enum class ModuleCode : int32_t
{
//  manager code
  kRobot = 100,      // cyberdog global manager
  kDevice = 200,
  kMotion = 300,
  kSensor = 400,

//  core code
  kAudio = 5200,
  kConnector = 1100,
  kCheck = 1200,
  kRemote = 1300,
  kNavigation = 3100,  // from motion
  kVisualProgram = 5100,  // from interaction

//  device code
  kLed = 2100,
  kWifi = 2200,
  kBms = 2300,
  kTouch = 2400,

// sensor code
  kCamera = 4100,
  kTof = 4200,
  kUltrasonic = 4300,
  kGps = 4400,
};  // enum class ModuleCode

/**
 * @brief Cyberdog system reserved code,
 *        (int) 0 - 30;
 *        All modules should set self-code between 31 - 99.
 */
enum class KeyCode : int32_t
{
  kFailed = -1,
  kOK = 0,
  kStateInvalid = 1,
  kStatusBusy = 2,
  kStatusError = 3,
  kNetError = 4,
  kPermissionDenied = 5,
  kTimeout = 6,
  kUnSupport = 7,
  kSelfCheckFailed = 8,
  kParametersInvalid = 9,
};  // enum class KeyCode
}  // namespace system
}  // namespace cyberdog

#endif  // CYBERDOG_SYSTEM__ROBOT_CODE_HPP_
