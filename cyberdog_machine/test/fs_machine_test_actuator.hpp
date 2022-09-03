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
 * @brief Finite State Machine test code.
 *        Descriptions:
 *                    1. This code will support Actuator Class.
 *                    2. The Actuator is anonymous, should be used with config file.
 *                    3. It is also demo code for using FS Machine in Cyberdog.
 *                    4. Specially, coding must design under doc: https://xiaomi.f.mioffice.cn/docs/dock4PIQuzX0tHEHbRAhDYM8YGe
 */
#include <chrono>
#include <thread>
#include <memory>
#include <mutex>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_machine/cyberdog_fs_machine.hpp"

enum class DemoCode : int32_t
{
  kDemoError1 = 21,
  kDemoError2 = 22,
  kDemoError3 = 23
};

class ActuactorDemo : public cyberdog::machine::MachineActuator
{
public:
  explicit ActuactorDemo(const std::string & name)
  : MachineActuator(name),
  name_(name) {
    node_ptr_ = rclcpp::Node::make_shared(name_);
    this->MachineActuatorInit(node_ptr_);
    code_ptr_ = std::make_shared<cyberdog::system::CyberdogCode<DemoCode>>(
    cyberdog::system::ModuleCode::kRobot);
    this->RegisterStateCallback((uint8_t)23, std::bind(&ActuactorDemo::OnCustom, this));
  }
  ~ActuactorDemo() {}
  void Spin() {
    INFO("ActuatorDemo: %s spin.", name_.c_str());
    rclcpp::spin(this->node_ptr_);
  }
public:
  /* 实现状态机虚函数，满足系统要求 */
  int32_t OnSetUp() {
    INFO("ActuatorDemo on setup.");
    return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
  }
  int32_t ONTearDown() {
    INFO("ActuatorDemo on teardown.");
    return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
  }
  int32_t OnSelfCheck() {
    INFO("ActuatorDemo on selfcheck.");
    return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
  }
  int32_t OnActive() {
    INFO("ActuatorDemo on active.");
    return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
  }
  int32_t OnProtected() {
    INFO("ActuatorDemo on protected.");
    return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
  }
  int32_t OnLowPower() {
    INFO("ActuatorDemo on lowpower.");
    return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
  }
  int32_t OnOTA() {
    INFO("ActuatorDemo on OTA.");
    return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
  }
  int32_t OnCustom() {
    INFO("ActuatorDemo on Custom.");
    return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
  }
private:
  std::string name_;
  rclcpp::Node::SharedPtr node_ptr_ {nullptr};
  std::shared_ptr<cyberdog::system::CyberdogCode<DemoCode>> code_ptr_ {nullptr};
};  // class ActuactorDemo
