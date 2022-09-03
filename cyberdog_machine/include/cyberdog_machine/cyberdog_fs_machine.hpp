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
#include <vector>
#include <map>
#include <algorithm>
#include <functional>
#include "rclcpp/node.hpp"
#include "cyberdog_common/cyberdog_json.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_system/robot_code.hpp"
#include "protocol/srv/fs_machine.hpp"
namespace cyberdog
{
namespace machine
{
/**
 * @brief 状态机指令，用来区分查询与设置
 * 
 */
enum class StateCmd : uint8_t
{
  kQuery = 0,  // 查询
  kSet = 1     // 设置
};  // enum class StateCmd

/**
 * @brief 控制器配置参数管理类
 * 
 */
class ControllerParams
{
public:
  bool Build(const toml::value & controller) {
    toml::array actuators;
    if(!common::CyberdogToml::Get(controller, "actuators", actuators)) {
      ERROR("Build ControllerParams failed, cannot get actuators!");
      return false;
    }
    toml::array states;
    if(!common::CyberdogToml::Get(controller, "actuators", states)) {
      ERROR("Build ControllerParams failed, cannot get states!");
      return false;
    }
    if(!common::CyberdogToml::Get(controller, "actuators", default_time_)) {
      ERROR("Build ControllerParams failed, cannot get states!");
      return false;
    }
    if(!common::CyberdogToml::Get(controller, "actuators", default_state_)) {
      ERROR("Build ControllerParams failed, cannot get states!");
      return false;
    }
    return true;
  }

  bool CheckActuator(const std::string & actuator) {
    return std::find(actuators_vec_.begin(), actuators_vec_.end(), actuator) == actuators_vec_.end() ? false : true;
  }

  bool CheckState(const std::string & state) {
    return std::find(state_vec_.begin(), state_vec_.end(), actuator) == state_vec_.end() ? false : true;
  }

  int32_t GetDefaultTime() {
    return default_time_;
  }

  std::string GetDefaultState() {
    return default_state_;
  }
private:
  std::vector<std::string> actuators_vec_;
  std::vector<std::string> states_vec_;
  int32_t default_time_;
  std::string default_state_;
};  // class ControllerParams

/**
 * @brief 执行器配置参数管理类
 * 
 */
class ActuatorParams final
{
public:
  bool Build(const toml::array & states, const toml::array & time) {
    if(states.size() != time.size()) {
      ERROR("Build ActuatorParams failed, states size is not equal with time!");
      return false;
    }
    for(size_t i = 0; i <= states.size(); i++) {
      states_map_.insert(std::make_pair(states[i].as_string(), time[i].as_integer()));
    }
    return true;
  }

  int32_t GetTime(const std::string & state) {
    auto iter = states_map_.find(state);
    if(iter != states_map_.end()) {
      ERROR("GetTime failed, state invalid!");
      return iter->second;
    } else {
      return -1;
    }
  }

  bool CheckState(const std::string & state) {
    return states_map_.find(state) == states_map_.end() ? false : true;
  }
private:
  std::map<std::string, int32_t> states_map_;
  std::string current_state_;
};  // class ActuatorParams

/**
 * @brief 状态机控制器类
 * 
 */
class MachineController
{
using FSMACHINE_SRV_T = protocol::srv::FsMachine;
using FS_CLINET_T = rclcpp::Client<FSMACHINE_SRV_T>::SharedPtr;
public:
  MachineController() {
    controller_params_ptr_ = std::make_shared<ControllerParams>();
  }
  ~MachineController() {}

  /* 状态机不允许通过转移、赋值、拷贝来构造 */
  MachineController(const MachineController &) = delete;
  MachineController& operator=(const MachineController &) = delete;
  MachineController(MachineController &&) = delete;
  MachineController& operator=(MachineController &&) = delete;

public:
  /**
   * @brief 初始化控制器
   * 
   * @param node_ptr 
   * @return true 只有返回初始化成功，才可以使用状态机
   * @return false 若初始化失败，状态机不可用，否则在切换、查询中其行为未定义
   */
  bool MachineControllerInit(const std::string& config_file, rclcpp::Node::SharedPtr node_ptr) {
    INFO("Init FS Machine.");
    toml::value config;
    if(!common::CyberdogToml::ParseFile(config_file, config)) {
      ERROR("Parse FS Machine config file failed, toml file is invalid!");
      return false;
    }
    toml::value actuator;
    if(!common::CyberdogToml::Get(config, "actuator", actuator)) {
      ERROR("FS Machine init failed, parse actuators config failed!");
      return false;
    } else if(!BuildActuatorMap(actuator)) {
      ERROR("FS Machine init failed, build actuators params failed!");
      return false;
    }
    toml::value controller;
    if(!common::CyberdogToml::Get(config, "controller", controller)) {
      ERROR("FS Machine init failed, parse controller config failed!");
      return false;
    } else if(!BuildControllerParams(controller)) {
      ERROR("FS Machine init failed, build controller failed!");
      return false;
    }

    /* controller中的actuator与单独声明的actuator数量要求一致，检测失败则返回错误 */
    toml::array actuator_array;
    if(!common::CyberdogToml::Get(controller, "actuators", actuator_array)) {
      ERROR("FS Machine init failed, parse actuator array failed!");
      return false;
    }
    if(actuator.size() != actuator_array.size()) {
      ERROR("FS Machine init failed, actuator size invalid!");
      return false;
    }
    BuildClientMap(actuator_array);
    BuildStateMap(actuator_array);

    // 最后处理指针，防止错误持有智能指针
    if(node_ptr == nullptr) {
      ERROR("FS Machine init failed, ros node pointer is invalid!");
      return false;
    } else {
      node_ptr_ = node_ptr;
    }
    return true;
  }

  /**
   * @brief 阻塞函数，等待所有执行器接入状态机
   * 
   * @return true 全部接入成功
   * @return false 超时
   */
  bool WaitActuatorsSetUp() {
    INFO("MachineController wait for all actuators setup.");
    return std::all_of(actuator_map_.cbegin(), actuator_map_.cend(), [this](const auto & iter){
      auto client_iter = this->client_map_.find(iter->first);
      if(client_iter == client_map_.end()) {
        ERROR("MachineController waot actuator: %s setup failed, maybe has an error while building.", iter->first.c_str());
        return false;
      }
      if(!client_iter->second->wait_for_service(std::chrono::milliseconds(iter->second.GetTime("SetUp")))){  // 需要改成config里的软编码
        ERROR("MachineController wait actuator: %s setup failed, timeout!", iter->first.c_str());
        return false;
      }
      INFO("MachineController wait actuator: %s setup OK.", iter->first.c_str());
      return true;
    });
  }

  /**
   * @brief 更新状态机记录字典
   * 
   * @return true 
   * @return false 
   */
  bool UpdateStateMap() {
    auto result = std::all_of(state_map_.begin(), state_map_.end(), [this](auto & iter){
      return this->GetState(iter.first, iter.second);
    });
    return result;
  }

  /**
   * @brief Get the State Map object
   * 
   * @param state_map 
   */
  void GetStateMap(std::map<std::string, uint8_t> & state_map) {
    if(!UpdateStateMap()) {
      ERROR("MachineController update state map failed, will use last state version.");
    }
    state_map = state_map_;
  }

  /**
   * @brief 获取特定Actuator的状态
   * 
   * @param target_actuator 目标name
   * @param state 特别地，使用引用返回状态值
   */
  bool GetState(const std::string & target_actuator, uint8_t & state) {
    auto iter = client_map_.find(target_actuator);
    if(iter == client_map_.end()) {
      ERROR("MachineController set state failed, target actuator: %s is invalid!", target_actuator.c_str());
      return false;
    }
    auto request = std::make_shared<FSMACHINE_SRV_T::Request>();
    request->cmd = static_cast<uint8_t>(StateCmd::kQuery);
    auto result = iter->second->async_send_request(request);
    if(rclcpp::spin_until_future_complete(node_ptr_, result, std::chrono::seconds(2)) != rclcpp::FutureReturnCode::SUCCESS){
      ERROR("MachineController get state failed, get result failed!");
      return false;
    }
    if(result.get()->code != static_cast<int32_t>(cyberdog::system::KeyCode::kOK)) {
      ERROR("MachineController get state failed, result code is not OK!");
      return false;
    } else {
      state = result.get()->current_state;
      return true;
    }
  }

  /**
   * @brief 设置特定Actuator的状态
   * 
   * @param target_actuator 目标name
   * @param target_state 目标状态
   * @return true 略。
   * @return false 如果返回失败，则目标状态为前一个状态，不受影响
   */
  bool SetState(const std::string & target_actuator, uint8_t target_state ) {
    auto iter = client_map_.find(target_actuator);
    if(iter == client_map_.end()) {
      ERROR("MachineController set state failed, target actuator: %s is invalid!", target_actuator.c_str());
      return false;
    }
    auto request = std::make_shared<FSMACHINE_SRV_T::Request>();
    request->cmd = static_cast<uint8_t>(StateCmd::kSet);
    request->target_state = target_state;
    auto response = std::make_shared<FSMACHINE_SRV_T::Response>();
    auto result = iter->second->async_send_request(request);
    if(rclcpp::spin_until_future_complete(node_ptr_, result, std::chrono::seconds(2)) != rclcpp::FutureReturnCode::SUCCESS){
      ERROR("MachineController set state failed, get service result failed!");
      return false;
    }
    if(result.get()->code != static_cast<int32_t>(cyberdog::system::KeyCode::kOK)) {
      ERROR("MachineController set state failed, result code is not OK, expect 0 but actual is: %d.", result.get()->code);
      return false;
    }
    return true;
  }

  /**
   * @brief 设置全局状态机
   * 
   * @param target_state 目标状态
   * @return true 
   * @return false 
   */
  bool SetState(uint8_t target_state) {
    INFO("MachineController set global state: %d", target_state);
    auto set_result = std::all_of(target_vec_.cbegin(), target_vec_.cend(), [this, target_state](const std::string & name) {
      INFO("all of name: %s, target: %d", name.c_str(), target_state);
      if(!SetState(name, target_state)) {
        ERROR("MachineController set state faild, target: %s, state: %d", name.c_str(), target_state);
        return false;
      }
      return true;
    });
    INFO("MachineController set state result: %d", set_result);
    return set_result;
  }
private:
  /* Internal API */
  /**
   * @brief 
   * 
   * @param target_array 
   */
  void BuildStateMap(const toml::array & target_array) {
    for(size_t i = 0;i < target_array.size(); i++) {  // TODO: 是否有更优雅的读取-添加方案 ？
      std::string tmp;
      common::CyberdogToml::Get(target_array, i, tmp);
      state_map_.insert(std::make_pair(tmp, static_cast<uint8_t>(StateKey::kUninitialized)));
    }
  }

  /**
   * @brief 创建ros客户端字典
   * 
   * @param target_array 
   */
  void BuildClientMap(const toml::array & target_array) {
    for(size_t i = 0;i < target_array.size(); i++) {  // TODO: 是否有更优雅的读取-添加方案 ？
      std::string tmp;
      common::CyberdogToml::Get(target_array, i, tmp);
      INFO("tmp info: %s", tmp.c_str());
      target_vec_.emplace_back(tmp);
      std::string client_name = tmp + std::string("machine_service");
      auto client = node_ptr_->create_client<protocol::srv::FsMachine>(client_name);
      client_map_.insert(std::make_pair(tmp, std::move(client)));
    }
  }

  bool BuildActuatorMap(const toml::value & actuator_params) {
    INFO("BuildActuatorMap on call.");
    return std::all_of(target_vec_.cbegin(), target_vec_.cend(), [this, actuator_params](const std::string & name){
      toml::value params;
      if(!common::CyberdogToml::Get(actuator_params, name, params)) {
        ERROR("BuildActuatorMap failed, cannot parse params!");
        return false;
      }
      if(!BuildActuatorParams(name, params)) {
        ERROR("BuildActuatorMap failed, cannot build params!");
        return false;
      }
      return true;
    });
  }

  bool BuildActuatorParams(const std::string name, const toml::value & params) {
    INFO("BuildActuatorParams on call.");
    toml::array states;
    if(!common::CyberdogToml::Get(params, "states", states)) {
      ERROR("BuildActuatorParams failed, cannot parse param states!");
      return false;
    }
    toml::array times;
    if(!common::CyberdogToml::Get(params, "times", times)) {
      ERROR("BuildActuatorParams failed, cannot parse param times!");
      return false;
    }
    ActuatorParams actuator_params;
    if(!actuator_params.Build(states, times)) {
      ERROR("BuildActuatorPrams failed!");
      return false;
    } else {
      actuator_map_.insert(std::make_pair(name, actuator_params));
    }
    return true;
  }

  bool BuildControllerParams(const toml::value & controller_params) {
    return controller_params_ptr_->Build(controller_params);
  }
private:
  std::vector<std::string> target_vec_;               // 执行器容器
  // toml::array target_array_;                          // 执行器列表
  rclcpp::Node::SharedPtr node_ptr_ {nullptr};        // 节点指针，用于内置进程间Ros通信
  std::map<std::string, FS_CLINET_T> client_map_;     // 执行器与Ros服务客户端反射
  std::map<std::string, uint8_t> state_map_;          // 状态机记录字典
  std::map<std::string, ActuatorParams> actuator_map_;
  std::shared_ptr<ControllerParams> controller_params_ptr_;
};  // class MachineController

/**
 * @brief 状态机执行器类
 * 
 */
class MachineActuator
{
using FSMACHINE_SRV_T = protocol::srv::FsMachine;
public:
  explicit MachineActuator(const std::string & name) : name_(name) {}
  ~MachineActuator() {}

  /* 状态机不允许通过转移、赋值、拷贝来构造 */
  MachineActuator(const MachineActuator &) = delete;
  MachineActuator& operator=(const MachineActuator &) = delete;
  MachineActuator(MachineActuator &&) = delete;
  MachineActuator& operator=(MachineActuator &&) = delete;

public:
  /**
   * @brief 初始化状态机
   *        若该操作失败，则后续状态机行为是未定义的
   * 
   * @param node_ptr 
   * @return true 
   * @return false 
   */
  bool MachineActuatorInit(rclcpp::Node::SharedPtr node_ptr) {
    INFO("MachineActuator Init on call");
    if(node_ptr == nullptr) {
      ERROR("MachineActuator init failed, node pointer is invalid!");
      return false;
    } else {
      node_ptr_ = node_ptr;
    }
    std::string service_name = name_ + std::string("machine_service");
    machine_service_ptr_ = node_ptr_->create_service<FSMACHINE_SRV_T>(service_name, std::bind(&MachineActuator::ServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
    BuildStateMap();
    return true;
  }
  /**
   * @brief 
   * 
   * @param state 
   * @param callback 
   */
  void RegisterStateCallback(uint8_t state, std::function<int32_t(void)> callback) {
    if(state_map_.find(state) != state_map_.end()) {
      ERROR("MachineActuator regitster failed, state: %d is invalid!", state);
    } else {
      state_map_.insert(std::make_pair(state, callback));
    }
  }

  bool ActuatorRun() {
    return truel
  }
private:
  /* 系统要求具备的状态机 */
  virtual int32_t OnSetUp() = 0;
  virtual int32_t ONTearDown() = 0;
  virtual int32_t OnSelfCheck() = 0;
  virtual int32_t OnActive() = 0;
  virtual int32_t OnProtected() = 0;
  virtual int32_t OnLowPower() = 0;
  virtual int32_t OnOTA() = 0;
private:
  void ServiceCallback(const FSMACHINE_SRV_T::Request::SharedPtr request, FSMACHINE_SRV_T::Response::SharedPtr response) {
    INFO("MachineActuator service callback on call.\n\tcmd: %d, target_state: %d", request->cmd, request->target_state);
    switch (request->cmd)
    {
    case (uint8_t)StateCmd::kQuery:
      QueryState(response);
      break;
    case (uint8_t)StateCmd::kSet:
      response->code = CheckoutState(request->target_state);
      break;
    default:
      WARN("MachineActuator cannot work, invalid cmd: %d", request->cmd);
      response->code = static_cast<int32_t>(system::KeyCode::kParametersInvalid);
      break;
    }
  }

  /**
   * @brief 
   * 
   * @param state 
   * @return int32_t 
   */
  int32_t CheckoutState(uint8_t state) {
    INFO("MachineActuator checkout state: %d.", state);
    auto iter = state_map_.find(state);
    if(iter == state_map_.end()) {
      INFO("MachineActuator has no state: %d, skip once.", state);
      return static_cast<int32_t>(system::KeyCode::kUnSupport);
    }
    auto result = iter->second();
    if(result == static_cast<int32_t>(system::KeyCode::kOK)) {
      INFO("MachineActuator checkout state: %d successfully.", state);
      current_state_ = state;
    } else {
      ERROR("MachineActuator checkout state: %d failed, error code: %d", state, result);
      return static_cast<int32_t>(system::KeyCode::kUnSupport);
    }
    return result;
  }

  /**
   * @brief 
   * 
   * @param response 
   */
  void QueryState(FSMACHINE_SRV_T::Response::SharedPtr response) {
    INFO("MachineActuator query state.");
    response->current_state = current_state_;
    response->code = static_cast<int32_t>(system::KeyCode::kOK);
  }

  /**
   * @brief 
   * 
   */
  void BuildStateMap() {
    state_map_.insert(std::make_pair(static_cast<uint8_t>(StateKey::kSetUp), std::bind(&MachineActuator::OnSetUp, this)));
    state_map_.insert(std::make_pair(static_cast<uint8_t>(StateKey::kTearDown), std::bind(&MachineActuator::ONTearDown, this)));
    state_map_.insert(std::make_pair(static_cast<uint8_t>(StateKey::kSelfCheck), std::bind(&MachineActuator::OnSelfCheck, this)));
    state_map_.insert(std::make_pair(static_cast<uint8_t>(StateKey::kActive), std::bind(&MachineActuator::OnActive, this)));
    state_map_.insert(std::make_pair(static_cast<uint8_t>(StateKey::kProtected), std::bind(&MachineActuator::OnProtected, this)));
    state_map_.insert(std::make_pair(static_cast<uint8_t>(StateKey::kLowPower), std::bind(&MachineActuator::OnLowPower, this)));
    state_map_.insert(std::make_pair(static_cast<uint8_t>(StateKey::kOTA), std::bind(&MachineActuator::OnOTA, this)));
  }

private:
  std::string name_;
  uint8_t current_state_{static_cast<uint8_t>(StateKey::kUninitialized)};
  std::map<uint16_t, std::function<int32_t(void)>> state_map_;
  rclcpp::Node::SharedPtr node_ptr_ {nullptr};
  rclcpp::Service<FSMACHINE_SRV_T>::SharedPtr machine_service_ptr_{nullptr};
};  // class MachineActuator
}  // namespace machine
}  // namespace cyberdog
#endif  // CYBERDOG_MACHINE__CYBERDOG_FS_MACHINE_HPP_
