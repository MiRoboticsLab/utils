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
#ifndef CYBERDOG_MACHINE__CYBERDOG_HEARTBEATS_HPP_
#define CYBERDOG_MACHINE__CYBERDOG_HEARTBEATS_HPP_
#include <string>
#include <vector>
#include <memory>
#include <map>
#include <utility>
#include "rclcpp/node.hpp"
#include "cyberdog_common/cyberdog_json.hpp"

namespace cyberdog
{
namespace machine
{
struct HeartClick
{
  void Push()
  {
    data_ = true;
  }

  bool Pop()
  {
    if (data_ == false) {
      return false;
    } else {
      data_ = false;
    }
    return true;
  }

  std::atomic_bool data_ {false};
};  // struct HeartQueue

/**
 * @brief 心跳类，具体参数及定义请参考心跳设计文档：
 *        https://xiaomi.f.mioffice.cn/docs/dock43ed3sAGiJmwgnPFIm7I5Kf
 */
class HeartBeats
{
  using BeatsMap = std::map<std::string, std::shared_ptr<HeartClick>>;
  using BeatsCounterMap = std::map<std::string, int8_t>;

public:
  /**
   * @brief Construct a new Heart Beats object
   *
   * @param name 模块名字，具备全局唯一性
   * @param duration 心跳发送、检测间隔，
   * @param lost_limit 丢失/超时 限制，达到该次数会触发心跳异常
   */
  HeartBeats(const std::string & name, int32_t duration, int8_t lost_limit)
  : name_(name), beats_duration_(duration), lost_limit_(lost_limit)
  {}
  ~HeartBeats() {}

  /**
   * @brief 配置心跳监听对象，单一发送模块不需要配置
   *
   * @param target_vec 监听对象name容器
   */
  void HeartConfig(const std::vector<std::string> & target_vec)
  {
    if (target_vec.empty()) {
      return;
    }
    for (auto iter = target_vec.begin(); iter != target_vec.end(); ++iter) {
      std::string tmp_name = std::string(*iter);
      auto tmp_ptr = std::make_shared<HeartClick>();
      beats_map_.insert(std::make_pair(tmp_name, tmp_ptr));
      beats_counter_map_.insert(std::make_pair(tmp_name, 0));
    }
  }

  /**
   * @brief 注册监听异常回调函数
   *
   * @param callback 回调函数以引用形式得到异常模块的名字
   * @return true
   * @return false
   */
  bool HeartRegisterListener(std::function<void(const std::string &)> callback)
  {
    if (beats_map_.empty()) {
      return false;
    }
    if (callback == nullptr) {
      return false;
    } else {
      lost_callback_ = callback;
    }
    return true;
  }

  /**
   * @brief 注册发送回调函数
   *
   * @param callback 回调函数以void(void)形式引用，即允许不同的通信方式发送心跳数据
   * @return true
   * @return false
   */
  bool HeartRegisterPublisher(std::function<void()> callback)
  {
    // if (beats_map_.empty()) {
    //   return false;
    // }
    if (callback == nullptr) {
      return false;
    } else {
      publish_callback_ = callback;
    }
    return true;
  }

  /**
   * @brief 心跳更新函数，监听方以ros或lcm、grpc等监听心跳，将name信息更新至该函数用于心跳刷新
   *
   * @param name
   */
  void HeartUpdate(const std::string & name)
  {
    auto iter = beats_map_.find(name);
    if (iter != beats_map_.end()) {
      std::cout << "receive once" << std::endl;
      iter->second->Push();
    }
  }

  /**
   * @brief 心跳启动函数，要求必须执行
   *
   */
  void HeartRun()
  {
    if (lost_callback_ != nullptr) {
      std::cout << "heartrun 1\n";
      auto t = std::thread(std::bind(&HeartBeats::BeatsCheck, this));
      t.detach();
    }
    if (publish_callback_ != nullptr) {
      std::cout << "heartrun 2\n";
      auto t = std::thread(std::bind(&HeartBeats::BeatsPublish, this));
      t.detach();
    }
  }

private:
  void BeatsPublish()
  {
    while (true) {
      publish_callback_();
      std::this_thread::sleep_for(std::chrono::milliseconds(beats_duration_));
    }
  }

  void BeatsCheck()
  {
    while (true) {
      for (auto iter = beats_map_.begin(); iter != beats_map_.end(); ++iter) {
        std::string tmp_name(iter->first.c_str());
        auto counter_iter = beats_counter_map_.find(tmp_name);
        if (!iter->second->Pop()) {
          if (++counter_iter->second >= lost_limit_) {
            lost_callback_(tmp_name);
          } else {
            continue;
          }
        } else {
          counter_iter->second = 0;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(beats_duration_));
    }
  }

private:
  std::function<void(const std::string &)> lost_callback_;
  std::function<void()> publish_callback_;

private:
  std::string name_;
  int32_t beats_duration_ {1000};       // 心跳间隔
  int8_t lost_limit_ {5};               // 连续丢失容忍上限
  BeatsMap beats_map_;                  // 心跳刷新机制实现map
  BeatsCounterMap beats_counter_map_;   // 心跳异常检测map
};  // class HeartBeats
}  // namespace machine
}  // namespace cyberdog

#endif  // CYBERDOG_MACHINE__CYBERDOG_HEARTBEATS_HPP_
