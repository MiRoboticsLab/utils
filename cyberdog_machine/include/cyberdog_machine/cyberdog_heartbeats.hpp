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
#include "rclcpp/node.hpp"
#include "cyberdog_common/cyberdog_json.hpp"
#include "protocol/msg/heartbeats.hpp"

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

class HeartBeats
{
  using BeatsMap = std::map<std::string, std::shared_ptr<HeartClick>>;

public:
  HeartBeats(const std::string & name, int32_t duration)
  : name_(name), beats_duration_(duration)
  {}
  ~HeartBeats() {}

  void HeartConfig(const std::vector<std::string> & target_vec)
  {
    if (target_vec.empty()) {
      return;
    }
    for (auto iter = target_vec.begin(); iter != target_vec.end(); ++iter) {
      std::string tmp_name = std::string(*iter);
      auto tmp_ptr = std::make_shared<HeartClick>();
      beats_map_.insert(std::make_pair(tmp_name, tmp_ptr));
    }
  }

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

  bool HeartRegisterPublisher(std::function<void()> callback)
  {
    if (beats_map_.empty()) {
      return false;
    }
    if (callback == nullptr) {
      return false;
    } else {
      publish_callback_ = callback;
    }
    return true;
  }

  void HeartUpdate(const std::string & name)
  {
    auto iter = beats_map_.find(name);
    if (iter != beats_map_.end()) {
      std::cout << "receive once" << std::endl;
      iter->second->Push();
    }
  }

  void HeartRun()
    {
      if(lost_callback_ != nullptr) {
        auto t = std::thread(std::bind(&HeartBeats::BeatsCheck, this));
        t.detach();
      }
      if(publish_callback_ != nullptr) {
        auto t = std::thread(std::bind(&HeartBeats::BeatsPublish, this));
        t.detach();
      }
    }
private:
  void BeatsPublish()
  {
    while(true) {
      publish_callback_();
      std::this_thread::sleep_for(std::chrono::milliseconds(beats_duration_));
    }
  }

  void BeatsCheck()
  {
    while (true) {
      for (auto iter = beats_map_.begin(); iter != beats_map_.end(); ++iter) {
        if (!iter->second->Pop()) {
          std::string tmp(iter->first.c_str());
          lost_callback_(tmp);
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
  int32_t beats_duration_ {1000};  // timer with millisecond
  BeatsMap beats_map_;
};  // class HeartBeats
}  // namespace machine
}  // namespace cyberdog

#endif  // CYBERDOG_MACHINE__CYBERDOG_HEARTBEATS_HPP_
