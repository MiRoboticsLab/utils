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

namespace cyberdog
{
namespace machine
{
class BeatsCounter final
{
  
};  // class BeatsDectector


class HeartBeatsBase
{
    using BeatsMap = std::map<std::string, int32_t>;

public:
    explicit HeartBeatsBase(const std::string & name)
        : name_(name)
    {}
    ~HeartBeatsBase(){}

    void HeartConfig(bool publisher, bool listener, int32_t gap) {
        is_publisher_ = publisher;
        is_listener_ = listener;
        beats_gap_ = gap;
    }

    bool HeartInit(const BeatsMap & beats_map) {
        if(beats_map.empty())
            return false;
        beats_map_ = beats_map;
        return true;
    }

    void HeartActivate() {
        SetBeatStatus(true);
    } 

    bool HeartCheck() {
      return true;
    }

    void HeartSuspend() {

        SetBeatStatus(false);
    }

    virtual void BeatsPublish() = 0;

private:
    void SetBeatStatus(bool state) {
        std::unique_lock<std::mutex> lk(beats_mutex_);
        if(! is_active_) {
            is_active_ = state;
            beats_cv_.notify_one();
        } else {
            is_active_ = state;
        }
    }

    bool GetBeatsStatus() {
        return is_active_;
    }

    void CheckBeatsStatus() {
        if(! GetBeatsStatus()) {
            std::unique_lock<std::mutex> lk(beats_mutex_);
            beats_cv_.wait(lk);
        }
    }

    void BeatsRunner() {
        while (true)
        {
            CheckBeatsStatus();
            // BeatsPublish();
            std::this_thread::sleep_for(std::chrono::milliseconds(beats_gap_));
        }
    }

private:
    std::string name_;
    bool    is_listener_ {false};
    bool    is_publisher_ {false};
    int32_t beats_gap_ {1000};  // timer with millisecond
    std::atomic_bool    is_active_ {false};
    std::thread beats_thread_;
    std::mutex beats_mutex_;
    std::condition_variable beats_cv_;
    BeatsMap beats_map_;
};  // class HeartBeats

// class HeartListener : public HeartBeatsBase
// {
// public:
//     HeartListener(const std::string & name) {}
//     ~HeartListener() {}
// };  // class HeartListener

// class HeartTalker : public HeartBeatsBase
// {};  // class HeartTalker

// class HeartBonder
// {};  // class HeartBonder

}  // namespace machine
}  // namespace cyberdog

#endif  // CYBERDOG_MACHINE__CYBERDOG_HEARTBEATS_HPP_

