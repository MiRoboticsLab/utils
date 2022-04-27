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

#ifndef CYBERDOG_COMMON__CYBERDOG_MSG_QUEUE_HPP_
#define CYBERDOG_COMMON__CYBERDOG_MSG_QUEUE_HPP_

#include <list>
#include <mutex>
#include <condition_variable>

namespace cyberdog
{
namespace common
{
template<typename T>
class MsgQueue
{
public:
  MsgQueue()
  {
    SetWait(false);
  }

  ~MsgQueue()
  {
    Clear();
    if (IsWait()) {
      read_signal_.notify_all();
    }
  }

/* Msg Queue opened API */

public:
  void EnQueue(const T & t)
  {
    std::unique_lock<std::mutex> lk(data_lock_);
    data_list_.emplace_front(t);
    if (IsWait()) {
      SetWait(false);
      read_signal_.notify_one();
    }
  }

  void EnQueueOne(const T & t)
  {
    std::unique_lock<std::mutex> lk(data_lock_);
    if (!IsEmpty()) {
      Clear();
    } else {
      data_list_.emplace_front(t);
      if (IsWait()) {
        SetWait(false);
        read_signal_.notify_one();
      }
    }
  }

  bool DeQueue(T & t)
  {
    std::unique_lock<std::mutex> lk(data_lock_);
    if (data_list_.empty()) {
      SetWait(true);
      read_signal_.wait(lk);
    }
    if (IsEmpty()) {
      return false;
    } else {
      t = data_list_.back();
      data_list_.pop_back();
      return true;
    }
  }

  void Reset()
  {
    Clear();
    if (IsWait()) {
      SetWait(false);
      read_signal_.notify_all();
    }
  }

  int Size()
  {
    std::unique_lock<std::mutex> lk(data_lock_);
    return static_cast<int>(data_list_.size());
  }

  bool IsEmpty()
  {
    std::unique_lock<std::mutex> lk(data_lock_);
    return data_list_.empty();
  }

/* Internal API */

private:
  bool IsWait()
  {
    return is_wait_;
  }

  void SetWait(bool wait_flag)
  {
    is_wait_ = wait_flag;
  }

  void Clear()
  {
    std::unique_lock<std::mutex> lk(data_lock_);
    while (!data_list_.empty()) {
      data_list_.pop_front();
    }
  }

private:
  std::condition_variable read_signal_;
  std::mutex data_lock_;
  std::list<T> data_list_;
  bool is_wait_;
};  // class MsgQueue
}  // namespace common
}  // namespace cyberdog

#endif  // CYBERDOG_COMMON__CYBERDOG_MSG_QUEUE_HPP_
