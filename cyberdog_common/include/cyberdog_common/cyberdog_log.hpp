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
#ifndef CYBERDOG_COMMON__CYBERDOG_LOG_HPP_
#define CYBERDOG_COMMON__CYBERDOG_LOG_HPP_

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <string>
#include <cstring>
#include <memory>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

namespace cyberdog
{
namespace common
{
#define UNINITIALIZED_NAME "uninitialized"

class CyberdogLogger final
{
public:
  explicit CyberdogLogger(const char * name)
  : logger_name(name)
  {
    logger = logger_name.empty() ?
      std::make_shared<rclcpp::Logger>(rclcpp::get_logger(UNINITIALIZED_NAME)) :
      std::make_shared<rclcpp::Logger>(rclcpp::get_logger(logger_name));
  }
  ~CyberdogLogger() {}

  std::shared_ptr<rclcpp::Logger> Get_Logger()
  {
    return logger;
  }

private:
  std::string logger_name;
  std::shared_ptr<rclcpp::Logger> logger;
};  // class CyberdogLogger

class CyberdogLoggerFactory final
{
public:
  static std::shared_ptr<rclcpp::Logger> Get_Logger();
  static std::shared_ptr<rclcpp::Logger> Get_Logger(const char * sz_name);

private:
  static std::shared_ptr<rclcpp::Logger> main_logger;
};  // class CyberdogLoggerFactory

}  // namespace common
}  // namespace cyberdog

inline rclcpp::Logger get_logger()
{
  return *(cyberdog::common::CyberdogLoggerFactory::Get_Logger());
}

// grade={0:DEBUG/INFO, 1:WARN, 2:ERROR, 3:FATAL}
#define GET_MSG(msg, grade) \
  ({ \
    const char * result; \
    std::ostringstream nowmsg; \
    nowmsg << msg; \
    if (grade > 0) { \
      nowmsg << "\n\t○ The trigger of the \33[1m\33[3m" << __FUNCTION__ \
             << "()\33[23m\33[22m function in the \33[1m\33[4m" << __FILE__ \
             << "\33[24m\33[22m file of the source code is located at line \33[1m" \
             << __LINE__ << "\33[22m"; \
      if (grade > 1) { \
        const char * result; \
        char process_path[1024] = {0}; \
        if (readlink("/proc/self/exe", process_path, 1024) > 0) { \
          nowmsg << "\n\t◎ The \33[1m\33[4m" << process_path \
                 << "\33[24m\33[22m executable file compiled from \33[1m\33[4m" \
                 << __DATE__ << " - " << __TIME__ << "\33[24m\33[22m"; \
          if (grade > 2) { \
            char * process_name = strrchr(process_path, '/'); \
            if (process_name) { \
              pid_t parent_process_id = getppid(); \
              pid_t process_id = getpid(); \
              nowmsg << "\n\t● The \33[1m" << process_name \
                     << "\33[22m process captures a message with a process ID of \33[1m" \
                     << process_id << "\33[22m and a parent process ID of \33[1m" \
                     << parent_process_id << "\33[22m"; \
            } \
          } \
        } \
      } \
    } \
    result = strdup(nowmsg.str().c_str()); \
  })

#define LOGGER_MAIN_INSTANCE(instance_name) \
  std::shared_ptr<rclcpp::Logger> out_logger = \
    cyberdog::common::CyberdogLoggerFactory::Get_Logger(instance_name);
/*
#define LOGGER_INSTANCE(instance_name) \
        cyberdog::common::CyberdogLogger cyberdog_logger(instance_name); \
        std::shared_ptr<rclcpp::Logger> out_logger = cyberdog_logger.Get_Logger();
*/

// 格式化式:("xxx %s, %d", str, num)
#define DEBUG(...) RCLCPP_DEBUG(get_logger(), __VA_ARGS__)
#define INFO(...) RCLCPP_INFO(get_logger(), __VA_ARGS__)
#define WARN(...) RCLCPP_WARN(get_logger(), __VA_ARGS__)
#define ERROR(...) RCLCPP_ERROR(get_logger(), __VA_ARGS__)
#define FATAL(...) RCLCPP_FATAL(get_logger(), __VA_ARGS__)

#define DEBUG_ONCE(...) RCLCPP_DEBUG_ONCE(get_logger(), __VA_ARGS__)
#define INFO_ONCE(...) RCLCPP_INFO_ONCE(get_logger(), __VA_ARGS__)
#define WARN_ONCE(...) RCLCPP_WARN_ONCE(get_logger(), __VA_ARGS__)
#define ERROR_ONCE(...) RCLCPP_ERROR_ONCE(get_logger(), __VA_ARGS__)
#define FATAL_ONCE(...) RCLCPP_FATAL_ONCE(get_logger(), __VA_ARGS__)

#define DEBUG_FUNCTION(function, ...) \
  RCLCPP_DEBUG_FUNCTION(get_logger(), function, __VA_ARGS__)
#define INFO_FUNCTION(function, ...) \
  RCLCPP_INFO_FUNCTION(get_logger(), function, __VA_ARGS__)
#define WARN_FUNCTION(function, ...) \
  RCLCPP_WARN_FUNCTION(get_logger(), function, __VA_ARGS__)
#define ERROR_FUNCTION(function, ...) \
  RCLCPP_ERROR_FUNCTION(get_logger(), function, __VA_ARGS__)
#define FATAL_FUNCTION(function, ...) \
  RCLCPP_FATAL_FUNCTION(get_logger(), function, __VA_ARGS__)

#define DEBUG_EXPRESSION(expression, ...) \
  RCLCPP_DEBUG_EXPRESSION(get_logger(), expression, __VA_ARGS__)
#define INFO_EXPRESSION(expression, ...) \
  RCLCPP_INFO_EXPRESSION(get_logger(), expression, __VA_ARGS__)
#define WARN_EXPRESSION(expression, ...) \
  RCLCPP_WARN_EXPRESSION(get_logger(), expression, __VA_ARGS__)
#define ERROR_EXPRESSION(expression, ...) \
  RCLCPP_ERROR_EXPRESSION(get_logger(), expression, __VA_ARGS__)
#define FATAL_EXPRESSION(expression, ...) \
  RCLCPP_FATAL_EXPRESSION(get_logger(), expression, __VA_ARGS__)

// 流式:("xxx " << str << ", " << num)
#define DEBUG_STREAM(msg) RCLCPP_DEBUG(get_logger(), GET_MSG(msg, 0))
#define INFO_STREAM(msg) RCLCPP_INFO(get_logger(), GET_MSG(msg, 0))
#define WARN_STREAM(msg) RCLCPP_WARN(get_logger(), GET_MSG(msg, 1))
#define ERROR_STREAM(msg) RCLCPP_ERROR(get_logger(), GET_MSG(msg, 2))
#define FATAL_STREAM(msg) RCLCPP_FATAL(get_logger(), GET_MSG(msg, 3))

#define DEBUG_ONCE_STREAM(msg) RCLCPP_DEBUG_ONCE(get_logger(), GET_MSG(msg, 0))
#define INFO_ONCE_STREAM(msg) RCLCPP_INFO_ONCE(get_logger(), GET_MSG(msg, 0))
#define WARN_ONCE_STREAM(msg) RCLCPP_WARN_ONCE(get_logger(), GET_MSG(msg, 1))
#define ERROR_ONCE_STREAM(msg) RCLCPP_ERROR_ONCE(get_logger(), GET_MSG(msg, 2))
#define FATAL_ONCE_STREAM(msg) RCLCPP_FATAL_ONCE(get_logger(), GET_MSG(msg, 3))

#define DEBUG_FUNCTION_STREAM(function, msg) \
  RCLCPP_DEBUG_FUNCTION(get_logger(), function, GET_MSG(msg, 0))
#define INFO_FUNCTION_STREAM(function, msg) \
  RCLCPP_INFO_FUNCTION(get_logger(), function, GET_MSG(msg, 0))
#define WARN_FUNCTION_STREAM(function, msg) \
  RCLCPP_WARN_FUNCTION(get_logger(), function, GET_MSG(msg, 1))
#define ERROR_FUNCTION_STREAM(function, msg) \
  RCLCPP_ERROR_FUNCTION(get_logger(), function, GET_MSG(msg, 2))
#define FATAL_FUNCTION_STREAM(function, msg) \
  RCLCPP_FATAL_FUNCTION(get_logger(), function, GET_MSG(msg, 3))

#define DEBUG_EXPRESSION_STREAM(expression, msg) \
  RCLCPP_DEBUG_EXPRESSION(get_logger(), expression, GET_MSG(msg, 0))
#define INFO_EXPRESSION_STREAM(expression, msg) \
  RCLCPP_INFO_EXPRESSION(get_logger(), expression, GET_MSG(msg, 0))
#define WARN_EXPRESSION_STREAM(expression, msg) \
  RCLCPP_WARN_EXPRESSION(get_logger(), expression, GET_MSG(msg, 1))
#define ERROR_EXPRESSION_STREAM(expression, msg) \
  RCLCPP_ERROR_EXPRESSION(get_logger(), expression, GET_MSG(msg, 2))
#define FATAL_EXPRESSION_STREAM(expression, msg) \
  RCLCPP_FATAL_EXPRESSION(get_logger(), expression, GET_MSG(msg, 3))

#endif  // CYBERDOG_COMMON__CYBERDOG_LOG_HPP_
