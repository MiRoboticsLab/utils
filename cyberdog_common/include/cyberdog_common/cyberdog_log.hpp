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

#include <memory>
#include <string>
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
      std::make_shared<rclcpp::Logger>(rclcpp::get_logger(UNINITIALIZED_NAME))
      :
      std::make_shared<rclcpp::Logger>(rclcpp::get_logger(logger_name));
  }
  CyberdogLogger(const CyberdogLogger &) = delete;
  CyberdogLogger & operator=(const CyberdogLogger &) = delete;

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

#define LOGGER_MINOR_INSTANCE(instance_name) \
  std::shared_ptr<rclcpp::Logger> logger; \
  inline rclcpp::Logger get_logger() { \
    if (!logger) { \
      std::shared_ptr<rclcpp::Logger> out_logger = \
        cyberdog::common::CyberdogLoggerFactory::Get_Logger(); \
      std::string str_name = std::string(out_logger->get_name()) + " " + instance_name; \
      cyberdog::common::CyberdogLogger cyberdog_logger(str_name.c_str()); \
      logger = cyberdog_logger.Get_Logger(); \
    } \
    return *logger; \
  }


#define LOGGER_MAIN_INSTANCE(instance_name) \
  std::shared_ptr<rclcpp::Logger> out_logger = \
    cyberdog::common::CyberdogLoggerFactory::Get_Logger( \
    instance_name);

#define DEBUG(...) RCLCPP_DEBUG(get_logger(), __VA_ARGS__)
#define INFO(...) RCLCPP_INFO(get_logger(), __VA_ARGS__)
#define WARN(...) RCLCPP_WARN(get_logger(), __VA_ARGS__)
#define ERROR(...) RCLCPP_ERROR(get_logger(), __VA_ARGS__)
#define FATAL(...) RCLCPP_FATAL(get_logger(), __VA_ARGS__)
#define DEBUG_STREAM(args) std::stringstream ss; ss << args; DEBUG("%s", ss.str().c_str())
#define INFO_STREAM(args) std::stringstream ss; ss << args; INFO("%s", ss.str().c_str())
#define WARN_STREAM(args) std::stringstream ss; ss << args; WARN("%s", ss.str().c_str())
#define ERROR_STREAM(args) std::stringstream ss; ss << args; ERROR("%s", ss.str().c_str())
#define FATAL_STREAM(args) std::stringstream ss; ss << args; FATAL("%s", ss.str().c_str())

#define DEBUG_ONCE(...) RCLCPP_DEBUG_ONCE(get_logger(), __VA_ARGS__)
#define INFO_ONCE(...) RCLCPP_INFO_ONCE(get_logger(), __VA_ARGS__)
#define WARN_ONCE(...) RCLCPP_WARN_ONCE(get_logger(), __VA_ARGS__)
#define ERROR_ONCE(...) RCLCPP_ERROR_ONCE(get_logger(), __VA_ARGS__)
#define FATAL_ONCE(...) RCLCPP_FATAL_ONCE(get_logger(), __VA_ARGS__)
#define DEBUG_STREAM_ONCE(args) std::stringstream ss; ss << args; DEBUG_ONCE("%s", ss.str().c_str())
#define INFO_STREAM_ONCE(args) std::stringstream ss; ss << args; INFO_ONCE("%s", ss.str().c_str())
#define WARN_STREAM_ONCE(args) std::stringstream ss; ss << args; WARN_ONCE("%s", ss.str().c_str())
#define ERROR_STREAM_ONCE(args) std::stringstream ss; ss << args; ERROR_ONCE("%s", ss.str().c_str())
#define FATAL_STREAM_ONCE(args) std::stringstream ss; ss << args; FATAL_ONCE("%s", ss.str().c_str())

#define DEBUG_FUNCTION(function, ...) RCLCPP_DEBUG_FUNCTION(get_logger(), function, __VA_ARGS__)
#define INFO_FUNCTION(function, ...) RCLCPP_INFO_FUNCTION(get_logger(), function, __VA_ARGS__)
#define WARN_FUNCTION(function, ...) RCLCPP_WARN_FUNCTION(get_logger(), function, __VA_ARGS__)
#define ERROR_FUNCTION(function, ...) RCLCPP_ERROR_FUNCTION(get_logger(), function, __VA_ARGS__)
#define FATAL_FUNCTION(function, ...) RCLCPP_FATAL_FUNCTION(get_logger(), function, __VA_ARGS__)
#define DEBUG_STREAM_FUNCTION(function, args) std::stringstream ss; ss << args; DEBUG_FUNCTION( \
    function, "%s", ss.str().c_str())
#define INFO_STREAM_FUNCTION(function, args) std::stringstream ss; ss << args; INFO_FUNCTION( \
    function, "%s", ss.str().c_str())
#define WARN_STREAM_FUNCTION(function, args) std::stringstream ss; ss << args; WARN_FUNCTION( \
    function, "%s", ss.str().c_str())
#define ERROR_STREAM_FUNCTION(function, args) std::stringstream ss; ss << args; ERROR_FUNCTION( \
    function, "%s", ss.str().c_str())
#define FATAL_STREAM_FUNCTION(function, args) std::stringstream ss; ss << args; FATAL_FUNCTION( \
    function, "%s", ss.str().c_str())

#define DEBUG_EXPRESSION(expression, ...) RCLCPP_DEBUG_EXPRESSION( \
    get_logger(), expression, __VA_ARGS__)
#define INFO_EXPRESSION(expression, ...) RCLCPP_INFO_EXPRESSION( \
    get_logger(), expression, __VA_ARGS__)
#define WARN_EXPRESSION(expression, ...) RCLCPP_WARN_EXPRESSION( \
    get_logger(), expression, __VA_ARGS__)
#define ERROR_EXPRESSION(expression, ...) RCLCPP_ERROR_EXPRESSION( \
    get_logger(), expression, __VA_ARGS__)
#define FATAL_EXPRESSION(expression, ...) RCLCPP_FATAL_EXPRESSION( \
    get_logger(), expression, __VA_ARGS__)
#define DEBUG_STREAM_EXPRESSION(expression, args) std::stringstream ss; ss << args; \
  DEBUG_EXPRESSION(expression, "%s", ss.str().c_str())
#define INFO_STREAM_EXPRESSION(expression, args) std::stringstream ss; ss << args; \
  INFO_EXPRESSION(expression, "%s", ss.str().c_str())
#define WARN_STREAM_EXPRESSION(expression, args) std::stringstream ss; ss << args; \
  WARN_EXPRESSION(expression, "%s", ss.str().c_str())
#define ERROR_STREAM_EXPRESSION(expression, args) std::stringstream ss; ss << args; \
  ERROR_EXPRESSION(expression, "%s", ss.str().c_str())
#define FATAL_STREAM_EXPRESSION(expression, args) std::stringstream ss; ss << args; \
  FATAL_EXPRESSION(expression, "%s", ss.str().c_str())

/*
#define LOGGER_INSTANCE(instance_name) \
        cyberdog::common::CyberdogLogger cyberdog_logger(instance_name); \
        std::shared_ptr<rclcpp::Logger> out_logger = cyberdog_logger.Get_Logger();
*/

#endif  // CYBERDOG_COMMON__CYBERDOG_LOG_HPP_
