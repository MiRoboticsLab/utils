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

#ifndef CYBERDOG_COMMON__CYBERDOG_LOG_LOGGER_HPP_
#define CYBERDOG_COMMON__CYBERDOG_LOG_LOGGER_HPP_

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
  CyberdogLogger(const char* name):logger_name(name) {
      logger = logger_name.empty() ?  
      std::make_shared<rclcpp::Logger>(rclcpp::get_logger(UNINITIALIZED_NAME)) 
      : 
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
}; // class CyberdogLogger

class CyberdogLoggerFactory final
{
public:
    static std::shared_ptr<rclcpp::Logger> Get_Logger();
    static std::shared_ptr<rclcpp::Logger> Get_Logger(const char* sz_name);
private:
    static std::shared_ptr<rclcpp::Logger> main_logger;
}; // class CyberdogLoggerFactory

} // namespace common
} // namespace cyberdog

#define LOGGER_MAIN_INSTANCE(instance_name) \
        std::shared_ptr<rclcpp::Logger> out_logger = cyberdog::common::CyberdogLoggerFactory::Get_Logger(instance_name);

#define DEBUG(...) RCLCPP_DEBUG((*(cyberdog::common::CyberdogLoggerFactory::Get_Logger())), __VA_ARGS__)
#define INFO(...) RCLCPP_INFO((*(cyberdog::common::CyberdogLoggerFactory::Get_Logger())), __VA_ARGS__)
#define WARN(...) RCLCPP_WARN((*(cyberdog::common::CyberdogLoggerFactory::Get_Logger())), __VA_ARGS__)
#define ERROR(...) RCLCPP_ERROR((*(cyberdog::common::CyberdogLoggerFactory::Get_Logger())), __VA_ARGS__)
#define FATAL(...) RCLCPP_FATAL((*(cyberdog::common::CyberdogLoggerFactory::Get_Logger())), __VA_ARGS__)

#define N_DEBUG(...) RCLCPP_DEBUG(this->get_logger(), __VA_ARGS__)
#define N_INFO(...) RCLCPP_INFO(this->get_logger(), __VA_ARGS__)
#define N_WARN(...) RCLCPP_WARN(this->get_logger(), __VA_ARGS__)
#define N_ERROR(...) RCLCPP_ERROR(this->get_logger(), __VA_ARGS__)
#define N_FATAL(...) RCLCPP_FATAL(this->get_logger(), __VA_ARGS__)

#define DEBUG_ONCE(...) RCLCPP_DEBUG_ONCE((*(cyberdog::common::CyberdogLoggerFactory::Get_Logger())), __VA_ARGS__)
#define INFO_ONCE(...) RCLCPP_INFO_ONCE((*(cyberdog::common::CyberdogLoggerFactory::Get_Logger())), __VA_ARGS__)
#define WARN_ONCE(...) RCLCPP_WARN_ONCE((*(cyberdog::common::CyberdogLoggerFactory::Get_Logger())), __VA_ARGS__)
#define ERROR_ONCE(...) RCLCPP_ERROR_ONCE((*(cyberdog::common::CyberdogLoggerFactory::Get_Logger())), __VA_ARGS__)
#define FATAL_ONCE(...) RCLCPP_FATAL_ONCE((*(cyberdog::common::CyberdogLoggerFactory::Get_Logger())), __VA_ARGS__)

#define N_DEBUG_ONCE(...) RCLCPP_DEBUG_ONCE(this->get_logger(), __VA_ARGS__)
#define N_INFO_ONCE(...) RCLCPP_INFO_ONCE(this->get_logger(), __VA_ARGS__)
#define N_WARN_ONCE(...) RCLCPP_WARN_ONCE(this->get_logger(), __VA_ARGS__)
#define N_ERROR_ONCE(...) RCLCPP_ERROR_ONCE(this->get_logger(), __VA_ARGS__)
#define N_FATAL_ONCE(...) RCLCPP_FATAL_ONCE(this->get_logger(), __VA_ARGS__)

#define DEBUG_FUNCTION(function, ...) RCLCPP_DEBUG_FUNCTION((*(cyberdog::common::CyberdogLoggerFactory::Get_Logger())), function, __VA_ARGS__)
#define INFO_FUNCTION(function, ...) RCLCPP_INFO_FUNCTION((*(cyberdog::common::CyberdogLoggerFactory::Get_Logger())), function, __VA_ARGS__)
#define WARN_FUNCTION(function, ...) RCLCPP_WARN_FUNCTION((*(cyberdog::common::CyberdogLoggerFactory::Get_Logger())), function, __VA_ARGS__)
#define ERROR_FUNCTION(function, ...) RCLCPP_ERROR_FUNCTION((*(cyberdog::common::CyberdogLoggerFactory::Get_Logger())), function, __VA_ARGS__)
#define FATAL_FUNCTION(function, ...) RCLCPP_FATAL_FUNCTION((*(cyberdog::common::CyberdogLoggerFactory::Get_Logger())), function, __VA_ARGS__)

#define N_DEBUG_FUNCTION(function, ...) RCLCPP_DEBUG_FUNCTION(this->get_logger(), function, __VA_ARGS__)
#define N_INFO_FUNCTION(function, ...) RCLCPP_INFO_FUNCTION(this->get_logger(), function, __VA_ARGS__)
#define N_WARN_FUNCTION(function, ...) RCLCPP_WARN_FUNCTION(this->get_logger(), function, __VA_ARGS__)
#define N_ERROR_FUNCTION(function, ...) RCLCPP_ERROR_FUNCTION(this->get_logger(), function, __VA_ARGS__)
#define N_FATAL_FUNCTION(function, ...) RCLCPP_FATAL_FUNCTION(this->get_logger(), function, __VA_ARGS__)

#define DEBUG_EXPRESSION(expression, ...) RCLCPP_DEBUG_EXPRESSION((*(cyberdog::common::CyberdogLoggerFactory::Get_Logger())), expression, __VA_ARGS__)
#define INFO_EXPRESSION(expression, ...) RCLCPP_INFO_EXPRESSION((*(cyberdog::common::CyberdogLoggerFactory::Get_Logger())), expression, __VA_ARGS__)
#define WARN_EXPRESSION(expression, ...) RCLCPP_WARN_EXPRESSION((*(cyberdog::common::CyberdogLoggerFactory::Get_Logger())), expression, __VA_ARGS__)
#define ERROR_EXPRESSION(expression, ...) RCLCPP_ERROR_EXPRESSION((*(cyberdog::common::CyberdogLoggerFactory::Get_Logger())), expression, __VA_ARGS__)
#define FATAL_EXPRESSION(expression, ...) RCLCPP_FATAL_EXPRESSION((*(cyberdog::common::CyberdogLoggerFactory::Get_Logger())), expression, __VA_ARGS__)

#define N_DEBUG_EXPRESSION(expression, ...) RCLCPP_DEBUG_EXPRESSION(this->get_logger(), expression, __VA_ARGS__)
#define N_INFO_EXPRESSION(expression, ...) RCLCPP_INFO_EXPRESSION(this->get_logger(), expression, __VA_ARGS__)
#define N_WARN_EXPRESSION(expression, ...) RCLCPP_WARN_EXPRESSION(this->get_logger(), expression, __VA_ARGS__)
#define N_ERROR_EXPRESSION(expression, ...) RCLCPP_ERROR_EXPRESSION(this->get_logger(), expression, __VA_ARGS__)
#define N_FATAL_EXPRESSION(expression, ...) RCLCPP_FATAL_EXPRESSION(this->get_logger(), expression, __VA_ARGS__)

/*
#define LOGGER_INSTANCE(instance_name) \
        cyberdog::common::CyberdogLogger cyberdog_logger(instance_name); \
        std::shared_ptr<rclcpp::Logger> out_logger = cyberdog_logger.Get_Logger();
*/

#endif