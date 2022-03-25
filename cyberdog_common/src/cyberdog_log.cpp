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

#include <memory>
#include "cyberdog_common/cyberdog_log.hpp"


using cyberdog::common::CyberdogLogger;
using cyberdog::common::CyberdogLoggerFactory;

std::shared_ptr<rclcpp::Logger> CyberdogLoggerFactory::main_logger = nullptr;

std::shared_ptr<rclcpp::Logger> CyberdogLoggerFactory::Get_Logger()
{
  return main_logger ==
         nullptr ? std::make_shared<rclcpp::Logger>(rclcpp::get_logger(UNINITIALIZED_NAME)) :
         main_logger;
}

static void handler(int signum, siginfo_t * info, void * context)
{
  (void)context;
  if(signum == SIGINT)
    INFO("SIGINT signal: %d,content:(%d %d)",
      signum, info->si_int, info->si_value.sival_int);
  else if(signum == SIGTSTP)
    INFO("SIGTSTP signal: %d,content:(%d %d)",
      signum, info->si_int, info->si_value.sival_int);
  else if(signum == SIGQUIT)
    INFO("SIGQUIT signal: %d,content:(%d %d)",
    signum, info->si_int, info->si_value.sival_int);
  else if(signum == SIGFPE)
    INFO("SIGFPE signal: %d,content:(%d %d)",
    signum, info->si_int, info->si_value.sival_int);
  exit(0);
}

std::shared_ptr<rclcpp::Logger> CyberdogLoggerFactory::Get_Logger(const char * sz_name)
{
  if (!main_logger) {
    struct sigaction act;
    act.sa_sigaction = handler;
    act.sa_flags = SA_SIGINFO;
    sigaction(SIGINT, &act, NULL);
    sigaction(SIGTSTP, &act, NULL);
    sigaction(SIGQUIT, &act, NULL);
    sigaction(SIGFPE, &act, NULL);
    CyberdogLogger cyberdog_logger(sz_name);
    main_logger = cyberdog_logger.Get_Logger();
  }
  return main_logger;
}
