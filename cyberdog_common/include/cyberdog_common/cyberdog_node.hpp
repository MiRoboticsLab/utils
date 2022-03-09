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

#ifndef CYBERDOG_COMMON__CYBERDOG_NODE_CYBERDOGNODE_HPP_
#define CYBERDOG_COMMON__CYBERDOG_NODE_CYBERDOGNODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "cyberdog_log.hpp"

namespace cyberdog
{
namespace common
{

using rclcpp::Node;
using rclcpp::NodeOptions;

class CyberdogNode : public Node
{
public:
  RCLCPP_PUBLIC
  explicit CyberdogNode(
    const std::string & node_name,
    const NodeOptions & options = NodeOptions())
  : Node(node_name, options)
  {
    init_service(node_name);
  }

  RCLCPP_PUBLIC
  explicit CyberdogNode(
    const std::string & node_name,
    const std::string & namespace_,
    const NodeOptions & options = NodeOptions())
  : Node(node_name, namespace_, options)
  {
    init_service(namespace_ + "/" + node_name);
  }

private:
  void init_service(std::string node_full_name)
  {
    srv_ = create_service<std_srvs::srv::SetBool>(
      node_full_name + "/set_severity", std::bind(
        &CyberdogNode::handle_logger_config_request,
        this, std::placeholders::_1, std::placeholders::_2));
  }
  void handle_logger_config_request(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    std::map<uint8_t, const char *> severity_map = {
      {0, "debug"},
      {1, "info"},
      {2, "warn"},
      {3, "error"},
      {4, "fatal"}
    };
    uint8_t request_level = request->data == true ? 1 : 0;
    const char * severity_string = severity_map[request_level];
    int severity;
    response->message = "unkown";
    rcutils_ret_t ret = rcutils_logging_severity_level_from_string(
      severity_string, rcl_get_default_allocator(), &severity);
    if (RCUTILS_RET_LOGGING_SEVERITY_STRING_INVALID == ret) {
      ERROR(
        "Unknown severity '%s'", severity_string);
      response->success = false;
      return;
    }
    if (RCUTILS_RET_OK != ret) {
      ERROR(
        "Error %d getting severity level from request: %s", ret,
        rcl_get_error_string().str);
      rcl_reset_error();
      response->success = false;
      return;
    }
    // TODO(dhood): allow configuration through rclcpp
    ret = rcutils_logging_set_logger_level(get_name(), severity);
    if (ret != RCUTILS_RET_OK) {
      ERROR(
        "Error setting severity: %s", rcutils_get_error_string().str);
      rcutils_reset_error();
      response->success = false;
    }
    response->success = true;
    response->message = severity_string;
  }

private:
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_;
};

} // namespace common
} // namespace cyberdog

#endif
