#include "cyberdog_common/cyberdog_log.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "cyberdog_common/srv/config_logger.hpp"

// namespace cyberdog
// {
// namespace common
// {
// class LoggerConfig : public rclcpp::Node
// {
// public:
//     LoggerConfig(std::string name) : Node(name)
//     {
//         srv_ = create_service<cyberdog_log::srv::ConfigLogger>(
//             name + "_config", std::bind(
//             &LoggerConfig::handle_logger_config_request,
//             this, std::placeholders::_1, std::placeholders::_2));
//     }

//     void
//     handle_logger_config_request(
//         const std::shared_ptr<cyberdog_log::srv::ConfigLogger::Request> request,
//         std::shared_ptr<cyberdog_log::srv::ConfigLogger::Response> response)
//     {
//         const char * severity_string = request->level.c_str();
//         N_INFO(
//             "Incoming request: logger '%s', severity '%s'",
//             request->logger_name.c_str(), severity_string);

//         int severity;
//         rcutils_ret_t ret = rcutils_logging_severity_level_from_string(
//             severity_string, rcl_get_default_allocator(), &severity);
//         if (RCUTILS_RET_LOGGING_SEVERITY_STRING_INVALID == ret) {
//             N_ERROR(
//             "Unknown severity '%s'", severity_string);
//             response->success = false;
//             return;
//         }
//         if (RCUTILS_RET_OK != ret) {
//             N_ERROR(
//             "Error %d getting severity level from request: %s", ret,
//             rcl_get_error_string().str);
//             rcl_reset_error();
//             response->success = false;
//             return;
//         }

//         // TODO(dhood): allow configuration through rclcpp
//         ret = rcutils_logging_set_logger_level(request->logger_name.c_str(), severity);
//         if (ret != RCUTILS_RET_OK) {
//             N_ERROR(
//             "Error setting severity: %s", rcutils_get_error_string().str);
//             rcutils_reset_error();
//             response->success = false;
//         }
//         response->success = true;
//     }

// private:
//     rclcpp::Service<cyberdog_log::srv::ConfigLogger>::SharedPtr srv_;
// };
// } // namespace common
// } // namespace cyberdog

using namespace cyberdog::common;

std::shared_ptr<rclcpp::Logger> CyberdogLoggerFactory::main_logger = nullptr;

std::shared_ptr<rclcpp::Logger> CyberdogLoggerFactory::Get_Logger()
{
  return main_logger ==
         nullptr ? std::make_shared<rclcpp::Logger>(rclcpp::get_logger(UNINITIALIZED_NAME)) :
         main_logger;
}

std::shared_ptr<rclcpp::Logger> CyberdogLoggerFactory::Get_Logger(const char * sz_name)
{
  if (!main_logger) {
    CyberdogLogger cyberdog_logger(sz_name);
    main_logger = cyberdog_logger.Get_Logger();
    // std::string config_name = std::string(sz_name)+"_severity";
    // static auto logger_config = std::make_shared<LoggerConfig>(config_name);
    // rclcpp::spin(logger_config);
  }
  return main_logger;
}
