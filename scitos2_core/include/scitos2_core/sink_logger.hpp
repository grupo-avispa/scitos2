// Copyright (c) 2024 Alberto J. Tudela Roldán
// Copyright (c) 2024 Grupo Avispa, DTE, Universidad de Málaga
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

#ifndef SCITOS2_CORE__SINK_LOGGER_HPP_
#define SCITOS2_CORE__SINK_LOGGER_HPP_

#include <error/LoggingCore.h>

#include "rclcpp/rclcpp.hpp"

namespace scitos2_core
{

/**
 * @class scitos2_core::SinkLogger
 * @brief Class for MIRA log sinks. Redirect Mira logging to RCLCPP logging.
 */

class SinkLogger : public mira::LogSink
{
public:
  /**
   * @brief Construct a new Sink Logger object
   *
   * @param logger RCLCPP logger to redirect MIRA logging.
   */
  explicit SinkLogger(rclcpp::Logger logger)
  : logger_(logger) {}

  /**
   * @brief Consume a log record and redirect it to RCLCPP logging.
   *
   * @param record Log record to consume.
   */
  void consume(const mira::LogRecord & record)
  {
    switch (record.level) {
      case mira::SeverityLevel::CRITICAL:
      case mira::SeverityLevel::ERROR:
        RCLCPP_ERROR_STREAM(logger_, record.message);
        break;
      case mira::SeverityLevel::WARNING:
        RCLCPP_WARN_STREAM(logger_, record.message);
        break;
      case mira::SeverityLevel::NOTICE:
        RCLCPP_INFO_STREAM(logger_, record.message);
        break;
      case mira::SeverityLevel::DEBUG:
        RCLCPP_DEBUG_STREAM(logger_, record.message);
        break;
      default:
        break;
    }
  }

private:
  rclcpp::Logger logger_;
};

}  // namespace scitos2_core

#endif  // SCITOS2_CORE__SINK_LOGGER_HPP_
