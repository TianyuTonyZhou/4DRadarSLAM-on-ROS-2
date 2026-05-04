// This file is part of RIO - Radar Inertial Odometry and Radar ego velocity estimation.
// Copyright (C) 2021  Christopher Doer <christopher.doer@kit.edu>

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
//(at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sstream>

namespace rio
{
enum class RosParameterType
{
  Required,
  Recommended,
  Optional
};

template <typename T>
static bool getRosParameter(rclcpp::Node* node,
                            const std::string kPrefix,
                            const RosParameterType& param_type,
                            const std::string& param_name,
                            T& param)
{
  // In ROS 2, parameters must first be declared before they can be read.
  // declare_parameter returns the value — either the one set externally,
  // or the default if no external value was provided.
  try {
    param = node->declare_parameter<T>(param_name, param);

    if (!node->has_parameter(param_name)) {
      // should never happen after declare_parameter, but guard anyway
      if (param_type == RosParameterType::Required) {
        RCLCPP_ERROR_STREAM(node->get_logger(),
          kPrefix << "<" << param_name << "> is required but not configured. Exiting!");
        return false;
      }
    }
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&) {
    // already declared elsewhere — just read it
    param = node->get_parameter(param_name).get_value<T>();
  }

  return true;
}

}  // namespace rio