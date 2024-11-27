/*
 * Copyright (c) 2019 Pilz GmbH & Co. KG
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <rclcpp/rclcpp.hpp>
#include <prbt_support/system_info.h>

using namespace prbt_support;

/**
 * @brief Logs important system information.
 */
// LCOV_EXCL_START
int main(int argc, char** argv)
{
  // Initialize the ROS 2 node
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("system_info");

  // Create the SystemInfo object with the ROS 2 node handle
  prbt_support::SystemInfo system_info(node);

  // Get firmware versions and log them
  FirmwareCont versions{ system_info.getFirmwareVersions() };
  for (const auto& curr_elem : versions)
  {
    RCLCPP_INFO(node->get_logger(), "Firmware version [%s]: %s", curr_elem.first.c_str(), curr_elem.second.c_str());
  }

  // Shutdown the ROS 2 node
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
// LCOV_EXCL_STOP
