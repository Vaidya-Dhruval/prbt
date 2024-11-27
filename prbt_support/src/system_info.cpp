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

#include <prbt_support/system_info.h>

#include <XmlRpcValue.h>
#include <XmlRpcException.h>
#include <sensor_msgs/msg/joint_state.hpp>

#include <pilz_utils/wait_for_service.hpp>
#include <pilz_utils/wait_for_message.hpp>
#include <canopen_chain_node/srv/get_object.hpp>
#include <prbt_support/system_info_exception.hpp>

namespace prbt_support
{
static const std::string CANOPEN_GETOBJECT_SERVICE_NAME{ "/prbt/driver/get_object" };
static const std::string CANOPEN_NODES_PARAMETER_NAME{ "/prbt/driver/nodes" };
static const std::string JOINT_STATE_TOPIC{ "/joint_states" };

static const std::string GET_FIRMWARE_VERSION_OBJECT{ "100A" };

// Currently the string is defined to be 41 characters long, but the last character can be omitted.
// This is currently under investigation. See https://github.com/PilzDE/pilz_robots/issues/299.
static constexpr std::size_t FIRMWARE_STRING_LENGTH{ 40 };

SystemInfo::SystemInfo(std::shared_ptr<rclcpp::Node> node) : node_(node), joint_names_(getNodeNames())
{
  // Wait till CAN is up and running.
  pilz_utils::waitForMessage<sensor_msgs::msg::JointState>(JOINT_STATE_TOPIC, node_);
  
  pilz_utils::waitForService(CANOPEN_GETOBJECT_SERVICE_NAME, node_);
  canopen_srv_get_client_ = node_->create_client<canopen_chain_node::srv::GetObject>(CANOPEN_GETOBJECT_SERVICE_NAME);
}

std::string SystemInfo::getFirmwareVersionOfJoint(const std::string& joint_name)
{
  auto srv = std::make_shared<canopen_chain_node::srv::GetObject::Request>();
  srv->node = joint_name;
  srv->object = GET_FIRMWARE_VERSION_OBJECT;
  srv->cached = false;

  RCLCPP_INFO(node_->get_logger(), "Call \"get firmware\" service for \"%s\"", joint_name.c_str());

  auto result = canopen_srv_get_client_->async_send_request(srv);
  if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS)
  {
    throw SystemInfoException("CANopen service to request firmware version failed");
  }

  auto response = result.get();
  if (!response->success)
  {
    throw SystemInfoException(response->message);
  }

  response->value.resize(FIRMWARE_STRING_LENGTH);
  return response->value;
}

FirmwareCont SystemInfo::getFirmwareVersions()
{
  FirmwareCont versions;
  for (const auto& joint : joint_names_)
  {
    versions[joint] = getFirmwareVersionOfJoint(joint);
  }
  return versions;
}

std::vector<std::string> SystemInfo::getNodeNames()
{
  rclcpp::Parameter param;
  if (!node_->get_parameter(CANOPEN_NODES_PARAMETER_NAME, param))
  {
    throw SystemInfoException("Could not read node names");
  }

  auto rpc = param.as_string_array();
  std::vector<std::string> node_names(rpc.begin(), rpc.end());
  return node_names;
}

}  // namespace prbt_support
