// Copyright (c) 2022, Grzegorz Bartyzel
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

#include "net_ft_driver/hardware_interface.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "net_ft_driver/types.hpp"
#include "net_ft_driver/interfaces/net_ft_interface.hpp"

const auto kLogger = rclcpp::get_logger("NetFtHardwareInerface");

namespace net_ft_driver
{
NetFtHardwareInterface::NetFtHardwareInterface()
{
}

hardware_interface::CallbackReturn NetFtHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SensorInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  ft_sensor_measurements_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  lost_packets_ = 0;
  packet_count_ = 0;
  out_of_order_count_ = 0;
  status_ = 0;

  ip_address_ = info_.hardware_parameters["ip_address"];
  sensor_type_ = info_.hardware_parameters["sensor_type"];
  const int rdt_rate = std::stoi(info_.hardware_parameters["rdt_sampling_rate"]);
  const int internal_filter_rate = std::stoi(info_.hardware_parameters["internal_filter_rate"]);

  driver_ = NetFTInterface::create(sensor_type_, ip_address_);

  if (!driver_->set_sampling_rate(rdt_rate)) {
    RCLCPP_FATAL(kLogger, "Couldn't set RDT sampling rate of the F/T Sensor!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!driver_->set_internal_filter(internal_filter_rate)) {
    RCLCPP_FATAL(kLogger, "Couldn't set internal low pass filter!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(kLogger, "Initialize connection with F/T Sensor");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> NetFtHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (const auto& sensor : info_.sensors) {
    for (std::size_t idx = 0; idx < sensor.state_interfaces.size(); ++idx) {
      state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, sensor.state_interfaces[idx].name,
                                                                       &ft_sensor_measurements_[idx]));
    }
  }

  state_interfaces.emplace_back(hardware_interface::StateInterface("diagnostic", "packet_count", &packet_count_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("diagnostic", "lost_packets", &lost_packets_));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface("diagnostic", "out_of_order_count", &out_of_order_count_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("diagnostic", "status", &status_));
  return state_interfaces;
}

hardware_interface::CallbackReturn
NetFtHardwareInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  std::string use_hardware_biasing = info_.hardware_parameters["use_hardware_biasing"];
  std::transform(use_hardware_biasing.begin(), use_hardware_biasing.end(), use_hardware_biasing.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  if (driver_->start_streaming()) {
    if (use_hardware_biasing == "true") {
      if (!driver_->set_bias()) {
        RCLCPP_FATAL(kLogger, "Couldn't zero sensor with software bias!");
        return hardware_interface::CallbackReturn::ERROR;
      }
    } else {
      if (!driver_->clear_bias()) {
        RCLCPP_FATAL(kLogger, "Couldn't clear sensor software bias!");
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    const auto data = driver_->receive_data();
    if (data) {
      ft_sensor_measurements_ = data->ft_values;
      RCLCPP_INFO(kLogger, "Successfully started data streaming!");
      return hardware_interface::CallbackReturn::SUCCESS;
    }
  }
  RCLCPP_FATAL(kLogger, "The data stream could not be started!");
  return hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn
NetFtHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (driver_->stop_streaming()) {
    RCLCPP_INFO(kLogger, "Successfully stoped data streaming!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  RCLCPP_FATAL(kLogger, "The data stream could not be stopped!");
  return hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::return_type NetFtHardwareInterface::read(const rclcpp::Time& /*time*/,
                                                             const rclcpp::Duration& /*period*/)
{
  const auto data = driver_->receive_data();
  if (data) {
    ft_sensor_measurements_ = data->ft_values;
    lost_packets_ = static_cast<double>(data->lost_packets);
    packet_count_ = static_cast<double>(data->packet_count);
    out_of_order_count_ = static_cast<double>(data->out_of_order_count);
    status_ = static_cast<double>(data->status);
  }
  return hardware_interface::return_type::OK;
}
}  // namespace net_ft_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(net_ft_driver::NetFtHardwareInterface, hardware_interface::SensorInterface)
