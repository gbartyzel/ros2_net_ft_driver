// Copyright (c) 2023, Grzegorz Bartyzel
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

#ifndef NET_FT_DIAGNOSTIC_BROADCASTER__NET_FT_DIAGNOSTIC_BROADCASTER_HPP_
#define NET_FT_DIAGNOSTIC_BROADCASTER__NET_FT_DIAGNOSTIC_BROADCASTER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"

namespace net_ft_diagnostic_broadcaster
{
class NetFTDiagnosticBroadcaster : public controller_interface::ControllerInterface
{
public:
  NetFTDiagnosticBroadcaster();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

protected:
  void publish_diagnostic();

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;

  diagnostic_msgs::msg::DiagnosticArray diag_array_;
  uint32_t last_packet_count_;
  double publish_rate_;
};
}  // namespace net_ft_diagnostic_broadcaster

#endif  // NET_FT_DIAGNOSTIC_BROADCASTER__NET_FT_DIAGNOSTIC_BROADCASTER_HPP_
