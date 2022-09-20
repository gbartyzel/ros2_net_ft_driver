#ifndef NET_FT_DRIVER__HARDWARE_INTERFACE_HPP_
#define NET_FT_DRIVER__HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "net_ft_driver/interfaces/net_ft_interface.hpp"
#include "net_ft_driver/visibility_control.h"

namespace net_ft_driver
{
class NetFtHardwareInterface : public hardware_interface::SensorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(NetFtHardwareInterface)

  NET_FT_DRIVER_PUBLIC
  NetFtHardwareInterface();

  NET_FT_DRIVER_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  NET_FT_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  NET_FT_DRIVER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  NET_FT_DRIVER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  NET_FT_DRIVER_PUBLIC
  hardware_interface::return_type read() override;

private:
  Vector6D apply_offset(Vector6D raw_values);

  std::unique_ptr<NetFTInterface> driver_;

  std::string ip_address_;
  std::string sensor_type_;

  Vector6D ft_sensor_measurements_;
  Vector6D offset_ft_values_;

  double packet_count_;
  double lost_packets_;
  double out_of_order_count_;
  double status_;
};
}  // namespace net_ft_driver

#endif  // NET_FT_DRIVER__HARDWARE_INTERFACE_HPP_
