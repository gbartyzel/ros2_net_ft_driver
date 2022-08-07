#ifndef NET_FT_DRIVER_INTERFACES__ONROBOT_FT_INTERFACE_HPP_
#define NET_FT_DRIVER_INTERFACES__ONROBOT_FT_INTERFACE_HPP_

#include "net_ft_driver/interfaces/net_ft_interface.hpp"

namespace net_ft_driver
{
class OnRobotFTInterface : public NetFTInterface
{
public:
  OnRobotFTInterface(const std::string & ip_address);

  OnRobotFTInterface() = delete;

  bool set_sampling_rate(int rate);
};
}  // namespace net_ft_driver
#endif  //NET_FT_DRIVER_INTERFACES__ONROBOT_FT_INTERFACE_HPP_
