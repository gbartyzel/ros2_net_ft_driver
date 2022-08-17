#ifndef NET_FT_DRIVER_INTERFACES__ONROBOT_FT_INTERFACE_HPP_
#define NET_FT_DRIVER_INTERFACES__ONROBOT_FT_INTERFACE_HPP_

#include <memory>
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

class OnRobotFTFactory : public NetFTFactory
{
public:
  OnRobotFTFactory() { NetFTInterface::register_type("onrobot", this); }
  std::unique_ptr<NetFTInterface> create(const std::string & ip_address)
  {
    return std::unique_ptr<OnRobotFTInterface>(new OnRobotFTInterface(ip_address));
  }
};

static OnRobotFTFactory onrobot_factory;
}  // namespace net_ft_driver
#endif  //NET_FT_DRIVER_INTERFACES__ONROBOT_FT_INTERFACE_HPP_
