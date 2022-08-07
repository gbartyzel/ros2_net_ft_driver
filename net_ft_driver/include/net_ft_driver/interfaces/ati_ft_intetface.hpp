#ifndef NET_FT_DRIVER_INTERFACES__ATI_FT_INTERFACE_HPP_
#define NET_FT_DRIVER_INTERFACES__ATI_FT_INTERFACE_HPP_

#include <string>

#include "net_ft_driver/interfaces/net_ft_interface.hpp"

namespace net_ft_driver
{
class AtiFTInterface : public NetFTInterface
{
public:
  AtiFTInterface(const std::string & ip_address);

  AtiFTInterface() = delete;

  bool set_sampling_rate(int rate) final;
};
}  // namespace net_ft_driver

#endif  // NET_FT_DRIVER_INTERFACES__ATI_FT_INTERFACE_HPP_
