#ifndef NET_FT_DRIVER_INTERFACES__ATI_FT_INTERFACE_HPP_
#define NET_FT_DRIVER_INTERFACES__ATI_FT_INTERFACE_HPP_

#include <memory>
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

class AtiFTFactory : public NetFTFactory
{
public:
  AtiFTFactory() { NetFTInterface::register_type("ati", this); }
  std::unique_ptr<NetFTInterface> create(const std::string & ip_address)
  {
    return std::unique_ptr<AtiFTInterface>(new AtiFTInterface(ip_address));
  }
};

static AtiFTFactory ati_factory;

}  // namespace net_ft_driver

#endif  // NET_FT_DRIVER_INTERFACES__ATI_FT_INTERFACE_HPP_
