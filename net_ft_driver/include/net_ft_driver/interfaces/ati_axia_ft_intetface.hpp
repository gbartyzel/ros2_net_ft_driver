#ifndef NET_FT_DRIVER_INTERFACES__ATI_AXIA_FT_INTERFACE_HPP_
#define NET_FT_DRIVER_INTERFACES__ATI_AXIA_FT_INTERFACE_HPP_

#include <memory>
#include "net_ft_driver/interfaces/net_ft_interface.hpp"

namespace net_ft_driver
{
class AtiAxiaFTInterface : public NetFTInterface
{
public:
  AtiAxiaFTInterface(const std::string & ip_address);

  AtiAxiaFTInterface() = delete;

  bool set_sampling_rate(int rate);
};

class AtiAxiaFTFactory : public NetFTFactory
{
public:
  AtiAxiaFTFactory() { NetFTInterface::register_type("ati_axia", this); }
  std::unique_ptr<NetFTInterface> create(const std::string & ip_address)
  {
    return std::unique_ptr<AtiAxiaFTInterface>(new AtiAxiaFTInterface(ip_address));
  }
};

static AtiAxiaFTFactory ati_axia_factory;
}  // namespace net_ft_driver

#endif  // NET_FT_DRIVER_INTERFACES__ATI_AXIA_FT_INTERFACE_HPP_
