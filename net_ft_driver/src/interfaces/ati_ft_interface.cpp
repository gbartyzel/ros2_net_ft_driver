#include <algorithm>

#include "net_ft_driver/interfaces/ati_ft_intetface.hpp"

namespace net_ft_driver
{
AtiFTInterface::AtiFTInterface(const std::string & ip_address) : NetFTInterface(ip_address, 7000) {}

bool AtiFTInterface::set_sampling_rate(int rate)
{
  rate = std::max(min_sampling_freq_, std::min(rate, max_sampling_freq_));
  return set_cgi_variable("comm.cgi", "commrdtrate", rate);
}
}  // namespace net_ft_driver
