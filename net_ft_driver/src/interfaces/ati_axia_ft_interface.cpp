#include <algorithm>
#include <array>

#include "net_ft_driver/interfaces/ati_axia_ft_intetface.hpp"

constexpr std::array<int, 5> kAllowedADCFrequency{488, 976, 1953, 3906, 7812};

namespace net_ft_driver
{
AtiAxiaFTInterface::AtiAxiaFTInterface(const std::string & ip_address) : NetFTInterface(ip_address, 7812) {}

bool AtiAxiaFTInterface::set_sampling_rate(int rate)
{
  auto response = get_config("netftapi2.xml");
  int current_rate = std::stoi(parse_config(response, "netFTApi", "setrate"));
  rate = std::min(min_sampling_freq_, std::max(rate, current_rate));
  return set_cgi_variable("comm.cgi", "commrdtrate", rate);
}

bool AtiAxiaFTInterface::set_adc_sampling_rate(int rate)
{
  if (std::find(kAllowedADCFrequency.begin(), kAllowedADCFrequency.end(), rate))
  {
    return set_cgi_variable("setting.cgi", "setadcrate", rate);
  }
  return false;
}
}  // namespace net_ft_driver
