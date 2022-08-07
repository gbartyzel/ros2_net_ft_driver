#include <algorithm>

#include "net_ft_driver/interfaces/onrobot_ft_intetface.hpp"

constexpr uint32_t kSetSamplingRate = 0x0082;

namespace net_ft_driver
{
OnRobotFTInterface::OnRobotFTInterface(const std::string & ip_address) : NetFTInterface(ip_address, 1000) {}

bool OnRobotFTInterface::set_sampling_rate(int rate)
{
  rate = std::max(min_sampling_freq_, std::min(rate, max_sampling_freq_));
  int sample_count = max_sampling_freq_ / rate;
  return send_command(kSetSamplingRate, sample_count);
}
}  // namespace net_ft_driver
