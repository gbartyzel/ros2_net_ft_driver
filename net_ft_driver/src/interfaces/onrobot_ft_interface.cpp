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

#include <algorithm>
#include <string>

#include "net_ft_driver/interfaces/onrobot_ft_interface.hpp"

constexpr uint32_t kBias = 0x0042;
constexpr uint32_t kSetInternalFiltering = 0x0081;
constexpr uint32_t kSetSamplingRate = 0x0082;

namespace net_ft_driver
{
OnRobotFTInterface::OnRobotFTInterface(const std::string& ip_address) : NetFTInterface(ip_address, 1000)
{
}

bool OnRobotFTInterface::set_bias()
{
  return send_command(kBias, 255);
}

bool OnRobotFTInterface::clear_bias()
{
  return send_command(kBias, 0);
}

bool OnRobotFTInterface::set_sampling_rate(int rate)
{
  rate = std::max(min_sampling_freq_, std::min(rate, max_sampling_freq_));
  int sample_count = max_sampling_freq_ / rate;
  return send_command(kSetSamplingRate, sample_count);
}
bool OnRobotFTInterface::set_internal_filter(int rate)
{
  if (rate < 0 || rate > 6) {
    std::cerr << "Filter rate out of the range, setting to the closest value!";
    rate = std::max(0, std::min(rate, 6));
  }
  return send_command(kSetInternalFiltering, rate);
}
}  // namespace net_ft_driver
