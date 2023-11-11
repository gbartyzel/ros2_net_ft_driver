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
#include <array>
#include <string>

#include "net_ft_driver/interfaces/ati_axia_ft_interface.hpp"

constexpr std::array<int, 5> kAllowedADCFrequency{ 488, 976, 1953, 3906, 7812 };

namespace net_ft_driver
{
AtiAxiaFTInterface::AtiAxiaFTInterface(const std::string& ip_address) : AtiFTInterface(ip_address)
{
  max_sampling_freq_ = 7812;
}

bool AtiAxiaFTInterface::set_sampling_rate(int rate)
{
  for (auto it = kAllowedADCFrequency.begin(); it != kAllowedADCFrequency.end(); it++) {
    if (rate < *it) {
      set_cgi_variable("setting.cgi", "setadcrate", std::to_string(*it));
      break;
    }
  }
  auto response = get_config("netftapi2.xml");
  int current_rate = std::stoi(parse_config(response, "netft", "setrate"));
  rate = std::max(min_sampling_freq_, std::min(rate, current_rate));
  return set_cgi_variable("comm.cgi", "comrdtrate", std::to_string(rate));
}
}  // namespace net_ft_driver
