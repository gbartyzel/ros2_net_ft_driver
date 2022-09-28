// Copyright (c) 2022, Grzegorz Bartyzel
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

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
