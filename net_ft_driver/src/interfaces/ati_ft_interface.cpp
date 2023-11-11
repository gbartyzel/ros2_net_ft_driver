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
#include <vector>

#include "net_ft_driver/interfaces/ati_ft_interface.hpp"

constexpr uint32_t kBias = 0x0042;

namespace net_ft_driver
{
AtiFTInterface::AtiFTInterface(const std::string& ip_address) : NetFTInterface(ip_address, 7000)
{
}

bool AtiFTInterface::set_cgi_variable(const std::string& cgi_name, const std::string& var_name,
                                      const std::string& value)
{
  try {
    curlpp::Cleanup cleanup;
    curlpp::Easy request;
    std::string xml_url{ "http://" + ip_address_ + "/" + cgi_name + "?" + var_name + "&" + value };
    request.setOpt(new curlpp::options::Url(xml_url));
    request.perform();
    return true;
  } catch (curlpp::RuntimeError& e) {
    std::cerr << e.what() << std::endl;
  } catch (curlpp::LogicError& e) {
    std::cerr << e.what() << std::endl;
  }
  return false;
}

bool AtiFTInterface::set_bias()
{
  return send_command(kBias);
}

bool AtiFTInterface::clear_bias()
{
  std::vector<bool> cum_ret;
  for (int i = 0; i < 6; i++) {
    auto ret = set_cgi_variable("setting.cgi", "setbias" + std::to_string(i), std::to_string(0));
    cum_ret.push_back(ret);
  }
  return std::all_of(cum_ret.begin(), cum_ret.end(), [](bool v) { return v; });
}

bool AtiFTInterface::set_sampling_rate(int rate)
{
  rate = std::max(min_sampling_freq_, std::min(rate, max_sampling_freq_));
  return set_cgi_variable("comm.cgi", "commrdtrate", std::to_string(rate));
}

bool AtiFTInterface::set_internal_filter(int rate)
{
  if (rate < 0 || rate > 8) {
    std::cerr << "Filter rate out of the range, setting to the closest value!";
    rate = std::max(0, std::min(rate, 8));
  }
  return set_cgi_variable("setting.cgi", "setuserfilter", std::to_string(rate));
}
}  // namespace net_ft_driver
