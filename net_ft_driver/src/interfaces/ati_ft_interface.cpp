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
    std::string xml_url{ "http://" + ip_address_ + "/" + cgi_name + "?" + var_name + "=" + value };
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
  return set_cgi_variable("comm.cgi", "comrdtrate", std::to_string(rate));
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
