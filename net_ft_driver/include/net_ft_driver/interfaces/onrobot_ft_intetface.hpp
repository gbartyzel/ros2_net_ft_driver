// Copyright (c) 2022, Grzegorz Bartyzel
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef NET_FT_DRIVER_INTERFACES__ONROBOT_FT_INTERFACE_HPP_
#define NET_FT_DRIVER_INTERFACES__ONROBOT_FT_INTERFACE_HPP_

#include <memory>
#include "net_ft_driver/interfaces/net_ft_interface.hpp"

namespace net_ft_driver
{
class OnRobotFTInterface : public NetFTInterface
{
public:
  OnRobotFTInterface(const std::string & ip_address);

  OnRobotFTInterface() = delete;

  bool set_sampling_rate(int rate);
};

class OnRobotFTFactory : public NetFTFactory
{
public:
  OnRobotFTFactory() { NetFTInterface::register_type("onrobot", this); }
  std::unique_ptr<NetFTInterface> create(const std::string & ip_address)
  {
    return std::unique_ptr<OnRobotFTInterface>(new OnRobotFTInterface(ip_address));
  }
};

static OnRobotFTFactory onrobot_factory;
}  // namespace net_ft_driver
#endif  //NET_FT_DRIVER_INTERFACES__ONROBOT_FT_INTERFACE_HPP_
