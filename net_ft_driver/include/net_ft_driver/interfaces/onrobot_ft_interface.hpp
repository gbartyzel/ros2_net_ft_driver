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

#ifndef NET_FT_DRIVER__INTERFACES__ONROBOT_FT_INTERFACE_HPP_
#define NET_FT_DRIVER__INTERFACES__ONROBOT_FT_INTERFACE_HPP_

#include <memory>
#include <string>

#include "net_ft_driver/interfaces/net_ft_interface.hpp"

namespace net_ft_driver
{
class OnRobotFTInterface : public NetFTInterface
{
public:
  explicit OnRobotFTInterface(const std::string& ip_address);

  OnRobotFTInterface() = delete;

  bool set_bias() final;

  bool clear_bias() final;

  bool set_sampling_rate(int rate) final;

  bool set_internal_filter(int rate) final;
};

class OnRobotFTFactory : public NetFTFactory
{
public:
  OnRobotFTFactory()
  {
    NetFTInterface::register_type("onrobot", this);
  }
  std::unique_ptr<NetFTInterface> create(const std::string& ip_address)
  {
    return std::unique_ptr<OnRobotFTInterface>(new OnRobotFTInterface(ip_address));
  }
};

static OnRobotFTFactory onrobot_factory;
}  // namespace net_ft_driver
#endif  // NET_FT_DRIVER__INTERFACES__ONROBOT_FT_INTERFACE_HPP_
