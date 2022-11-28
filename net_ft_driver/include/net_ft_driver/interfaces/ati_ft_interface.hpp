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

#ifndef NET_FT_DRIVER__INTERFACES__ATI_FT_INTERFACE_HPP_
#define NET_FT_DRIVER__INTERFACES__ATI_FT_INTERFACE_HPP_

#include <memory>
#include <string>

#include "net_ft_driver/interfaces/net_ft_interface.hpp"

namespace net_ft_driver
{
class AtiFTInterface : public NetFTInterface
{
public:
  explicit AtiFTInterface(const std::string& ip_address);

  AtiFTInterface() = delete;

  bool set_bias() final;

  bool clear_bias() final;

  bool set_sampling_rate(int rate) override;

  bool set_internal_filter(int rate) final;

protected:
  bool set_cgi_variable(const std::string& cgi_name, const std::string& var_name, const std::string& value);
};

class AtiFTFactory : public NetFTFactory
{
public:
  AtiFTFactory()
  {
    NetFTInterface::register_type("ati", this);
  }
  std::unique_ptr<NetFTInterface> create(const std::string& ip_address)
  {
    return std::unique_ptr<AtiFTInterface>(new AtiFTInterface(ip_address));
  }
};

static AtiFTFactory ati_factory;

}  // namespace net_ft_driver

#endif  // NET_FT_DRIVER__INTERFACES__ATI_FT_INTERFACE_HPP_
