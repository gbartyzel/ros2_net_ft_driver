// Copyright (c) 2022, Grzegorz Bartyzel
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

#ifndef NET_FT_DRIVER__INTERFACES__NET_FT_INTERFACE_HPP_
#define NET_FT_DRIVER__INTERFACES__NET_FT_INTERFACE_HPP_

#include <array>
#include <cstdlib>
#include <memory>
#include <map>
#include <string>
#include <utility>

#include "asio.hpp"

#include "net_ft_driver/types.hpp"

namespace net_ft_driver
{
class NetFTInterface;

class NetFTFactory
{
public:
  NetFTFactory()
  {
  }
  virtual std::unique_ptr<NetFTInterface> create(const std::string& ip_address) = 0;
};

class NetFTInterface
{
public:
  static std::unique_ptr<NetFTInterface> create(const std::string& sensor_type, const std::string& ip_address)
  {
    return get_factory_instance()[sensor_type]->create(ip_address);
  }

  static void register_type(const std::string& sensor_type, NetFTFactory* factory)
  {
    get_factory_instance()[sensor_type] = factory;
  }

  explicit NetFTInterface(const std::string& ip_address, int max_sampling_freq_);

  NetFTInterface() = delete;

  virtual ~NetFTInterface();

  bool start_streaming();

  bool stop_streaming();

  virtual bool set_bias() = 0;

  virtual bool clear_bias() = 0;

  virtual bool set_sampling_rate(int rate) = 0;

  virtual bool set_internal_filter(int rate) = 0;

  std::unique_ptr<types::SensorData> receive_data();

protected:
  static std::map<std::string, NetFTFactory*>& get_factory_instance()
  {
    static std::map<std::string, NetFTFactory*> map_instance;
    return map_instance;
  }

  bool send_command(std::uint32_t command, std::uint32_t sample_count = 0);

  std::string get_config(const std::string& xml_name);

  std::string parse_config(const std::string& response, const std::string& root, const std::string& var_name);

  void pack(std::uint8_t* buffer, std::uint32_t command, std::uint32_t sample_count) const;

  void unpack(std::uint8_t* buffer);

  asio::io_context io_context_;
  asio::ip::udp::socket socket_;

  std::string ip_address_;

  double force_scale_;
  double torque_scale_;

  int min_sampling_freq_;
  int max_sampling_freq_;

  std::uint32_t rdt_sequence_;
  std::uint32_t ft_sequence_;
  std::uint32_t last_rdt_sequence_;

  std::uint32_t lost_packets_;
  std::uint32_t packet_count_;
  std::uint32_t out_of_order_count_;
  std::uint32_t status_;

  types::Vector6D ft_values_;
  types::Vecotr6I32 raw_counts_;
};
}  // namespace net_ft_driver

#endif  // NET_FT_DRIVER__INTERFACES__NET_FT_INTERFACE_HPP_
