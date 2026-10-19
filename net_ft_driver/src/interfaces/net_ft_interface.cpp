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

#include "net_ft_driver/interfaces/net_ft_interface.hpp"

#include <bits/stdint-uintn.h>
#include <netinet/in.h>
#include <tinyxml2.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "asio.hpp"
#include "curlpp/Easy.hpp"
#include "curlpp/Options.hpp"
#include "curlpp/cURLpp.hpp"

namespace
{
constexpr int kPort = 49152;

constexpr std::uint16_t kHeader = 0x1234;
constexpr std::uint16_t kCommandSize = 8;
constexpr std::uint16_t kRecordSize = 36;

constexpr std::uint32_t kStopStreaming = 0x0000;
constexpr std::uint32_t kStartStreaming = 0x0002;
}  // namespace

namespace net_ft_driver
{
NetFTInterface::NetFTInterface(const std::string& ip_address, int max_sampling_freq)
  : socket_(io_context_)
  , ip_address_(ip_address)
  , force_scale_(1.0)
  , torque_scale_(1.0)
  , min_sampling_freq_(1)
  , max_sampling_freq_(max_sampling_freq)
  , rdt_sequence_(0)
  , ft_sequence_(0)
  , last_rdt_sequence_(0)
  , lost_packets_(0)
  , packet_count_(0)
  , out_of_order_count_(0)
  , status_(0)
  , ft_values_({ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 })
{
  asio::ip::udp::endpoint endpoint(asio::ip::make_address(ip_address), kPort);
  socket_.open(asio::ip::udp::v4());
  socket_.connect(endpoint);

  const auto cal_config = get_config("netftcalapi.xml");
  force_scale_ = 1.0 / std::stod(parse_config(cal_config, "netftCalibration", "calcpf"));
  torque_scale_ = 1.0 / std::stod(parse_config(cal_config, "netftCalibration", "calcpt"));
}

NetFTInterface::~NetFTInterface()
{
  stop_streaming();
  socket_.close();
}

bool NetFTInterface::start_streaming()
{
  return send_command(kStartStreaming);
}

bool NetFTInterface::stop_streaming()
{
  return send_command(kStopStreaming);
}

std::unique_ptr<types::SensorData> NetFTInterface::receive_data()
{
  std::uint8_t buffer[kRecordSize + 1];
  const std::size_t len = socket_.receive(asio::buffer(buffer, kRecordSize + 1));
  if (len != kRecordSize) {
    return nullptr;
  }
  unpack(buffer);

  auto seq_diff = rdt_sequence_ - last_rdt_sequence_;
  if (seq_diff < 1) {
    out_of_order_count_++;
  } else {
    packet_count_++;
    lost_packets_ += (seq_diff - 1);
  }
  last_rdt_sequence_ = rdt_sequence_;
  return std::make_unique<types::SensorData>(
      types::SensorData{ ft_values_, lost_packets_, packet_count_, out_of_order_count_, status_ });
}

bool NetFTInterface::send_command(std::uint32_t command, std::uint32_t sample_count)
{
  try {
    std::uint8_t buffer[kRecordSize];
    pack(buffer, command, sample_count);
    socket_.send(asio::buffer(buffer, kCommandSize));
    return true;
  } catch (std::exception& e) {
    std::cerr << "Exception: " << e.what() << "\n";
    return false;
  }
}

std::string NetFTInterface::get_config(const std::string& xml_name)
{
  try {
    curlpp::Cleanup cleanup;
    curlpp::Easy request;

    std::ostringstream os;
    std::string xml_url = "http://" + ip_address_ + "/" + xml_name;
    request.setOpt(new curlpp::options::Url(xml_url));
    os << request;
    return os.str();
  } catch (curlpp::RuntimeError& e) {
    std::cerr << e.what() << std::endl;
  } catch (curlpp::LogicError& e) {
    std::cerr << e.what() << std::endl;
  }
  return "";
}

std::string NetFTInterface::parse_config(const std::string& response, const std::string& root,
                                         const std::string& var_name)
{
  tinyxml2::XMLDocument xml_doc;
  xml_doc.Parse(response.c_str());
  if (xml_doc.Error()) {
    std::cout << xml_doc.ErrorName() << std::endl;
  } else {
    tinyxml2::XMLElement* cal_xml = xml_doc.FirstChildElement(root.c_str());
    if (!cal_xml) {
      std::cout << "Could not find the '" + root + "' element in the xml file" << std::endl;
    } else {
      tinyxml2::XMLElement* cpf_xml = cal_xml->FirstChildElement(var_name.c_str());
      if (cpf_xml && cpf_xml->GetText()) {
        return cpf_xml->GetText();
      } else {
        std::cerr << "Could not find the '" + var_name + "' attribute" << std::endl;
      }
    }
  }
  return "";
}

void NetFTInterface::pack(std::uint8_t* buffer, std::uint32_t command, std::uint32_t sample_count) const
{
  *reinterpret_cast<std::uint16_t*>(&buffer[0]) = htons(kHeader);
  *reinterpret_cast<std::uint16_t*>(&buffer[2]) = htons(command);
  *reinterpret_cast<std::uint32_t*>(&buffer[4]) = htonl(sample_count);
}

void NetFTInterface::unpack(std::uint8_t* buffer)
{
  rdt_sequence_ = ntohl(*reinterpret_cast<std::uint32_t*>(&buffer[0]));
  ft_sequence_ = ntohl(*reinterpret_cast<std::uint32_t*>(&buffer[4]));
  status_ = ntohl(*reinterpret_cast<std::uint32_t*>(&buffer[8]));
  for (int idx = 0; idx < 6; idx++) {
    raw_counts_[idx] = ntohl(*reinterpret_cast<int32_t*>(&buffer[12 + idx * 4]));
    if (idx < 3) {
      ft_values_[idx] = static_cast<double>(raw_counts_[idx]) * force_scale_;
    } else {
      ft_values_[idx] = static_cast<double>(raw_counts_[idx]) * torque_scale_;
    }
  }
}
}  // namespace net_ft_driver
