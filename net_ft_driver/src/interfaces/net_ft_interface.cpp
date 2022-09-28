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

#include "net_ft_driver/interfaces/net_ft_interface.hpp"

#include <bits/stdint-uintn.h>
#include <netinet/in.h>
#include <tinyxml2.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

constexpr int kPort = 49152;

constexpr uint16_t kHeader = 0x1234;
constexpr uint16_t kCommandSize = 8;
constexpr uint16_t kRecordSize = 36;

constexpr uint32_t kStopStreaming = 0x0000;
constexpr uint32_t kStartStreaming = 0x0002;

namespace net_ft_driver
{
NetFTInterface::NetFTInterface(const std::string& ip_address, int max_sampling_freq)
  : socket_(io_service_)
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
  asio::ip::udp::endpoint endpoint(asio::ip::address_v4::from_string(ip_address), kPort);
  socket_.open(asio::ip::udp::v4());
  socket_.connect(endpoint);

  auto cal_config = get_config("netftcalapi.xml");
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

std::unique_ptr<SensorData> NetFTInterface::receive_data()
{
  uint8_t buffer[kRecordSize + 1];
  size_t len = socket_.receive(asio::buffer(buffer, kRecordSize + 1));
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
  std::unique_ptr<SensorData> data(
      new SensorData{ ft_values_, lost_packets_, packet_count_, out_of_order_count_, status_ });
  return data;
}

bool NetFTInterface::send_command(uint32_t command, uint32_t sample_count)
{
  try {
    uint8_t buffer[kRecordSize];
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

void NetFTInterface::pack(uint8_t* buffer, uint32_t command, uint32_t sample_count) const
{
  *reinterpret_cast<uint16_t*>(&buffer[0]) = htons(kHeader);
  *reinterpret_cast<uint16_t*>(&buffer[2]) = htons(command);
  *reinterpret_cast<uint32_t*>(&buffer[4]) = htonl(sample_count);
}

void NetFTInterface::unpack(uint8_t* buffer)
{
  rdt_sequence_ = ntohl(*reinterpret_cast<uint32_t*>(&buffer[0]));
  ft_sequence_ = ntohl(*reinterpret_cast<uint32_t*>(&buffer[4]));
  status_ = ntohl(*reinterpret_cast<uint32_t*>(&buffer[8]));
  for (int i = 0; i < 6; i++) {
    raw_counts_[i] = ntohl(*reinterpret_cast<int32_t*>(&buffer[12 + i * 4]));
    if (i < 3) {
      ft_values_[i] = static_cast<double>(raw_counts_[i]) * force_scale_;
    } else {
      ft_values_[i] = static_cast<double>(raw_counts_[i]) * torque_scale_;
    }
  }
}
}  // namespace net_ft_driver
