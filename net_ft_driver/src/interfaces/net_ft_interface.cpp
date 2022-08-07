#include "net_ft_driver/interfaces/net_ft_interface.hpp"

#include <netinet/in.h>
#include <tinyxml2.h>

#include <algorithm>
#include <iostream>
#include <sstream>

constexpr int kPort = 49152;

constexpr uint16_t kHeader = 0x1234;
constexpr uint16_t kCommandSize = 8;
constexpr uint16_t kRecordSize = 36;
constexpr uint32_t kNumSamples = 0;

constexpr uint32_t kStopStreaming = 0x0000;
constexpr uint32_t kStartStreaming = 0x0002;

namespace net_ft_driver
{
NetFTInterface::NetFTInterface(const std::string & ip_address, int max_sampling_freq)
: min_sampling_freq_(1),
  max_sampling_freq_(max_sampling_freq),
  socket_(io_service_),
  ip_address_(ip_address),
  force_scale_(1.0),
  torque_scale_(1.0),
  rdt_sequence_(0),
  ft_sequence_(0),
  last_rdt_sequence_(0),
  lost_packets_(0),
  packet_count_(0),
  out_of_order_count_(0),
  status_(0),
  raw_ft_values_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0})
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

bool NetFTInterface::start_streaming() { return send_command(kStartStreaming, kNumSamples); }

bool NetFTInterface::stop_streaming() { return send_command(kStopStreaming, kNumSamples); }

std::unique_ptr<SensorData> NetFTInterface::receive_data()
{
  uint8_t buffer[kRecordSize + 1];
  size_t len = socket_.receive(asio::buffer(buffer, kRecordSize + 1));
  if (len != kRecordSize + 1)
  {
    return nullptr;
  }
  unpack(buffer);

  auto seq_diff = rdt_sequence_ - last_rdt_sequence_;
  if (seq_diff < 1)
  {
    out_of_order_count_++;
  }
  else
  {
    packet_count_++;
    lost_packets_ += (seq_diff - 1);
  }
  std::unique_ptr<SensorData> data(
    new SensorData{raw_ft_values_, lost_packets_, packet_count_, out_of_order_count_, status_});
  return data;
}

bool NetFTInterface::send_command(uint32_t command, uint32_t sample_count)
{
  try
  {
    uint8_t buffer[kRecordSize];
    pack(buffer, command, sample_count);
    socket_.send(asio::buffer(buffer, kCommandSize));
    return true;
  }
  catch (std::exception & e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
    return false;
  }
}

std::string NetFTInterface::get_config(const std::string & xml_name)
{
  try
  {
    curlpp::Cleanup cleanup;
    curlpp::Easy request;

    std::ostringstream os;
    std::string xml_url = "http://" + ip_address_ + "/" + xml_name;
    request.setOpt(new curlpp::options::Url(xml_url));
    os << request;
    return os.str();
  }
  catch (curlpp::RuntimeError & e)
  {
    std::cerr << e.what() << std::endl;
  }
  catch (curlpp::LogicError & e)
  {
    std::cerr << e.what() << std::endl;
  }
  return "";
}

std::string NetFTInterface::parse_config(
  const std::string & response, const std::string & root, const std::string & var_name)
{
  tinyxml2::XMLDocument xml_doc;
  xml_doc.Parse(response.c_str());
  if (xml_doc.Error())
  {
    std::cout << xml_doc.ErrorName() << std::endl;
  }
  else
  {
    tinyxml2::XMLElement * cal_xml = xml_doc.FirstChildElement(root.c_str());
    if (!cal_xml)
    {
      std::cout << "Could not find the '" + root + "' element in the xml file" << std::endl;
    }
    else
    {
      tinyxml2::XMLElement * cpf_xml = cal_xml->FirstChildElement(var_name.c_str());
      if (cpf_xml && cpf_xml->GetText())
      {
        return cpf_xml->GetText();
      }
      else
      {
        std::cerr << "Could not find the '" + var_name + "' attribute" << std::endl;
      }
    }
  }
  return "";
}

void NetFTInterface::pack(uint8_t * buffer, uint32_t command, uint32_t sample_count) const
{
  *(uint16_t *)&buffer[0] = htons(kHeader);
  *(uint16_t *)&buffer[2] = htons(command);
  *(uint32_t *)&buffer[4] = htonl(sample_count);
}

void NetFTInterface::unpack(const uint8_t * buffer)
{
  rdt_sequence_ = ntohl(*(uint32_t *)&buffer[0]);
  ft_sequence_ = ntohl(*(uint32_t *)&buffer[4]);
  status_ = ntohl(*(uint32_t *)&buffer[8]);
  for (int i = 0; i < 6; i++)
  {
    raw_ft_values_[i] = static_cast<double>(ntohl(*(int32_t *)&buffer[12 + i * 4]));
    if (i < 3)
    {
      raw_ft_values_[i] *= force_scale_;
    }
    else
    {
      raw_ft_values_[i] *= torque_scale_;
    }
  }
}
}  // namespace net_ft_driver
