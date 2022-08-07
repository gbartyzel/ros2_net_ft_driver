#ifndef NET_FT_DRIVER_INTERFACES__NET_FT_INTERFACE_HPP_
#define NET_FT_DRIVER_INTERFACES__NET_FT_INTERFACE_HPP_

#include <array>
#include <asio.hpp>
#include <cstdlib>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>
#include <curlpp/cURLpp.hpp>
#include <memory>
#include <string>

namespace net_ft_driver
{
using Vector6D = std::array<double, 6>;

struct SensorData
{
  Vector6D ft_values;
  uint32_t lost_packets;
  uint32_t packet_count;
  uint32_t out_of_order_count;
  uint32_t status;
};

class NetFTInterface
{
public:
  NetFTInterface(const std::string & ip_address, int max_sampling_freq_);

  NetFTInterface() = delete;

  virtual ~NetFTInterface();

  bool start_streaming();

  bool stop_streaming();

  virtual bool set_sampling_rate(int rate);

  virtual bool set_adc_sampling_rate(int rate);

  std::unique_ptr<SensorData> receive_data();

protected:
  template <typename T>
  bool set_cgi_variable(const std::string & cgi_name, const std::string & var_name, T value)
  {
    try
    {
      curlpp::Cleanup cleanup;
      curlpp::Easy request;
      std::string xml_url{"http://" + ip_address_ + "/" + cgi_name + "?" + var_name + "&" + std::to_string(value)};
      request.setOpt(new curlpp::options::Url(xml_url));
      request.perform();
      return true;
    }
    catch (curlpp::RuntimeError & e)
    {
      std::cerr << e.what() << std::endl;
    }
    catch (curlpp::LogicError & e)
    {
      std::cerr << e.what() << std::endl;
    }
    return false;
  }

  bool send_command(uint32_t command, uint32_t sample_count);

  std::string get_config(const std::string & xml_name);

  std::string parse_config(const std::string & response, const std::string & root, const std::string & var_name);

  void unpack(const uint8_t * buffer);

  int min_sampling_freq_;
  int max_sampling_freq_;

private:
  void pack(uint8_t * buffer, uint32_t command, uint32_t sample_count) const;

  asio::io_service io_service_;
  asio::ip::udp::socket socket_;

  std::string ip_address_;

  double force_scale_;
  double torque_scale_;

  uint32_t rdt_sequence_;
  uint32_t ft_sequence_;
  uint32_t last_rdt_sequence_;

  uint32_t lost_packets_;
  uint32_t packet_count_;
  uint32_t out_of_order_count_;
  uint32_t status_;

  Vector6D raw_ft_values_;
};
}  // namespace net_ft_driver

#endif  //NET_FT_DRIVER_INTERFACES__NET_FT_INTERFACE_HPP_
