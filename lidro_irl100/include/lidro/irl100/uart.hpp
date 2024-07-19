#ifndef LIDRO_IRL100__UART_HPP_
#define LIDRO_IRL100__UART_HPP_

#include <netinet/in.h>
#include <pcap.h>

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <lidro_msgs/msg/lidro_packet.hpp>
#include <lidro_msgs/msg/lidro_scan.hpp>
#include <poll.h>

namespace lidro::irl100 {
/** @brief lidro input base class */
class UART {
public:
  explicit UART(rclcpp::Node * private_nh, const std::string & dev, uint32_t baudrate, bool gps_time=false);
  ~UART();

  int init();

  int getPacket(
    lidro_msgs::msg::LidroPacket * pkt,
    const double time_offset);

private:
  int sendCommand(const char *cmd);

protected:
  rclcpp::Node * private_nh_{};
  std::string dev_{};
  uint16_t baudrate_{};
  int read_poll_ms_{100};

  int fd_{};
  bool gps_time_;
};

} // namespace lidro::irl100
#endif
