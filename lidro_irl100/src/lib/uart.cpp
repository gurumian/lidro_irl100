#include "lidro/irl100/uart.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

#include <lidro_msgs/msg/lidro_packet.hpp>

#include "lidro/irl100/time_conversion.hpp"

static uint32_t baud_number_to_rate(uint32_t baud) {
  static std::map<uint32_t, uint32_t> BaudNumberToRate{
    {0, B0},
    {50, B50},
    {75, B75},
    {110, B110},
    {134, B134},
    {150, B150},
    {200, B200},
    {300, B300},
    {600, B600},
    {1200, B1200},
    {1800, B1800},
    {2400, B2400},
    {4800, B4800},
    {9600, B9600},
    {19200, B19200},
    {38400, B38400},
    {57600, B57600},
    {115200, B115200},
    {230400, B230400},
    {460800, B460800},
    {500000, B500000},
    {576000, B576000},
    {921600, B921600},
    {1000000, B1000000},
    {1500000, B1500000},
    {2000000, B2000000},
    {2500000, B2500000},
    {3000000, B3000000},
    {3500000, B3500000},
    {4000000, B4000000},
  };

  if (BaudNumberToRate.count(baud) == 0) {
    throw std::runtime_error("Invalid baudrate");
  }
  return BaudNumberToRate[baud];
}

namespace lidro::irl100 {

  static const size_t packet_size =
      sizeof(lidro_msgs::msg::LidroPacket().data);

  UART::UART(
      rclcpp::Node *private_nh,
      const std::string &dev,
      uint32_t baudrate, bool gps_time)
      : private_nh_(private_nh), dev_(dev), gps_time_(gps_time) {
    baudrate_ = baud_number_to_rate(baudrate);

    fd_ = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) {
      throw std::runtime_error("failed to open:" + dev);
    }

    if (baudrate_ != B0) {
      // Try to set baud rate
      struct termios uart_config = {};
      int termios_state;

      // Back up the original uart configuration to restore it after exit
      if ((termios_state = ::tcgetattr(fd_, &uart_config)) < 0) {
        RCLCPP_DEBUG(private_nh->get_logger(), "ERR GET CONF %s: %d (%d)\n", dev_.c_str(), termios_state, errno);
        close(fd_);
        throw std::runtime_error("error in getting conf");
      }

      // Set up the UART for non-canonical binary communication: 8 bits, 1 stop bit, no parity,
      // no flow control, no modem control
      uart_config.c_iflag &= ~(INPCK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXANY | IXOFF);
      uart_config.c_iflag |= IGNBRK | IGNPAR;

      uart_config.c_oflag &= ~(OPOST | ONLCR | OCRNL | ONOCR | ONLRET | OFILL | NLDLY | VTDLY);
      uart_config.c_oflag |= NL0 | VT0;

      uart_config.c_cflag &= ~(CSIZE | CSTOPB | PARENB);
      uart_config.c_cflag |= CS8 | CREAD | CLOCAL;

      uart_config.c_lflag &= ~(ISIG | ICANON | ECHO | TOSTOP | IEXTEN);

      // Set baud rate
      if (::cfsetispeed(&uart_config, baudrate_) < 0 || ::cfsetospeed(&uart_config, baudrate_) < 0) {
        RCLCPP_DEBUG(private_nh->get_logger(), "ERR SET BAUD %s: %d (%d)\n", dev_.c_str(), termios_state, errno);
        ::close(fd_);
        throw std::runtime_error("error in setting baud");
      }

      if ((termios_state = ::tcsetattr(fd_, TCSANOW, &uart_config)) < 0) {
        RCLCPP_DEBUG(private_nh->get_logger(), "ERR SET CONF %s (%d)\n", dev_.c_str(), errno);
        ::close(fd_);
        throw std::runtime_error("error in setting conf");
      }
    }
  }

  /** @brief destructor */
  UART::~UART() {
    sendCommand("LSTOP");
    ::close(fd_);
  }

  int UART::init() {
    sendCommand("LSTART");
    return 0;
  }

  int UART::sendCommand(const char *arg) {
    std::string cmd{"AT$"};
    cmd.append(arg).append("\r\n");
    
    const size_t len = cmd.length();
    int n{0};
    n = ::write(fd_, cmd.c_str(), len);
    if(n - len) {
      RCLCPP_DEBUG(private_nh_->get_logger(), "failed to write a command");
      return -1;
    }

    // TODO: read ack
    return 0;
  }

  int UART::getPacket(lidro_msgs::msg::LidroPacket *pkt, const double time_offset) {
    rclcpp::Time time1;

    struct pollfd fds[1];
    fds[0].fd = fd_;
    fds[0].events = POLLIN;
    static const int POLL_TIMEOUT = 1000; // one second (in msec)

    off_t offset = 0;
    while (true) {
      do {
        int retval = ::poll(fds, 1, POLL_TIMEOUT);
        if (retval < 0) { // poll() error?
          if (errno != EINTR) {
            RCLCPP_ERROR(private_nh_->get_logger(), "poll() error: %s", ::strerror(errno));
          }

          return -1;
        }

        if (retval == 0) { // poll() timeout?
          RCLCPP_WARN(private_nh_->get_logger(), "Lidro poll() timeout");
          return -1;
        }

        if ((fds[0].revents & POLLERR) ||
            (fds[0].revents & POLLHUP) ||
            (fds[0].revents & POLLNVAL)) {
          RCLCPP_ERROR(private_nh_->get_logger(), "poll() reports Lidro error");
          return -1;
        }
      } while ((fds[0].revents & POLLIN) == 0);

      time1 = private_nh_->get_clock()->now();

      ssize_t nbytes = ::read(fd_, &pkt->data[offset], packet_size - offset);
      offset += nbytes;
      if (nbytes < 0) {
        if (errno != EWOULDBLOCK)
        {
          RCLCPP_ERROR(private_nh_->get_logger(), "recvfail: %s", ::strerror(errno));
          return -1;
        }
      }
      else if (offset >= int(packet_size - 1)) {
        auto findSyncByte = [&]() -> int {
          for (int i = 0; i < int(packet_size); ++i) {
            // H((char *)&pkt->data[0], 1);
            if (pkt->data[i] == 0xFA)
              return i;
          }
          return -1;
        };

        int found_at = findSyncByte();
        if (found_at > 0) {
          offset -= found_at;
          memmove(&pkt->data[0], &pkt->data[found_at], packet_size - found_at);
          // RCLCPP_ERROR(private_nh_->get_logger(), "not fit: %d", found_at);
          continue;
        }
        break; // done
      }

      RCLCPP_DEBUG(
          private_nh_->get_logger(),
          "incomplete packet read: %zd bytes", nbytes);
    }

    rclcpp::Time time2 = private_nh_->get_clock()->now();
    pkt->stamp = rclcpp::Time((time2.nanoseconds() + time1.nanoseconds()) / 2.0 + time_offset);

    return 0;
  }

} // namespace lidro_driver