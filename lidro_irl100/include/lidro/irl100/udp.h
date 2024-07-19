#ifndef DIFF_DRIVE_UDP_H_
#define DIFF_DRIVE_UDP_H_

#include <string>
#include <arpa/inet.h>
#include <json/json.h>

namespace net {
class UDP {
public:
  explicit UDP(const std::string &ip, uint16_t port);
  ~UDP();

  int Send(const Json::Value &value);
  int Send(const std::string &msg);

  static std::string to_string(const Json::Value &value);

private:
  int sck_{0};
  struct sockaddr_in server_addr_{};
};

} // namespace net

#endif // DIFF_DRIVE_UDP_H_
