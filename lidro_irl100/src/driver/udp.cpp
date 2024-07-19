#include "lidro/irl100/udp.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <iostream>

namespace net {

UDP::UDP(const std::string &ip, uint16_t port) {
  sck_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (sck_ == -1) {
    perror("Socket creation failed");
    exit(EXIT_FAILURE);
  }

  fprintf(stderr, "%s:%d\n", ip.c_str(), port);
  bzero((char*)&server_addr_, sizeof(server_addr_));
  server_addr_.sin_family = AF_INET;
  server_addr_.sin_port = htons(port);
  server_addr_.sin_addr.s_addr = inet_addr(ip.c_str());
}

UDP::~UDP() {
  if(sck_ > 0) {
    close(sck_);
    sck_ = -1;
  }
}

int UDP::Send(const std::string &msg) {
  return ::sendto(sck_, msg.c_str(), msg.size(), 0, (struct sockaddr*)&server_addr_, sizeof(server_addr_));
}


int UDP::Send(const Json::Value &value) {
  return Send(to_string(value));
}

// static
std::string UDP::to_string(const Json::Value &value) {
  Json::StreamWriterBuilder builder;
  builder["commentStyle"] = "None";
  builder["indentation"] = "   ";  // or whatever you like
  // builder.settings_["emitUTF8"] = true;
  std::unique_ptr<Json::StreamWriter> writer{builder.newStreamWriter()};

  std::ostringstream stream;
  writer->write(value, &stream);
  return stream.str();
}


} // namespace net
