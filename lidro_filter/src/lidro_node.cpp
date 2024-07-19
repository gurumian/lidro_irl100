#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "lidro_filter/filter.hpp"

int main(int argc, char ** argv)
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<lidro_filter::LidroFilter>(
      rclcpp::NodeOptions()));

  rclcpp::shutdown();

  return 0;
}
