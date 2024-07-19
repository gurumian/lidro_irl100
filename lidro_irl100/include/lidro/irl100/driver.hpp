#ifndef LIDRO_IRL100__DRIVER_HPP_
#define LIDRO_IRL100__DRIVER_HPP_

#include <future>
#include <memory>
#include <string>
#include <array>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include "lidro/irl100/uart.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <sensor_msgs/msg/laser_scan.hpp>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace lidro::irl100 {

class Driver final : public rclcpp::Node {
public:
  explicit Driver(const rclcpp::NodeOptions & options);
  ~Driver() override;
  Driver(Driver && c) = delete;
  Driver & operator=(Driver && c) = delete;
  Driver(const Driver & c) = delete;
  Driver & operator=(const Driver & c) = delete;

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr toPointCloud2(const lidro_msgs::msg::LidroScan& raw);
  int publish(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, builtin_interfaces::msg::Time stamp);
  int publish(const lidro_msgs::msg::LidroScan& raw);

  bool poll();
  void pollThread();

  void getLinkPosition();

  double undistort(double distance);

  static constexpr double kHeaderSize{3};
  static constexpr float kAngleIncrement{0.4};
  // Physical device mapping: {LD-0: -18 (deg), LD-1: -27, LD-2: 0, LD-3: -9}
  static constexpr std::array<float, 4> _tilted_angles = {-18., -27., 0., -9. };

  // configuration parameters
  struct {
    std::string frame_id;            // tf frame ID
    std::string model;               // device model name
    int baudrate;                    // baudrate
    int npackets;                    // number of packets to collect
    double rpm;                      // device rotation rate (RPMs)
    double time_offset;              // time in seconds added to each lidro time stamp
    bool enabled;                    // polling is enabled
    bool timestamp_first_packet;     // timestamp based on first packet instead of last one
    double range_max;
    double range_min;
    double angle_max;
    double angle_min;
    bool output_laserscan;
    bool output_pcl2;
    bool flip_x_axis;
    bool detect_ground;
  }
  config_;

  std::unique_ptr<UART> input_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_;
  std::array<rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr, 4> laserscan_publishers_;


  std::shared_future<void> future_;
  std::promise<void> exit_signal_;

  std::thread poll_thread_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  double height_from_the_base_link_{0.f};
};

}  // namespace lidro::irl100

#endif  // LIDRO_IRL100__DRIVER_HPP_
