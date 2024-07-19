#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/msg/point_cloud2.hpp"

// #define LASER_SCAN_SUPPORT

// #ifdef LASER_SCAN_SUPPORT
#include <sensor_msgs/msg/laser_scan.hpp>
// #endif

using namespace std::chrono_literals;

namespace lidro_filter {

class LidroFilter : public rclcpp::Node {
public:
  LidroFilter(const rclcpp::NodeOptions &options);

  void topic_callback(const sensor_msgs::msg::PointCloud2 msg) const;

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr filterCustomOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) const;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filterCustomOutlierRemoval2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) const;

// #ifdef LASER_SCAN_SUPPORT
  std::array<rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr, 4> laser_scan_publishers_;
  void publishLaserScan(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int index) const;
// #endif

  static inline bool is_valid_point(const pcl::PointXYZ &p) {
    return !(std::isinf(p.x) || std::isinf(p.y) || std::isnan(p.x) || std::isnan(p.y));
  }

private:
  struct {
      std::string frame_id;            // tf frame ID
      bool timestamp_first_packet;     // timestamp based on first packet instead of last one
      bool publish_laserscan;
      double range_min;
      double range_max;
  }
  config_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  int count_{};
};

} // namespace lidro_filter

