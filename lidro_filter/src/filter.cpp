#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "lidro_filter/filter.hpp"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <cmath>
#include <limits>

static double distance(const pcl::PointXYZ &p, const pcl::PointXYZ &p0) {
  return sqrt(pow(p.x - p0.x, 2) + pow(p.y - p0.y, 2));
}

using namespace std::chrono_literals;

namespace lidro_filter {

LidroFilter::LidroFilter(const rclcpp::NodeOptions &options)
: rclcpp::Node("lidro_filter_node", options), count_(0) {
  config_.frame_id = this->declare_parameter("frame_id", "");
  config_.timestamp_first_packet = this->declare_parameter("timestamp_first_pack", false);
  config_.publish_laserscan = this->declare_parameter("publish_laserscan", false);
  config_.range_min = this->declare_parameter("range_min", 0.0f);
  config_.range_max = this->declare_parameter("range_max", 40.0f);

  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point2_filtered", 1000);
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "point2_in",
      1000,
      std::bind(&LidroFilter::topic_callback, this, std::placeholders::_1));

  if(config_.publish_laserscan) {
    for(size_t i = 0; i < laser_scan_publishers_.size(); ++i ) {
      std::string topic{"scan_filtered_"};
      topic.append(std::to_string(i));
      laser_scan_publishers_[i] = this->create_publisher<sensor_msgs::msg::LaserScan>(
        topic,
        1000
      );
    }
  }
}

void LidroFilter::topic_callback(const sensor_msgs::msg::PointCloud2 msg) const {
  pcl::PCLPointCloud2 pcl;
  pcl_conversions::toPCL(msg, pcl);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl,*cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = filterCustomOutlierRemoval2(
      cloud
  );

  sensor_msgs::msg::PointCloud2 filtered_msg;
  pcl::toROSMsg(*cloud_filtered, filtered_msg);
  filtered_msg.header.frame_id = config_.frame_id;
  filtered_msg.header.stamp = this->now();
  publisher_->publish(filtered_msg);

  if(config_.publish_laserscan && cloud_filtered) {
    for(size_t i = 0; i < laser_scan_publishers_.size(); ++i) {
      publishLaserScan(cloud_filtered, i);
    }
  }
}



pcl::PointCloud<pcl::PointXYZ>::Ptr LidroFilter::filterCustomOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) const {
  int width = cloud->width;
  int height = cloud->height;
  pcl::PointXYZ prev_point = {};

  static const pcl::PointXYZ null_point = {
      0.0f,
      0.0f,
      0.0f
  };

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  cloud_filtered->width = width;
  cloud_filtered->height = height;
  cloud_filtered->is_dense = true;
  cloud_filtered->header.frame_id = config_.frame_id.empty() ? cloud->header.frame_id : config_.frame_id;
  cloud_filtered->points.resize(cloud_filtered->width * cloud_filtered->height);

  for(int i = 0; i < height; ++i) {
    size_t offset = i*width;
    for(int j = 0; j < width; ++j) {
      const auto &point = cloud->points[i*width + j];
      if(j == 0 || !is_valid_point(prev_point)) {
        prev_point = point;
        continue;
      }

      float dx = point.x - prev_point.x;
      float dy = point.y - prev_point.y;

      prev_point = point;
      float dr = std::fabs(point.y/point.x - dy/dx);
      if(std::isinf(dr) || std::isnan(dr)) continue;

      const float K = 0.8f; // TODO:
      if(std::isless(dr, K)) {
        cloud_filtered->points[offset + j] = null_point;
        continue;
      }

      cloud_filtered->points[offset + j] = point;
    }
  }

  return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LidroFilter::filterCustomOutlierRemoval2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) const {
  int width = cloud->width;
  int height = cloud->height;
  pcl::PointXYZ prev_point = {};

  static const pcl::PointXYZ null_point = {
      0.0f,
      0.0f,
      0.0f
  };

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  cloud_filtered->width = width;
  cloud_filtered->height = height;
  cloud_filtered->is_dense = false;
  cloud_filtered->header.frame_id = config_.frame_id.empty() ? cloud->header.frame_id : config_.frame_id;
  cloud_filtered->points.resize(cloud_filtered->width * cloud_filtered->height);

  for(int i = 0; i < height; ++i) {
    size_t offset = i*width;
    for(int j = 0; j < width; ++j) {
      const auto &point = cloud->points[i*width + j];
      if(j == 0 || !is_valid_point(prev_point)) {
        prev_point = point;
        continue;
      }

      pcl::PointXYZ dP {prev_point.x-point.x, prev_point.y-point.y, 0};

      // (A-B)*B ~= |A-B||B|
      // (A-B)*B
      float dot_product = std::fabs(dP.x*point.x + dP.y*point.y);
      // |A-B||B|
      float cal = distance(prev_point, point) * distance(null_point, point);

      prev_point = point;
      float dr = std::fabs(dot_product - cal);
      if(std::isinf(dr) || std::isnan(dr)) continue;

      const float K = 0.0010f;
      if(std::isless(dr, K)) {
        cloud_filtered->points[offset + j] = null_point;
        continue;
      }

      cloud_filtered->points[offset + j] = point;
    }
  }

  return cloud_filtered;
}


// #ifdef LASER_SCAN_SUPPORT
void LidroFilter::publishLaserScan(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int index) const{
  int width = cloud->width;
  auto msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  msg->header.frame_id = cloud->header.frame_id;
  msg->angle_min = -M_PI;
  msg->angle_max = M_PI;
  msg->angle_increment = 0.00698132;
  msg->range_min= config_.range_min;
  msg->range_max = config_.range_max;
  msg->header.stamp = this->now();
  msg->ranges.resize(width);
  msg->ranges.assign(width, std::numeric_limits<double>::infinity());

  size_t offset = index*width;
  for(int j = 0; j < width; ++j) {
    const auto &point = cloud->points[offset + j];
    if(!is_valid_point(point)) {
      continue;
    }
    
    auto range = std::hypot(point.x, point.y);
    if(range < msg->range_min || range > msg->range_max) continue;

    double angle = std::atan2(point.y, point.x);
    if (angle < msg->angle_min || angle > msg->angle_max) 
      continue;

    int index = (angle - msg->angle_min) / msg->angle_increment;
    msg->ranges[index] = range;
    
  }

  laser_scan_publishers_[index]->publish(std::move(msg));
}
// #endif

} // namespace lidro_filter

//