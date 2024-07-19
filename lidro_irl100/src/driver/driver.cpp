#include "lidro/irl100/driver.hpp"

#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <lidro_msgs/msg/lidro_scan.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

#define deg_to_rad(deg) (deg * M_PI / 180.0)

namespace base {
static
uint16_t GetBits16(const uint8_t* data) {
  return (uint16_t(data[0] & 0x00ff)) | uint16_t(0xFF00 & data[1] << 8);
}
}

namespace lidro::irl100 {

Driver::Driver(const rclcpp::NodeOptions & options)
: rclcpp::Node("lidro_irl100_node", options) {
  std::string device = this->declare_parameter("device", std::string("/dev/ttyUSB0"));
  rcl_interfaces::msg::ParameterDescriptor offset_desc;
  offset_desc.name = "time_offset";
  offset_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  offset_desc.description = "Time offset";
  rcl_interfaces::msg::FloatingPointRange offset_range;
  offset_range.from_value = -1.0;
  offset_range.to_value = 1.0;
  offset_desc.floating_point_range.push_back(offset_range);
  config_.time_offset = this->declare_parameter("time_offset", 0.0, offset_desc);

  config_.enabled = this->declare_parameter("enabled", true);
  config_.frame_id = this->declare_parameter("frame_id", std::string("lidro"));
  config_.range_max = this->declare_parameter("range_max", 100.0);
  config_.range_min = this->declare_parameter("range_min", 0.0);
  config_.angle_min = this->declare_parameter("angle_min", -M_PI);
  config_.angle_max = this->declare_parameter("angle_max", M_PI);
  config_.output_laserscan = this->declare_parameter("output_laserscan", false);
  config_.output_pcl2 = this->declare_parameter("output_pcl2", true);
  config_.flip_x_axis = this->declare_parameter("flip_x_axis", false);
  config_.detect_ground = this->declare_parameter("detect_ground", true);

  int baudrate = config_.baudrate = this->declare_parameter("baudrate", 1500000);

  future_ = exit_signal_.get_future();

  config_.npackets = 4;

  input_ = std::make_unique<lidro::irl100::UART>(this, device, baudrate);
  input_->init();
  if(config_.output_laserscan) {
    for(size_t i = 0; i < laserscan_publishers_.size(); ++i) {
      std::string topic_prefix{"scan_out_"};
      topic_prefix.append(std::to_string(i));
      laserscan_publishers_[i] = this->create_publisher<sensor_msgs::msg::LaserScan>(topic_prefix, 1000);
    }
  }

  if(config_.output_pcl2) {
    pcl_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("out", 1000);
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  poll_thread_ = std::thread(&Driver::pollThread, this);
}

Driver::~Driver() {
  exit_signal_.set_value();
  poll_thread_.join();
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool Driver::poll() {
  if (!config_.enabled) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return true;
  }

  auto raw = std::make_unique<lidro_msgs::msg::LidroScan>();
  raw->packets.resize(config_.npackets);

  for (int i = 0; i < config_.npackets; ++i) {
    while (true) {
      int rc = input_->getPacket(&raw->packets[i], config_.time_offset);
      if (rc == 0) {  // got a full packet?
        break;
      }

      if (rc < 0) {  // end of file reached?
        return false;
      }
    }
  }

  if(config_.output_laserscan) {
    publish(*raw);
  }

  if(config_.output_pcl2) {
    builtin_interfaces::msg::Time stamp =
      config_.timestamp_first_packet ? raw->packets.front().stamp : raw->packets.back().stamp;
    auto cloud = toPointCloud2(*raw);
    publish(cloud, stamp);
  }

  // TODO: lidar would be a fixed frame. It doesn't need to be updated multiple times.
  getLinkPosition();

  return true;
}

void Driver::pollThread() {
  std::future_status status;

  do {
    poll();
    status = future_.wait_for(std::chrono::seconds(0));
  } while (status == std::future_status::timeout);
}

// laserscan
int Driver::publish(const lidro_msgs::msg::LidroScan& raw) {
  builtin_interfaces::msg::Time stamp =
  config_.timestamp_first_packet ? raw.packets.front().stamp : raw.packets.back().stamp;
  
  int width = 900;
  int height = config_.npackets;

  sensor_msgs::msg::LaserScan msg{};
  msg.header.frame_id = config_.frame_id;
  msg.header.stamp = stamp;
  const float offset = M_PI/2;
  msg.angle_min = -M_PI - offset;
  msg.angle_max = M_PI - offset;
  msg.angle_increment = 0.00698132; // deg to radian for 0.4
  msg.range_min= config_.range_min;
  msg.range_max = config_.range_max;
  msg.ranges.resize(width);

  for(int i = 0; i < height; ++i) {
    msg.ranges.assign(width, std::numeric_limits<double>::infinity());

    const auto packet = raw.packets[i];
    int id = packet.data[2];
    const double phi = deg_to_rad(_tilted_angles[id]);
    const double sin_phi = std::sin(phi);
    // const double cos_phi = std::cos(phi);
    for(int j = 0, k = 0; j < width; ++j, k=kHeaderSize+j*2) {
      uint16_t distance = base::GetBits16(&packet.data[k]);
      double r = undistort(double(distance)*1e-3); // mm > m
      // r = r*cos_phi;
      
      if(!config_.detect_ground) {
        if(std::isgreater(
            static_cast<double>(std::abs(phi)),
            static_cast<double>(0.f)
          ) &&
          std::isless(
            static_cast<double>(r*sin_phi),
            static_cast<double>(-height_from_the_base_link_)
          )
        ) {
          r = std::numeric_limits<double>::infinity();
        }
      }

      if(config_.flip_x_axis) {
        msg.ranges[j] = r;
      }
      else {
        msg.ranges[width-1-j] = r;
      }
    }
    laserscan_publishers_[id]->publish(msg);
  }
  return 0;
}

int Driver::publish(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, builtin_interfaces::msg::Time stamp) {
  sensor_msgs::msg::PointCloud2 pcl_msg;
  pcl::toROSMsg(*cloud, pcl_msg);
  pcl_msg.header.frame_id = config_.frame_id;
  pcl_msg.header.stamp = stamp;
  pcl_->publish(pcl_msg);
  return 0;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Driver::toPointCloud2(const lidro_msgs::msg::LidroScan& scan) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width = 900; //1800/2;
  cloud->height = config_.npackets; // 4ch
  cloud->is_dense = false;
  cloud->points.resize(cloud->width * cloud->height);

  for(size_t i = 0; i < cloud->height; ++i) {
    const auto packet = scan.packets[i];
    uint32_t id = packet.data[2];
    if(!(id < cloud->height)) {
      // throw std::runtime_error("out of index:" + std::to_string(id));
      // TODO: [Observation] This happens sometimes.
      continue;
    }
    const float phi = deg_to_rad(_tilted_angles[id]);
    const float sin_phi  = std::sin(phi);
    for(size_t j = 0, k = 0; j < cloud->width; ++j, k=kHeaderSize+j*2) {
      uint16_t distance = base::GetBits16(&packet.data[k]);
      const double r = undistort(double(distance)*1e-3); // mm > m
      if(r > config_.range_max || r < config_.range_min) {
        cloud->points[id*cloud->width+j].x = 
        cloud->points[id*cloud->width+j].y = 
        cloud->points[id*cloud->width+j].z = std::numeric_limits<double>::infinity();
        continue;
      }

      if(!config_.detect_ground) {
        if(std::isgreater(
            static_cast<float>(std::abs(phi)),
            static_cast<float>(0.f)
          ) &&
          std::isless( // x <> y
            static_cast<float>(r*sin_phi),
            static_cast<float>(-height_from_the_base_link_)
          )
        ) {
          continue;
        }
      }

      const auto theta = deg_to_rad(0.4 * float(j));
      const float cos_phi = cos(phi);
      cloud->points[id*cloud->width+j].x = r*std::sin(theta)*cos_phi;
      cloud->points[id*cloud->width+j].y = r*std::cos(theta)*cos_phi;
      cloud->points[id*cloud->width+j].z = r*sin_phi;
    }
  }
  return cloud;
}

void Driver::getLinkPosition() {
  std::string target_link = config_.frame_id;
  try {
    geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_->lookupTransform(
                  "base_link", target_link, tf2::TimePointZero);
    height_from_the_base_link_ = std::abs(transformStamped.transform.translation.z);

  }
  catch (tf2::TransformException &e) {
    // RCLCPP_ERROR(this->get_logger(), "Could NOT transform: %s", e.what());
  }
}

double Driver::undistort(double r) {
  const double K{2.1f};
  const double SCALE{0.110};
  return (r*1e1-K*std::log(r*1e1) +1)*SCALE; // m
}

}  // namespace lidro_driver

RCLCPP_COMPONENTS_REGISTER_NODE(lidro::irl100::Driver)
