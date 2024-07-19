#ifndef LIDRO_IRL100__TIME_CONVERSION_HPP_
#define LIDRO_IRL100__TIME_CONVERSION_HPP_

#include <rclcpp/time.hpp>

inline
rclcpp::Time resolveHourAmbiguity(const rclcpp::Time & stamp, const rclcpp::Time & nominal_stamp)
{
  const int HALFHOUR_TO_SEC = 1800;
  rclcpp::Time retval = stamp;

  if (nominal_stamp.seconds() > stamp.seconds()) {
    if (nominal_stamp.seconds() - stamp.seconds() > HALFHOUR_TO_SEC) {
      retval = rclcpp::Time(retval.seconds() + 2 * HALFHOUR_TO_SEC);
    }
  } else if (stamp.seconds() - nominal_stamp.seconds() > HALFHOUR_TO_SEC) {
    retval = rclcpp::Time(retval.seconds() - 2 * HALFHOUR_TO_SEC);
  }

  return retval;
}

inline
rclcpp::Time rosTimeFromGpsTimestamp(rclcpp::Time & time_nom, const uint8_t * const data)
{
  // time_nom is used to recover the hour
  const int HOUR_TO_SEC = 3600;
  // time for each packet is a 4 byte uint
  // It is the number of microseconds from the top of the hour
  uint32_t usecs =
    static_cast<uint32_t>(
    ((uint32_t) data[3]) << 24 |
      ((uint32_t) data[2]) << 16 |
      ((uint32_t) data[1]) << 8 |
      ((uint32_t) data[0]));
  uint32_t cur_hour = time_nom.nanoseconds() / 1000000000 / HOUR_TO_SEC;
  auto stamp = rclcpp::Time(
    (cur_hour * HOUR_TO_SEC) + (usecs / 1000000),
    (usecs % 1000000) * 1000);
  return resolveHourAmbiguity(stamp, time_nom);
}

#endif  // LIDRO_IRL100__TIME_CONVERSION_HPP_
