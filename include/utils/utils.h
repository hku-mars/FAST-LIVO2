#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <cstdint> // for int64_t
#include <limits>  // for std::numeric_limits
#include <stdexcept> // for std::out_of_range
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

std::vector<int> convertToIntVectorSafe(const std::vector<int64_t>& int64_vector);

inline double stamp2Sec(const builtin_interfaces::msg::Time& stamp)
{
    return rclcpp::Time(stamp).seconds();
}

inline rclcpp::Time sec2Stamp(double timestamp)
{
  int32_t sec = std::floor(timestamp);
  auto nanosec_d = (timestamp - std::floor(timestamp)) * 1e9;
  uint32_t nanosec = nanosec_d;
  return rclcpp::Time(sec, nanosec);
}

namespace tf
{

inline geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
}

inline geometry_msgs::msg::Quaternion createQuaternionMsgFromRollPitchYaw(double roll, double pitch, double yaw)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return tf2::toMsg(q);
}

inline tf2::Quaternion createQuaternionFromYaw(double yaw)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return q;
}

inline tf2::Quaternion createQuaternionFromRPY(double roll, double pitch, double yaw)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return q;
}
}

inline geometry_msgs::msg::TransformStamped createTransformStamped(
    const tf2::Transform &transform,
    const builtin_interfaces::msg::Time &stamp,
    const std::string &frame_id,
    const std::string &child_frame_id)
{
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = stamp;
    transform_stamped.header.frame_id = frame_id;
    transform_stamped.child_frame_id = child_frame_id;
    transform_stamped.transform = tf2::toMsg(transform);
    return transform_stamped;
}

#endif // UTILS_H