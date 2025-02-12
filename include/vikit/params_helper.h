#ifndef ROS_PARAMS_HELPER_H_
#define ROS_PARAMS_HELPER_H_

#include <string>
#include <rclcpp/rclcpp.hpp>

namespace vk {

inline
bool hasParam(rclcpp::Node::SharedPtr node, const std::string& name)
{
  return node->has_parameter(name);
}

template<typename T>
T getParam(rclcpp::Node::SharedPtr node, const std::string& name, const T& defaultValue)
{
  T v;
  if (node->get_parameter(name, v)) 
  {
    RCLCPP_INFO(node->get_logger(), "Found parameter: %s, value: %s", name.c_str(), std::to_string(v).c_str());
    return v;
  }
  else
  {
    RCLCPP_WARN(node->get_logger(), "Cannot find value for parameter: %s, assigning default: %s", name.c_str(), std::to_string(defaultValue).c_str());
  }
  return defaultValue;
}

template<typename T>
T getParam(rclcpp::Node::SharedPtr node, const std::string& name)
{
  T v;
  int i = 0;
  while (!node->get_parameter(name, v)) 
  {
    RCLCPP_ERROR(node->get_logger(), "Cannot find value for parameter: %s, will try again.", name.c_str());
    if ((i++) >= 5) return T();
  }

  RCLCPP_INFO(node->get_logger(), "Found parameter: %s, value: %s", name.c_str(), std::to_string(v).c_str());
  return v;
}

} // namespace vk

#endif // ROS_PARAMS_HELPER_H_
