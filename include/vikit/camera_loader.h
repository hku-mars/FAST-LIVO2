#ifndef VIKIT_CAMERA_LOADER_H_
#define VIKIT_CAMERA_LOADER_H_

#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include <vikit/abstract_camera.h>
#include <vikit/pinhole_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/omni_camera.h>
#include <vikit/equidistant_camera.h>
#include <vikit/polynomial_camera.h>

namespace vk {
namespace camera_loader {

/// Load a single camera from ROS 2 parameters
bool loadFromRosNs(const rclcpp::Node* node, const std::string& ns, vk::AbstractCamera* &cam)
{
  std::string cam_model;
  if (!node->get_parameter(ns + ".cam_model", cam_model)) {
    RCLCPP_ERROR(node->get_logger(), "Camera model parameter not found: %s.cam_model", ns.c_str());
    return false;
  }

  if (cam_model == "Ocam") {
    std::string calib_file;
    node->get_parameter(ns + ".cam_calib_file", calib_file);
    cam = new vk::OmniCamera(calib_file);
  }
  else if (cam_model == "Pinhole") {
    int cam_width = 640;
    int cam_height = 480;
    double scale = 1.0;
    double cam_fx = 0.0;
    double cam_fy = 0.0;
    double cam_cx = 0.0;
    double cam_cy = 0.0;
    double cam_d0 = 0.0;
    double cam_d1 = 0.0;
    double cam_d2 = 0.0;
    double cam_d3 = 0.0;

    node->get_parameter(ns + ".cam_width", cam_width);
    node->get_parameter(ns + ".cam_height", cam_height);
    node->get_parameter(ns + ".scale", scale);
    node->get_parameter(ns + ".cam_fx", cam_fx);
    node->get_parameter(ns + ".cam_fy", cam_fy);
    node->get_parameter(ns + ".cam_cx", cam_cx);
    node->get_parameter(ns + ".cam_cy", cam_cy);
    node->get_parameter(ns + ".cam_d0", cam_d0);
    node->get_parameter(ns + ".cam_d1", cam_d1);
    node->get_parameter(ns + ".cam_d2", cam_d2);
    node->get_parameter(ns + ".cam_d3", cam_d3);

    cam = new vk::PinholeCamera(
      cam_width, cam_height, scale,
      cam_fx, cam_fy, cam_cx, cam_cy,
      cam_d0, cam_d1, cam_d2, cam_d3
    );

  }
  else if (cam_model == "EquidistantCamera") {
    int cam_width = 640;
    int cam_height = 480;
    double scale = 1.0;
    double cam_fx = 0.0;
    double cam_fy = 0.0;
    double cam_cx = 0.0;
    double cam_cy = 0.0;
    double k1 = 0.0;
    double k2 = 0.0;
    double k3 = 0.0;
    double k4 = 0.0;
    
    node->get_parameter(ns + ".cam_width", cam_width);
    node->get_parameter(ns + ".cam_height", cam_height);
    node->get_parameter(ns + ".scale", scale);
    node->get_parameter(ns + ".cam_fx", cam_fx);
    node->get_parameter(ns + ".cam_fy", cam_fy);
    node->get_parameter(ns + ".cam_cx", cam_cx);
    node->get_parameter(ns + ".cam_cy", cam_cy);
    node->get_parameter(ns + ".k1", k1);
    node->get_parameter(ns + ".k2", k2);
    node->get_parameter(ns + ".k3", k3);
    node->get_parameter(ns + ".k4", k4);

    cam = new vk::EquidistantCamera(
      cam_width, cam_height, scale,
      cam_fx, cam_fy, cam_cx, cam_cy,
      k1, k2, k3, k4
    );
  }
  else if (cam_model == "Polynomial Camera") {
    double cam_width = 640;
    double cam_height = 480;
    double scale = 1.0;
    double cam_fx = 0.0;
    double cam_fy = 0.0;
    double cam_cx = 0.0;
    double cam_cy = 0.0;
    double cam_skew;
    double k2 = 0.0;
    double k3 = 0.0;
    double k4 = 0.0;
    double k5 = 0.0;
    double k6 = 0.0;
    double k7 = 0.0;
    
    node->get_parameter(ns + ".cam_width", cam_width);
    node->get_parameter(ns + ".cam_height", cam_height);
    // node->get_parameter(ns + ".scale", scale);
    node->get_parameter(ns + ".cam_fx", cam_fx);
    node->get_parameter(ns + ".cam_fy", cam_fy);
    node->get_parameter(ns + ".cam_cx", cam_cx);
    node->get_parameter(ns + ".cam_cy", cam_cy);
    node->get_parameter(ns + ".cam_skew", cam_skew);
    node->get_parameter(ns + ".k2", k2);
    node->get_parameter(ns + ".k3", k3);
    node->get_parameter(ns + ".k4", k4);
    node->get_parameter(ns + ".k5", k5);
    node->get_parameter(ns + ".k6", k6);
    node->get_parameter(ns + ".k7", k7);
    
    cam = new vk::PolynomialCamera(
      cam_width, cam_height,
      cam_fx, cam_fy, cam_cx, cam_cy,
      cam_skew, k2, k3, k4, k5, k6, k7
    );
  }
  else if (cam_model == "ATAN") {
    int cam_width = 640;
    int cam_height = 480;
    double cam_fx = 0.0;
    double cam_fy = 0.0;
    double cam_cx = 0.0;
    double cam_cy = 0.0;
    double cam_d0 = 0.0;

    node->get_parameter(ns + ".cam_width", cam_width);
    node->get_parameter(ns + ".cam_height", cam_height);
    node->get_parameter(ns + ".cam_fx", cam_fx);
    node->get_parameter(ns + ".cam_fy", cam_fy);
    node->get_parameter(ns + ".cam_cx", cam_cx);
    node->get_parameter(ns + ".cam_cy", cam_cy);
    node->get_parameter(ns + ".cam_d0", cam_d0);

    cam = new vk::ATANCamera(
      cam_width, cam_height,
      cam_fx, cam_fy, cam_cx, cam_cy,
      cam_d0
    );
  }
  else {
    RCLCPP_ERROR(node->get_logger(), "Unknown camera model: %s", cam_model.c_str());
    return false;
  }
  
  return true;
}

/// Load multiple cameras from ROS 2 parameters
bool loadFromRosNs(const rclcpp::Node *node, const std::string& ns, std::vector<vk::AbstractCamera*>& cam_list)
{
  int cam_num;
  if (!node->get_parameter(ns + ".cam_num", cam_num)) {
    RCLCPP_ERROR(node->get_logger(), "Camera number parameter not found: %s.cam_num", ns.c_str());
    return false;
  }

  bool success = true;
  for (int i = 0; i < cam_num; i++) {
    std::string cam_ns = ns + ".cam_" + std::to_string(i);
    vk::AbstractCamera *cam;
    
    if (!loadFromRosNs(node, cam_ns, cam)) {
      RCLCPP_WARN(node->get_logger(), "Failed to load camera from namespace: %s", cam_ns.c_str());
      success = false;
    } else {
      cam_list.push_back(cam);
    }
  }

  return success;
}

} // namespace camera_loader
} // namespace vk

#endif // VIKIT_CAMERA_LOADER_H_
