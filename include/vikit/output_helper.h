/*
 * output_helper.h
 *
 *  Created on: Jan 20, 2013
 *      Author: cforster
 *  Updated for ROS 2 by: [Your Name]
 */

#ifndef VIKIT_OUTPUT_HELPER_H_
#define VIKIT_OUTPUT_HELPER_H_

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace vk {
namespace output_helper {

using namespace std;
using namespace Eigen;

void
publishTfTransform      (const Sophus::SE3d& T, const rclcpp::Time& stamp,
                         const string& frame_id, const string& child_frame_id,
                         tf2_ros::TransformBroadcaster& br);

void
publishPointMarker      (rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                         const Vector3d& pos,
                         const string& ns,
                         const rclcpp::Time& timestamp,
                         int id,
                         int action,
                         double marker_scale,
                         const Vector3d& color,
                         rclcpp::Duration lifetime = rclcpp::Duration(0, 0));

void
publishLineMarker       (rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                         const Vector3d& start,
                         const Vector3d& end,
                         const string& ns,
                         const rclcpp::Time& timestamp,
                         int id,
                         int action,
                         double marker_scale,
                         const Vector3d& color,
                         rclcpp::Duration lifetime = rclcpp::Duration(0, 0));

void
publishArrowMarker      (rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                         const Vector3d& pos,
                         const Vector3d& dir,
                         double scale,
                         const string& ns,
                         const rclcpp::Time& timestamp,
                         int id,
                         int action,
                         double marker_scale,
                         const Vector3d& color);

void
publishHexacopterMarker (rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                         const string& frame_id,
                         const string& ns,
                         const rclcpp::Time& timestamp,
                         int id,
                         int action,
                         double marker_scale,
                         const Vector3d& color);

void
publishCameraMarker(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                    const string& frame_id,
                    const string& ns,
                    const rclcpp::Time& timestamp,
                    int id,
                    double marker_scale,
                    const Vector3d& color);

void
publishFrameMarker     (rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                        const Matrix3d& rot,
                        const Vector3d& pos,
                        const string& ns,
                        const rclcpp::Time& timestamp,
                        int id,
                        int action,
                        double marker_scale,
                        rclcpp::Duration lifetime = rclcpp::Duration(0, 0));

} // namespace output_helper
} // namespace vk

#endif /* VIKIT_OUTPUT_HELPER_H_ */
