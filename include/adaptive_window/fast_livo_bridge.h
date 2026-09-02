#ifndef FAST_LIVO_ADAPTIVE_WINDOW_FAST_LIVO_BRIDGE_H_
#define FAST_LIVO_ADAPTIVE_WINDOW_FAST_LIVO_BRIDGE_H_

#include "adaptive_window/adaptive_window_backend.h"
#include "common_lib.h"
#include "vio.h"
#include "voxel_map.h"

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64MultiArray.h>

#include <deque>
#include <memory>

namespace fast_livo
{
namespace adaptive_window
{

class FastLivoBridge
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FastLivoBridge(ros::NodeHandle &node_handle, const M3D &rotation_i_l,
                 const V3D &translation_i_l,
                 const VIOManagerPtr &vio_manager);

  bool enabled() const { return config_.enabled && backend_ != nullptr; }

  bool ProcessLidar(
      double timestamp, const StatesGroup &state,
      const std::deque<sensor_msgs::Imu::ConstPtr> &imu_messages,
      double acceleration_scale, const std::vector<PointToPlane> &planes,
      std::size_t raw_feature_count);

  bool ProcessVision(
      double timestamp, const StatesGroup &state,
      const VIOManagerPtr &vio_manager,
      const std::deque<sensor_msgs::Imu::ConstPtr> &imu_messages,
      double acceleration_scale);

  bool ApplyStateFeedback(double timestamp, StatesGroup *state) const;
  bool ApplyExtrinsicFeedback(const VIOManagerPtr &vio_manager) const;

private:
  static AdaptiveWindowConfig LoadConfig(ros::NodeHandle &node_handle);
  static NavState ToNavState(double timestamp, const StatesGroup &state);
  static void CopyToFastLivoState(const NavState &input, StatesGroup *output);

  MeasurementSummary BuildLidarSummary(
      double timestamp, const StatesGroup &state,
      const std::vector<PointToPlane> &planes,
      std::size_t raw_feature_count,
      LidarPlaneObservationVector *observations) const;
  MeasurementSummary BuildVisionSummary(
      double timestamp, const VIOManagerPtr &vio_manager,
      CrossModalObservationVector *observations) const;
  void Publish(double timestamp) const;

  AdaptiveWindowConfig config_;
  LidarExtrinsic lidar_extrinsic_;
  ExtrinsicState camera_extrinsic_initial_;
  std::unique_ptr<AdaptiveWindowBackend> backend_;

  ros::Publisher odometry_publisher_;
  ros::Publisher diagnostics_publisher_;
  ros::Publisher extrinsic_publisher_;
};

}  // namespace adaptive_window
}  // namespace fast_livo

#endif  // FAST_LIVO_ADAPTIVE_WINDOW_FAST_LIVO_BRIDGE_H_
