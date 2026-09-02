#ifndef FAST_LIVO_ADAPTIVE_WINDOW_TYPES_H_
#define FAST_LIVO_ADAPTIVE_WINDOW_TYPES_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace fast_livo
{
namespace adaptive_window
{

constexpr int kNavStateDim = 15;
constexpr int kPoseDim = 6;
constexpr int kExtrinsicDim = 6;

enum class Modality
{
  kPrior = 0,
  kImu,
  kLidar,
  kVision,
  kCrossModal
};

inline const char *ToString(Modality modality)
{
  switch (modality)
  {
    case Modality::kPrior: return "prior";
    case Modality::kImu: return "imu";
    case Modality::kLidar: return "lidar";
    case Modality::kVision: return "vision";
    case Modality::kCrossModal: return "cross_modal";
  }
  return "unknown";
}

struct NavState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double timestamp = 0.0;
  Eigen::Quaterniond q_w_i = Eigen::Quaterniond::Identity();
  Eigen::Vector3d p_w_i = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_w_i = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_g = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_a = Eigen::Vector3d::Zero();
  Eigen::Vector3d gravity_w = Eigen::Vector3d(0.0, 0.0, -9.81);
};

struct ExtrinsicState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Active transform from the IMU frame to the camera frame:
  // p_C = q_c_i * p_I + p_c_i.
  Eigen::Quaterniond q_c_i = Eigen::Quaterniond::Identity();
  Eigen::Vector3d p_c_i = Eigen::Vector3d::Zero();
};

struct LidarExtrinsic
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Active transform from the LiDAR frame to the IMU frame:
  // p_I = q_i_l * p_L + p_i_l.
  Eigen::Quaterniond q_i_l = Eigen::Quaterniond::Identity();
  Eigen::Vector3d p_i_l = Eigen::Vector3d::Zero();
};

struct ImuSample
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double timestamp = 0.0;
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
};

struct LidarPlaneObservation
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d point_l = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal_w = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d center_w = Eigen::Vector3d::Zero();
  double signed_distance = 0.0;
  double variance = 1.0e-3;
};

struct CrossModalObservation
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double lidar_timestamp = -1.0;
  double image_timestamp = -1.0;
  Eigen::Vector3d point_l = Eigen::Vector3d::Zero();
  Eigen::Vector2d pixel = Eigen::Vector2d::Zero();
  double pixel_sigma = 1.5;
  double tracking_score = 1.0;
  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>
      patch_offsets;
  std::vector<double> reference_intensities;
  double reference_inverse_exposure = 1.0;
  double current_inverse_exposure = 1.0;
  double photometric_sigma = 10.0;

  bool has_photometric_patch() const
  {
    return !patch_offsets.empty() &&
           patch_offsets.size() == reference_intensities.size();
  }
};

struct MeasurementSummary
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Modality modality = Modality::kPrior;
  double timestamp = 0.0;
  Eigen::Matrix<double, kPoseDim, kPoseDim> information =
      Eigen::Matrix<double, kPoseDim, kPoseDim>::Zero();
  std::size_t residual_count = 0;
  double rmse = std::numeric_limits<double>::infinity();
  double inlier_ratio = 0.0;

  double log_det = -std::numeric_limits<double>::infinity();
  double min_eigenvalue = 0.0;
  double condition_number = std::numeric_limits<double>::infinity();
  double normalized_information = 0.0;
  double reliability = 0.0;
  bool valid = false;
};

struct ModalityWeights
{
  double lidar = 1.0;
  double vision = 1.0;
  double cross_modal = 1.0;
};

struct PolicyDecision
{
  ModalityWeights weights;
  int target_window_size = 6;
  bool select_keyframe = false;
  double keyframe_utility = 0.0;
};

struct AdaptiveWindowConfig
{
  bool enabled = false;
  bool feedback_enabled = false;

  int min_window_size = 4;
  int max_window_size = 12;
  int initial_window_size = 6;
  int max_lidar_factors = 160;
  int max_cross_modal_factors = 80;

  double merge_time_tolerance = 1.0e-4;
  double association_max_dt = 0.08;
  double association_time_sigma = 0.025;
  bool allow_pair_reuse = true;

  double min_keyframe_dt = 0.05;
  double max_keyframe_dt = 0.50;
  double keyframe_translation = 0.20;
  double keyframe_rotation_deg = 8.0;
  double keyframe_utility_threshold = 0.55;

  double min_modality_weight = 0.15;
  double max_modality_weight = 2.50;
  double weight_smoothing = 0.80;
  double information_smoothing = 0.90;
  double lidar_rmse_scale = 0.08;
  double vision_rmse_scale = 20.0;

  double information_eigen_floor = 1.0e-9;
  double information_logdet_floor = -35.0;
  double information_logdet_ceiling = 30.0;
  double degeneracy_eigen_threshold = 1.0e-4;

  double solver_damping = 1.0e-6;
  int solver_iterations = 3;
  double huber_delta_lidar = 2.0;
  double huber_delta_cross_modal = 3.0;

  double pose_prior_translation_sigma = 0.08;
  double pose_prior_rotation_deg = 3.0;
  double velocity_prior_sigma = 0.30;
  double bias_prior_sigma = 0.05;
  double extrinsic_translation_sigma = 0.05;
  double extrinsic_rotation_deg = 3.0;

  double imu_acceleration_scale = 1.0;
  double imu_acc_noise = 0.10;
  double imu_gyr_noise = 0.01;
  double imu_acc_bias_rw = 0.001;
  double imu_gyr_bias_rw = 0.0001;

  double feedback_max_translation = 0.10;
  double feedback_max_rotation_deg = 2.0;
};

struct BackendDiagnostics
{
  std::size_t window_size = 0;
  std::size_t factor_count = 0;
  int target_window_size = 0;
  ModalityWeights weights;
  double solve_time_ms = 0.0;
  double last_cost = 0.0;
  bool solve_success = false;
  std::uint64_t last_marginalized_id = 0;
  std::size_t cross_modal_pairs = 0;
  double camera_extrinsic_translation_change = 0.0;
  double camera_extrinsic_rotation_change_deg = 0.0;
};

using NavStateVector =
    std::vector<NavState, Eigen::aligned_allocator<NavState>>;
using ImuSampleVector =
    std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>>;
using LidarPlaneObservationVector =
    std::vector<LidarPlaneObservation,
                Eigen::aligned_allocator<LidarPlaneObservation>>;
using CrossModalObservationVector =
    std::vector<CrossModalObservation,
                Eigen::aligned_allocator<CrossModalObservation>>;

}  // namespace adaptive_window
}  // namespace fast_livo

#endif  // FAST_LIVO_ADAPTIVE_WINDOW_TYPES_H_
