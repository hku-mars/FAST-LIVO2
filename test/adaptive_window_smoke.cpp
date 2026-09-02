#include "adaptive_window/adaptive_window_backend.h"
#include "adaptive_window/math_utils.h"

#include <Eigen/Core>

#include <cassert>
#include <cmath>
#include <iostream>

using fast_livo::adaptive_window::AdaptiveWindowBackend;
using fast_livo::adaptive_window::AdaptiveWindowConfig;
using fast_livo::adaptive_window::CrossModalObservation;
using fast_livo::adaptive_window::CrossModalObservationVector;
using fast_livo::adaptive_window::ExtrinsicState;
using fast_livo::adaptive_window::ImuSample;
using fast_livo::adaptive_window::ImuSampleVector;
using fast_livo::adaptive_window::LidarExtrinsic;
using fast_livo::adaptive_window::LidarPlaneObservation;
using fast_livo::adaptive_window::LidarPlaneObservationVector;
using fast_livo::adaptive_window::MeasurementSummary;
using fast_livo::adaptive_window::Modality;
using fast_livo::adaptive_window::NavState;

namespace
{

void CheckCrossModalJacobian()
{
  using fast_livo::adaptive_window::ApplyIncrement;
  using fast_livo::adaptive_window::FactorBuilders;
  using fast_livo::adaptive_window::LinearFactor;
  using fast_livo::adaptive_window::ProjectionFunction;
  using fast_livo::adaptive_window::kExtrinsicDim;
  using fast_livo::adaptive_window::kNavStateDim;

  ProjectionFunction projection =
      [](const Eigen::Vector3d &point, Eigen::Vector2d *pixel,
         Eigen::Matrix<double, 2, 3> *jacobian) {
        if (point.z() <= 0.1) return false;
        pixel->x() = 250.0 * point.x() / point.z() + 320.0;
        pixel->y() = 245.0 * point.y() / point.z() + 240.0;
        jacobian->setZero();
        (*jacobian)(0, 0) = 250.0 / point.z();
        (*jacobian)(0, 2) =
            -250.0 * point.x() / (point.z() * point.z());
        (*jacobian)(1, 1) = 245.0 / point.z();
        (*jacobian)(1, 2) =
            -245.0 * point.y() / (point.z() * point.z());
        return true;
      };

  NavState lidar_state;
  lidar_state.q_w_i =
      Eigen::Quaterniond(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitY()));
  lidar_state.p_w_i = Eigen::Vector3d(0.2, -0.1, 0.05);
  NavState image_state;
  image_state.q_w_i =
      Eigen::Quaterniond(Eigen::AngleAxisd(-0.08, Eigen::Vector3d::UnitX()));
  image_state.p_w_i = Eigen::Vector3d(0.25, -0.08, 0.07);
  LidarExtrinsic lidar_extrinsic;
  lidar_extrinsic.p_i_l = Eigen::Vector3d(0.03, 0.01, -0.02);
  ExtrinsicState camera_extrinsic;
  camera_extrinsic.q_c_i =
      Eigen::Quaterniond(Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitZ()));
  camera_extrinsic.p_c_i = Eigen::Vector3d(0.02, -0.01, 0.01);
  CrossModalObservation observation;
  observation.point_l = Eigen::Vector3d(0.3, -0.2, 4.0);
  observation.pixel = Eigen::Vector2d(330.0, 225.0);

  const LinearFactor base = FactorBuilders::CrossModalReprojection(
      1, 2, lidar_state, image_state, lidar_extrinsic, camera_extrinsic,
      observation, projection, 1.0, 1.0, 3.0);
  assert(base.IsValid());
  constexpr double epsilon = 1.0e-7;

  for (int block = 0; block < 3; ++block)
  {
    const int columns = block == 2 ? kExtrinsicDim : 6;
    for (int column = 0; column < columns; ++column)
    {
      NavState perturbed_lidar = lidar_state;
      NavState perturbed_image = image_state;
      ExtrinsicState perturbed_extrinsic = camera_extrinsic;
      if (block < 2)
      {
        Eigen::Matrix<double, kNavStateDim, 1> increment =
            Eigen::Matrix<double, kNavStateDim, 1>::Zero();
        increment(column) = epsilon;
        if (block == 0) perturbed_lidar = ApplyIncrement(lidar_state, increment);
        else perturbed_image = ApplyIncrement(image_state, increment);
      }
      else
      {
        Eigen::Matrix<double, kExtrinsicDim, 1> increment =
            Eigen::Matrix<double, kExtrinsicDim, 1>::Zero();
        increment(column) = epsilon;
        perturbed_extrinsic = ApplyIncrement(camera_extrinsic, increment);
      }
      const LinearFactor perturbed = FactorBuilders::CrossModalReprojection(
          1, 2, perturbed_lidar, perturbed_image, lidar_extrinsic,
          perturbed_extrinsic, observation, projection, 1.0, 1.0, 3.0);
      const Eigen::Vector2d numerical =
          (perturbed.residual - base.residual) / epsilon;
      assert((numerical - base.jacobians[block].col(column)).norm() < 1.0e-3);
    }
  }
}

}  // namespace

int main()
{
  CheckCrossModalJacobian();
  AdaptiveWindowConfig config;
  config.enabled = true;
  config.min_window_size = 3;
  config.max_window_size = 5;
  config.initial_window_size = 3;
  config.min_keyframe_dt = 0.01;
  config.max_keyframe_dt = 0.09;
  config.keyframe_utility_threshold = 0.0;
  config.association_max_dt = 0.11;
  config.max_lidar_factors = 8;
  config.max_cross_modal_factors = 8;

  AdaptiveWindowBackend backend(config);
  LidarExtrinsic lidar_extrinsic;
  ExtrinsicState camera_extrinsic;
  backend.Initialize(
      lidar_extrinsic, camera_extrinsic,
      [](const Eigen::Vector3d &point, Eigen::Vector2d *pixel,
         Eigen::Matrix<double, 2, 3> *jacobian) {
        if (point.z() <= 0.1) return false;
        constexpr double fx = 200.0;
        constexpr double fy = 200.0;
        pixel->x() = fx * point.x() / point.z() + 320.0;
        pixel->y() = fy * point.y() / point.z() + 240.0;
        jacobian->setZero();
        (*jacobian)(0, 0) = fx / point.z();
        (*jacobian)(0, 2) = -fx * point.x() / (point.z() * point.z());
        (*jacobian)(1, 1) = fy / point.z();
        (*jacobian)(1, 2) = -fy * point.y() / (point.z() * point.z());
        return true;
      });

  MeasurementSummary lidar_summary;
  lidar_summary.modality = Modality::kLidar;
  lidar_summary.information.setIdentity();
  lidar_summary.information *= 100.0;
  lidar_summary.residual_count = 20;
  lidar_summary.rmse = 0.01;
  lidar_summary.inlier_ratio = 0.9;

  MeasurementSummary vision_summary = lidar_summary;
  vision_summary.modality = Modality::kVision;
  vision_summary.rmse = 5.0;

  NavState latest;
  for (int frame = 0; frame < 9; ++frame)
  {
    const double timestamp = 0.1 * frame;
    NavState state;
    state.timestamp = timestamp;
    state.p_w_i.x() = 0.05 * frame;

    ImuSampleVector imu;
    for (int sample = 0; sample <= 10; ++sample)
    {
      ImuSample value;
      value.timestamp = timestamp - 0.1 + 0.01 * sample;
      value.acceleration = Eigen::Vector3d(0.0, 0.0, 9.81);
      imu.push_back(value);
    }
    backend.AddImuSamples(imu);

    LidarPlaneObservationVector planes;
    for (int i = 0; i < 10; ++i)
    {
      LidarPlaneObservation plane;
      plane.point_l = Eigen::Vector3d(0.1 * i, 0.0, 1.0);
      plane.normal_w = Eigen::Vector3d::UnitZ();
      plane.center_w = Eigen::Vector3d(0.0, 0.0, 1.0);
      plane.variance = 0.01;
      planes.push_back(plane);
    }
    lidar_summary.timestamp = timestamp;
    backend.ProcessLidar(state, lidar_summary, planes);

    CrossModalObservation observation;
    observation.lidar_timestamp = timestamp;
    observation.image_timestamp = timestamp + 0.02;
    observation.point_l = Eigen::Vector3d(0.0, 0.0, 5.0);
    observation.pixel = Eigen::Vector2d(318.0, 240.0);
    observation.pixel_sigma = 2.0;
    observation.patch_offsets = {
        Eigen::Vector2d(-1.0, -1.0), Eigen::Vector2d(1.0, -1.0),
        Eigen::Vector2d(-1.0, 1.0), Eigen::Vector2d(1.0, 1.0)};
    for (const auto &offset : observation.patch_offsets)
      observation.reference_intensities.push_back(
          100.0 + 0.1 * (320.0 + offset.x()) +
          0.2 * (240.0 + offset.y()));
    observation.photometric_sigma = 5.0;
    CrossModalObservationVector cross_observations{observation};

    NavState image_state = state;
    image_state.timestamp = observation.image_timestamp;
    vision_summary.timestamp = image_state.timestamp;
    backend.ProcessVision(
        image_state, vision_summary, cross_observations,
        [](const Eigen::Vector2d &pixel, double *intensity,
           Eigen::Vector2d *gradient) {
          *intensity = 100.0 + 0.1 * pixel.x() + 0.2 * pixel.y();
          *gradient = Eigen::Vector2d(0.1, 0.2);
          return true;
        });
  }

  const auto diagnostics = backend.diagnostics();
  assert(diagnostics.solve_success);
  assert(diagnostics.window_size <=
         static_cast<std::size_t>(diagnostics.target_window_size));
  assert(diagnostics.window_size >= 1);
  assert(diagnostics.cross_modal_pairs > 0);
  assert(backend.LatestEstimate(&latest));
  assert(std::isfinite(latest.p_w_i.x()));

  std::cout << "adaptive_window_smoke: window=" << diagnostics.window_size
            << ", factors=" << diagnostics.factor_count
            << ", cross_pairs=" << diagnostics.cross_modal_pairs << '\n';
  return 0;
}
