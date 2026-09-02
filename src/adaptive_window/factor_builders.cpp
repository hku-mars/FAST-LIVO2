#include "adaptive_window/factor_builders.h"

#include "adaptive_window/math_utils.h"

#include <algorithm>
#include <cmath>

namespace fast_livo
{
namespace adaptive_window
{
namespace
{

struct PreintegratedImu
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double dt = 0.0;
  Eigen::Quaterniond delta_q = Eigen::Quaterniond::Identity();
  Eigen::Vector3d delta_v = Eigen::Vector3d::Zero();
  Eigen::Vector3d delta_p = Eigen::Vector3d::Zero();
  bool valid = false;
};

PreintegratedImu IntegrateImu(const ImuSampleVector &samples,
                              const Eigen::Vector3d &bias_g,
                              const Eigen::Vector3d &bias_a,
                              const double acceleration_scale,
                              const double start_time,
                              const double end_time)
{
  PreintegratedImu result;
  if (samples.size() < 2 || end_time <= start_time) return result;

  Eigen::Matrix3d delta_rotation = Eigen::Matrix3d::Identity();
  for (std::size_t k = 0; k + 1 < samples.size(); ++k)
  {
    const ImuSample &head = samples[k];
    const ImuSample &tail = samples[k + 1];
    const double interval_start = std::max(start_time, head.timestamp);
    const double interval_end = std::min(end_time, tail.timestamp);
    const double dt = interval_end - interval_start;
    if (dt <= 0.0) continue;

    const Eigen::Vector3d gyro =
        0.5 * (head.angular_velocity + tail.angular_velocity) - bias_g;
    const Eigen::Vector3d acc_head =
        acceleration_scale * head.acceleration - bias_a;
    const Eigen::Vector3d acc_tail =
        acceleration_scale * tail.acceleration - bias_a;

    const Eigen::Vector3d acceleration_0 = delta_rotation * acc_head;
    const Eigen::Matrix3d next_rotation =
        delta_rotation * ExpQuaternion(gyro * dt).toRotationMatrix();
    const Eigen::Vector3d acceleration_1 = next_rotation * acc_tail;
    const Eigen::Vector3d acceleration =
        0.5 * (acceleration_0 + acceleration_1);

    result.delta_p += result.delta_v * dt + 0.5 * acceleration * dt * dt;
    result.delta_v += acceleration * dt;
    delta_rotation = next_rotation;
    result.dt += dt;
  }

  result.delta_q = Eigen::Quaterniond(delta_rotation).normalized();
  result.valid = result.dt > 1.0e-5;
  return result;
}

Eigen::Matrix<double, kNavStateDim, 1> ImuResidual(
    const NavState &state_i, const NavState &state_j,
    const ImuSampleVector &samples, const AdaptiveWindowConfig &config)
{
  Eigen::Matrix<double, kNavStateDim, 1> residual =
      Eigen::Matrix<double, kNavStateDim, 1>::Zero();
  const PreintegratedImu delta =
      IntegrateImu(samples, state_i.bias_g, state_i.bias_a,
                   config.imu_acceleration_scale, state_i.timestamp,
                   state_j.timestamp);
  if (!delta.valid) return residual;

  const Eigen::Quaterniond predicted_q =
      (state_i.q_w_i * delta.delta_q).normalized();
  const Eigen::Vector3d predicted_v =
      state_i.v_w_i + state_i.gravity_w * delta.dt +
      state_i.q_w_i * delta.delta_v;
  const Eigen::Vector3d predicted_p =
      state_i.p_w_i + state_i.v_w_i * delta.dt +
      0.5 * state_i.gravity_w * delta.dt * delta.dt +
      state_i.q_w_i * delta.delta_p;

  residual.segment<3>(0) =
      LogQuaternion(predicted_q.conjugate() * state_j.q_w_i);
  residual.segment<3>(3) = state_j.p_w_i - predicted_p;
  residual.segment<3>(6) = state_j.v_w_i - predicted_v;
  residual.segment<3>(9) = state_j.bias_g - state_i.bias_g;
  residual.segment<3>(12) = state_j.bias_a - state_i.bias_a;
  return residual;
}

Eigen::Matrix<double, kNavStateDim, kNavStateDim> NumericalImuJacobian(
    const bool perturb_first_state, const NavState &state_i,
    const NavState &state_j, const ImuSampleVector &samples,
    const AdaptiveWindowConfig &config,
    const Eigen::Matrix<double, kNavStateDim, 1> &base_residual)
{
  Eigen::Matrix<double, kNavStateDim, kNavStateDim> jacobian;
  const double epsilon = 1.0e-6;
  for (int column = 0; column < kNavStateDim; ++column)
  {
    Eigen::Matrix<double, kNavStateDim, 1> increment =
        Eigen::Matrix<double, kNavStateDim, 1>::Zero();
    increment(column) = epsilon;
    NavState perturbed_i = state_i;
    NavState perturbed_j = state_j;
    if (perturb_first_state) perturbed_i = ApplyIncrement(state_i, increment);
    else perturbed_j = ApplyIncrement(state_j, increment);
    jacobian.col(column) =
        (ImuResidual(perturbed_i, perturbed_j, samples, config) -
         base_residual) /
        epsilon;
  }
  return jacobian;
}

}  // namespace

LinearFactor FactorBuilders::PosePrior(
    const std::uint64_t node_id, const NavState &linearization_state,
    const NavState &measurement,
    const Eigen::Matrix<double, kPoseDim, kPoseDim> &information,
    const Modality modality, const double modality_weight)
{
  LinearFactor factor;
  factor.modality = modality;
  factor.keys = {ParameterBlockKey::NavState(node_id)};
  factor.residual.resize(kPoseDim);
  factor.residual.head<3>() = LogQuaternion(
      measurement.q_w_i.conjugate() * linearization_state.q_w_i);
  factor.residual.tail<3>() =
      linearization_state.p_w_i - measurement.p_w_i;
  factor.jacobians = {
      Eigen::MatrixXd::Zero(kPoseDim, kNavStateDim)};
  factor.jacobians[0].leftCols(kPoseDim).setIdentity();
  factor.sqrt_information = SymmetricSqrtInformation(
      std::max(0.0, modality_weight) * information);
  return factor;
}

LinearFactor FactorBuilders::ExtrinsicPrior(
    const ExtrinsicState &linearization_extrinsic,
    const ExtrinsicState &prior_extrinsic,
    const double rotation_sigma_radians, const double translation_sigma)
{
  LinearFactor factor;
  factor.modality = Modality::kPrior;
  factor.keys = {ParameterBlockKey::CameraExtrinsic()};
  factor.residual =
      ExtrinsicDifference(prior_extrinsic, linearization_extrinsic);
  factor.jacobians = {Eigen::Matrix<double, kExtrinsicDim,
                                                  kExtrinsicDim>::Identity()};
  factor.sqrt_information =
      Eigen::Matrix<double, kExtrinsicDim, kExtrinsicDim>::Zero();
  factor.sqrt_information.topLeftCorner<3, 3>().diagonal().setConstant(
      1.0 / std::max(1.0e-9, rotation_sigma_radians));
  factor.sqrt_information.bottomRightCorner<3, 3>().diagonal().setConstant(
      1.0 / std::max(1.0e-9, translation_sigma));
  return factor;
}

LinearFactor FactorBuilders::ImuPreintegration(
    const std::uint64_t node_i, const std::uint64_t node_j,
    const NavState &state_i, const NavState &state_j,
    const ImuSampleVector &samples, const AdaptiveWindowConfig &config)
{
  LinearFactor factor;
  if (samples.size() < 2 || state_j.timestamp <= state_i.timestamp)
    return factor;

  factor.modality = Modality::kImu;
  factor.keys = {ParameterBlockKey::NavState(node_i),
                 ParameterBlockKey::NavState(node_j)};
  factor.residual = ImuResidual(state_i, state_j, samples, config);
  factor.jacobians = {
      NumericalImuJacobian(true, state_i, state_j, samples, config,
                           factor.residual),
      NumericalImuJacobian(false, state_i, state_j, samples, config,
                           factor.residual)};

  const double dt = std::max(1.0e-3, state_j.timestamp - state_i.timestamp);
  factor.sqrt_information =
      Eigen::Matrix<double, kNavStateDim, kNavStateDim>::Zero();
  factor.sqrt_information.block<3, 3>(0, 0).diagonal().setConstant(
      1.0 / std::max(1.0e-6, config.imu_gyr_noise * std::sqrt(dt)));
  factor.sqrt_information.block<3, 3>(3, 3).diagonal().setConstant(
      1.0 / std::max(1.0e-6,
                     config.imu_acc_noise * std::pow(dt, 1.5)));
  factor.sqrt_information.block<3, 3>(6, 6).diagonal().setConstant(
      1.0 / std::max(1.0e-6, config.imu_acc_noise * std::sqrt(dt)));
  factor.sqrt_information.block<3, 3>(9, 9).diagonal().setConstant(
      1.0 / std::max(1.0e-8, config.imu_gyr_bias_rw * std::sqrt(dt)));
  factor.sqrt_information.block<3, 3>(12, 12).diagonal().setConstant(
      1.0 / std::max(1.0e-8, config.imu_acc_bias_rw * std::sqrt(dt)));
  return factor;
}

LinearFactor FactorBuilders::LidarPointToPlane(
    const std::uint64_t node_id, const NavState &state,
    const LidarExtrinsic &lidar_extrinsic,
    const LidarPlaneObservation &observation, const double modality_weight,
    const double huber_delta)
{
  LinearFactor factor;
  const Eigen::Vector3d point_i =
      lidar_extrinsic.q_i_l * observation.point_l +
      lidar_extrinsic.p_i_l;
  const Eigen::Matrix3d rotation_w_i = state.q_w_i.toRotationMatrix();
  const Eigen::Vector3d point_w = rotation_w_i * point_i + state.p_w_i;

  factor.modality = Modality::kLidar;
  factor.keys = {ParameterBlockKey::NavState(node_id)};
  factor.residual.resize(1);
  factor.residual(0) =
      observation.normal_w.dot(point_w - observation.center_w);
  factor.jacobians = {Eigen::MatrixXd::Zero(1, kNavStateDim)};
  factor.jacobians[0].block<1, 3>(0, 0) =
      observation.normal_w.transpose() * (-rotation_w_i * Skew(point_i));
  factor.jacobians[0].block<1, 3>(0, 3) =
      observation.normal_w.transpose();
  factor.sqrt_information.resize(1, 1);
  factor.sqrt_information(0, 0) =
      std::sqrt(std::max(0.0, modality_weight) /
                std::max(1.0e-9, observation.variance));
  factor.huber_delta = huber_delta;
  return factor;
}

LinearFactor FactorBuilders::CrossModalReprojection(
    const std::uint64_t lidar_node_id,
    const std::uint64_t image_node_id, const NavState &lidar_state,
    const NavState &image_state, const LidarExtrinsic &lidar_extrinsic,
    const ExtrinsicState &camera_extrinsic,
    const CrossModalObservation &observation,
    const ProjectionFunction &projection, const double modality_weight,
    const double temporal_confidence, const double huber_delta)
{
  LinearFactor factor;
  if (!projection) return factor;

  const Eigen::Matrix3d rotation_w_i_l =
      lidar_state.q_w_i.toRotationMatrix();
  const Eigen::Matrix3d rotation_w_i_c =
      image_state.q_w_i.toRotationMatrix();
  const Eigen::Matrix3d rotation_c_i =
      camera_extrinsic.q_c_i.toRotationMatrix();
  const Eigen::Vector3d point_i_l =
      lidar_extrinsic.q_i_l * observation.point_l +
      lidar_extrinsic.p_i_l;
  const Eigen::Vector3d point_w =
      rotation_w_i_l * point_i_l + lidar_state.p_w_i;
  const Eigen::Vector3d point_i_c =
      rotation_w_i_c.transpose() * (point_w - image_state.p_w_i);
  const Eigen::Vector3d point_c =
      rotation_c_i * point_i_c + camera_extrinsic.p_c_i;

  Eigen::Vector2d prediction;
  Eigen::Matrix<double, 2, 3> projection_jacobian;
  if (!projection(point_c, &prediction, &projection_jacobian)) return factor;

  factor.modality = Modality::kCrossModal;
  factor.keys = {ParameterBlockKey::NavState(lidar_node_id),
                 ParameterBlockKey::NavState(image_node_id),
                 ParameterBlockKey::CameraExtrinsic()};
  factor.residual = prediction - observation.pixel;
  factor.jacobians = {
      Eigen::MatrixXd::Zero(2, kNavStateDim),
      Eigen::MatrixXd::Zero(2, kNavStateDim),
      Eigen::MatrixXd::Zero(2, kExtrinsicDim)};

  factor.jacobians[0].block<2, 3>(0, 0) =
      projection_jacobian * rotation_c_i * rotation_w_i_c.transpose() *
      (-rotation_w_i_l * Skew(point_i_l));
  factor.jacobians[0].block<2, 3>(0, 3) =
      projection_jacobian * rotation_c_i * rotation_w_i_c.transpose();

  factor.jacobians[1].block<2, 3>(0, 0) =
      projection_jacobian * rotation_c_i * Skew(point_i_c);
  factor.jacobians[1].block<2, 3>(0, 3) =
      projection_jacobian * rotation_c_i * (-rotation_w_i_c.transpose());

  factor.jacobians[2].block<2, 3>(0, 0) =
      projection_jacobian * (-rotation_c_i * Skew(point_i_c));
  factor.jacobians[2].block<2, 3>(0, 3) = projection_jacobian;

  const double confidence =
      Clamp(temporal_confidence * observation.tracking_score, 0.0, 1.0);
  factor.sqrt_information =
      Eigen::Matrix2d::Identity() *
      (std::sqrt(std::max(0.0, modality_weight) * confidence) /
       std::max(1.0e-6, observation.pixel_sigma));
  factor.huber_delta = huber_delta;
  return factor;
}

LinearFactor FactorBuilders::CrossModalPhotometric(
    const std::uint64_t lidar_node_id,
    const std::uint64_t image_node_id, const NavState &lidar_state,
    const NavState &image_state, const LidarExtrinsic &lidar_extrinsic,
    const ExtrinsicState &camera_extrinsic,
    const CrossModalObservation &observation,
    const ProjectionFunction &projection,
    const ImageSampleFunction &image_sampler, const double modality_weight,
    const double temporal_confidence, const double huber_delta)
{
  LinearFactor factor;
  if (!projection || !image_sampler || !observation.has_photometric_patch())
    return factor;

  const Eigen::Matrix3d rotation_w_i_l =
      lidar_state.q_w_i.toRotationMatrix();
  const Eigen::Matrix3d rotation_w_i_c =
      image_state.q_w_i.toRotationMatrix();
  const Eigen::Matrix3d rotation_c_i =
      camera_extrinsic.q_c_i.toRotationMatrix();
  const Eigen::Vector3d point_i_l =
      lidar_extrinsic.q_i_l * observation.point_l +
      lidar_extrinsic.p_i_l;
  const Eigen::Vector3d point_w =
      rotation_w_i_l * point_i_l + lidar_state.p_w_i;
  const Eigen::Vector3d point_i_c =
      rotation_w_i_c.transpose() * (point_w - image_state.p_w_i);
  const Eigen::Vector3d point_c =
      rotation_c_i * point_i_c + camera_extrinsic.p_c_i;

  Eigen::Vector2d center_pixel;
  Eigen::Matrix<double, 2, 3> projection_jacobian;
  if (!projection(point_c, &center_pixel, &projection_jacobian))
    return factor;

  const Eigen::Matrix<double, 2, 3> source_rotation_jacobian =
      projection_jacobian * rotation_c_i * rotation_w_i_c.transpose() *
      (-rotation_w_i_l * Skew(point_i_l));
  const Eigen::Matrix<double, 2, 3> source_position_jacobian =
      projection_jacobian * rotation_c_i * rotation_w_i_c.transpose();
  const Eigen::Matrix<double, 2, 3> image_rotation_jacobian =
      projection_jacobian * rotation_c_i * Skew(point_i_c);
  const Eigen::Matrix<double, 2, 3> image_position_jacobian =
      projection_jacobian * rotation_c_i * (-rotation_w_i_c.transpose());
  const Eigen::Matrix<double, 2, 3> extrinsic_rotation_jacobian =
      projection_jacobian * (-rotation_c_i * Skew(point_i_c));
  const Eigen::Matrix<double, 2, 3> extrinsic_position_jacobian =
      projection_jacobian;

  const int maximum_rows =
      static_cast<int>(observation.patch_offsets.size());
  factor.modality = Modality::kCrossModal;
  factor.keys = {ParameterBlockKey::NavState(lidar_node_id),
                 ParameterBlockKey::NavState(image_node_id),
                 ParameterBlockKey::CameraExtrinsic()};
  factor.residual = Eigen::VectorXd::Zero(maximum_rows);
  factor.jacobians = {
      Eigen::MatrixXd::Zero(maximum_rows, kNavStateDim),
      Eigen::MatrixXd::Zero(maximum_rows, kNavStateDim),
      Eigen::MatrixXd::Zero(maximum_rows, kExtrinsicDim)};

  int valid_rows = 0;
  for (int i = 0; i < maximum_rows; ++i)
  {
    double current_intensity = 0.0;
    Eigen::Vector2d image_gradient = Eigen::Vector2d::Zero();
    if (!image_sampler(center_pixel + observation.patch_offsets[i],
                       &current_intensity, &image_gradient))
      continue;

    factor.residual(valid_rows) =
        observation.current_inverse_exposure * current_intensity -
        observation.reference_inverse_exposure *
            observation.reference_intensities[i];
    const Eigen::Matrix<double, 1, 2> photometric_projection =
        observation.current_inverse_exposure * image_gradient.transpose();
    factor.jacobians[0].block<1, 3>(valid_rows, 0) =
        photometric_projection * source_rotation_jacobian;
    factor.jacobians[0].block<1, 3>(valid_rows, 3) =
        photometric_projection * source_position_jacobian;
    factor.jacobians[1].block<1, 3>(valid_rows, 0) =
        photometric_projection * image_rotation_jacobian;
    factor.jacobians[1].block<1, 3>(valid_rows, 3) =
        photometric_projection * image_position_jacobian;
    factor.jacobians[2].block<1, 3>(valid_rows, 0) =
        photometric_projection * extrinsic_rotation_jacobian;
    factor.jacobians[2].block<1, 3>(valid_rows, 3) =
        photometric_projection * extrinsic_position_jacobian;
    ++valid_rows;
  }

  if (valid_rows < 4) return LinearFactor();
  factor.residual.conservativeResize(valid_rows);
  for (Eigen::MatrixXd &jacobian : factor.jacobians)
    jacobian.conservativeResize(valid_rows, jacobian.cols());

  const double confidence =
      Clamp(temporal_confidence * observation.tracking_score, 0.0, 1.0);
  factor.sqrt_information =
      Eigen::MatrixXd::Identity(valid_rows, valid_rows) *
      (std::sqrt(std::max(0.0, modality_weight) * confidence) /
       std::max(1.0e-6, observation.photometric_sigma));
  factor.huber_delta = huber_delta;
  return factor;
}

}  // namespace adaptive_window
}  // namespace fast_livo
