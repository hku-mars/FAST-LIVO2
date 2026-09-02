#include "adaptive_window/adaptive_window_backend.h"

#include "adaptive_window/math_utils.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

namespace fast_livo
{
namespace adaptive_window
{

AdaptiveWindowBackend::AdaptiveWindowBackend(
    const AdaptiveWindowConfig &config)
    : config_(config),
      information_evaluator_(config),
      policy_(config),
      associator_(config),
      optimizer_(config),
      target_window_size_(config.initial_window_size)
{
}

void AdaptiveWindowBackend::Initialize(
    const LidarExtrinsic &lidar_extrinsic,
    const ExtrinsicState &camera_extrinsic, ProjectionFunction projection)
{
  lidar_extrinsic_ = lidar_extrinsic;
  camera_extrinsic_initial_ = camera_extrinsic;
  projection_ = std::move(projection);
  optimizer_.Reset(camera_extrinsic_initial_);

  const LinearFactor prior = FactorBuilders::ExtrinsicPrior(
      camera_extrinsic_initial_, camera_extrinsic_initial_,
      Radians(config_.extrinsic_rotation_deg),
      config_.extrinsic_translation_sigma);
  optimizer_.AddFactor(prior);
  diagnostics_ = BackendDiagnostics();
  target_window_size_ = config_.initial_window_size;
}

void AdaptiveWindowBackend::Reset()
{
  pending_imu_.clear();
  associator_.Clear();
  has_lidar_summary_ = false;
  has_vision_summary_ = false;
  latest_node_id_ = 0;
  previous_new_node_id_ = 0;
  Initialize(lidar_extrinsic_, camera_extrinsic_initial_, projection_);
}

void AdaptiveWindowBackend::AddImuSamples(const ImuSampleVector &samples)
{
  for (const ImuSample &sample : samples)
  {
    if (!pending_imu_.empty() &&
        sample.timestamp <= pending_imu_.back().timestamp + 1.0e-9)
    {
      if (std::abs(sample.timestamp - pending_imu_.back().timestamp) < 1.0e-9)
        pending_imu_.back() = sample;
      continue;
    }
    pending_imu_.push_back(sample);
  }
}

Eigen::Matrix<double, kPoseDim, kPoseDim>
AdaptiveWindowBackend::DefaultPoseInformation() const
{
  Eigen::Matrix<double, kPoseDim, kPoseDim> information =
      Eigen::Matrix<double, kPoseDim, kPoseDim>::Zero();
  const double rotation_sigma = Radians(config_.pose_prior_rotation_deg);
  information.topLeftCorner<3, 3>().diagonal().setConstant(
      1.0 / std::max(1.0e-12, rotation_sigma * rotation_sigma));
  information.bottomRightCorner<3, 3>().diagonal().setConstant(
      1.0 /
      std::max(1.0e-12, config_.pose_prior_translation_sigma *
                               config_.pose_prior_translation_sigma));
  return information;
}

std::uint64_t AdaptiveWindowBackend::AddOrGetNode(
    const NavState &state, const Modality modality,
    const MeasurementSummary &summary, const PolicyDecision &decision)
{
  std::uint64_t node_id = optimizer_.FindNodeNear(
      state.timestamp, config_.merge_time_tolerance);
  if (node_id != 0)
  {
    latest_node_id_ = node_id;
    optimizer_.MarkNode(node_id, modality, summary.normalized_information);
    return node_id;
  }

  if (!decision.select_keyframe) return 0;

  node_id = optimizer_.AddNode(state);
  latest_node_id_ = node_id;
  optimizer_.MarkNode(node_id, modality, summary.normalized_information);

  if (optimizer_.NumNodes() == 1)
  {
    optimizer_.AddFactor(FactorBuilders::PosePrior(
        node_id, state, state, DefaultPoseInformation(), Modality::kPrior,
        1.0));
  }
  AddImuFactorToNewNode(node_id);
  previous_new_node_id_ = node_id;
  ConsumeImuThrough(state.timestamp);
  return node_id;
}

void AdaptiveWindowBackend::AddPoseMeasurement(
    const std::uint64_t node_id, const NavState &state,
    const MeasurementSummary &summary, const double modality_weight)
{
  if (node_id == 0) return;
  Eigen::Matrix<double, kPoseDim, kPoseDim> information =
      summary.information;
  information += 1.0e-3 * DefaultPoseInformation();
  optimizer_.AddFactor(FactorBuilders::PosePrior(
      node_id, optimizer_.EstimateNode(node_id), state, information,
      summary.modality, modality_weight));
}

void AdaptiveWindowBackend::AddImuFactorToNewNode(
    const std::uint64_t new_node_id)
{
  if (previous_new_node_id_ == 0 ||
      previous_new_node_id_ == new_node_id || pending_imu_.size() < 2 ||
      !optimizer_.HasNode(previous_new_node_id_))
    return;

  const NavState state_i = optimizer_.EstimateNode(previous_new_node_id_);
  const NavState state_j = optimizer_.EstimateNode(new_node_id);
  LinearFactor factor = FactorBuilders::ImuPreintegration(
      previous_new_node_id_, new_node_id, state_i, state_j, pending_imu_,
      config_);
  if (factor.IsValid()) optimizer_.AddFactor(factor);
}

void AdaptiveWindowBackend::ConsumeImuThrough(const double timestamp)
{
  if (pending_imu_.size() <= 2) return;
  auto keep = std::lower_bound(
      pending_imu_.begin(), pending_imu_.end(), timestamp,
      [](const ImuSample &sample, const double value) {
        return sample.timestamp < value;
      });
  if (keep != pending_imu_.begin()) --keep;
  pending_imu_.erase(pending_imu_.begin(), keep);
}

bool AdaptiveWindowBackend::ProcessLidar(
    const NavState &state, const MeasurementSummary &input_summary,
    const LidarPlaneObservationVector &observations)
{
  MeasurementSummary summary = input_summary;
  summary.modality = Modality::kLidar;
  summary.timestamp = state.timestamp;
  summary = information_evaluator_.Evaluate(summary);
  latest_lidar_summary_ = summary;
  has_lidar_summary_ = true;

  const PolicyDecision decision = policy_.Decide(
      state, &latest_lidar_summary_,
      has_vision_summary_ ? &latest_vision_summary_ : nullptr,
      optimizer_.PolicyFrames(), diagnostics_.solve_time_ms);
  target_window_size_ = decision.target_window_size;
  diagnostics_.weights = decision.weights;

  const std::uint64_t node_id =
      AddOrGetNode(state, Modality::kLidar, summary, decision);
  if (node_id == 0) return false;
  associator_.Add(Modality::kLidar, state.timestamp, node_id);
  AddPoseMeasurement(node_id, state, summary, decision.weights.lidar);

  const std::size_t factor_limit = static_cast<std::size_t>(
      std::max(0, config_.max_lidar_factors));
  if (factor_limit == 0) return OptimizeAndAdapt();
  const std::size_t stride =
      std::max<std::size_t>(1, observations.size() / factor_limit);
  std::size_t factor_count = 0;
  const NavState estimate = optimizer_.EstimateNode(node_id);
  for (std::size_t i = 0;
       i < observations.size() && factor_count < factor_limit; i += stride)
  {
    const LinearFactor factor = FactorBuilders::LidarPointToPlane(
        node_id, estimate, lidar_extrinsic_, observations[i],
        decision.weights.lidar, config_.huber_delta_lidar);
    if (factor.IsValid() && optimizer_.AddFactor(factor)) ++factor_count;
  }
  return OptimizeAndAdapt();
}

bool AdaptiveWindowBackend::ProcessVision(
    const NavState &state, const MeasurementSummary &input_summary,
    const CrossModalObservationVector &observations,
    const ImageSampleFunction &image_sampler)
{
  MeasurementSummary summary = input_summary;
  summary.modality = Modality::kVision;
  summary.timestamp = state.timestamp;
  summary = information_evaluator_.Evaluate(summary);
  latest_vision_summary_ = summary;
  has_vision_summary_ = true;

  const PolicyDecision decision = policy_.Decide(
      state, has_lidar_summary_ ? &latest_lidar_summary_ : nullptr,
      &latest_vision_summary_, optimizer_.PolicyFrames(),
      diagnostics_.solve_time_ms);
  target_window_size_ = decision.target_window_size;
  diagnostics_.weights = decision.weights;

  const std::uint64_t image_node_id =
      AddOrGetNode(state, Modality::kVision, summary, decision);
  if (image_node_id == 0) return false;
  associator_.Add(Modality::kVision, state.timestamp, image_node_id);
  AddPoseMeasurement(image_node_id, state, summary, decision.weights.vision);

  const std::size_t factor_limit = static_cast<std::size_t>(
      std::max(0, config_.max_cross_modal_factors));
  if (factor_limit == 0) return OptimizeAndAdapt();
  const std::size_t stride =
      std::max<std::size_t>(1, observations.size() / factor_limit);
  std::size_t pair_count = 0;
  for (std::size_t i = 0;
       i < observations.size() && pair_count < factor_limit; i += stride)
  {
    const CrossModalObservation &observation = observations[i];
    const TemporalMatch match = associator_.FindNearest(
        Modality::kLidar, observation.lidar_timestamp);
    if (!match.valid || !optimizer_.HasNode(match.node_id)) continue;

    LinearFactor factor;
    if (observation.has_photometric_patch() && image_sampler)
    {
      factor = FactorBuilders::CrossModalPhotometric(
          match.node_id, image_node_id,
          optimizer_.EstimateNode(match.node_id),
          optimizer_.EstimateNode(image_node_id), lidar_extrinsic_,
          optimizer_.EstimateCameraExtrinsic(), observation, projection_,
          image_sampler, decision.weights.cross_modal, match.confidence,
          config_.huber_delta_cross_modal);
    }
    else
    {
      factor = FactorBuilders::CrossModalReprojection(
          match.node_id, image_node_id,
          optimizer_.EstimateNode(match.node_id),
          optimizer_.EstimateNode(image_node_id), lidar_extrinsic_,
          optimizer_.EstimateCameraExtrinsic(), observation, projection_,
          decision.weights.cross_modal, match.confidence,
          config_.huber_delta_cross_modal);
    }
    if (!factor.IsValid() || !optimizer_.AddFactor(factor)) continue;
    optimizer_.MarkNode(match.node_id, Modality::kCrossModal,
                        summary.normalized_information);
    optimizer_.MarkNode(image_node_id, Modality::kCrossModal,
                        summary.normalized_information);
    ++pair_count;
  }
  diagnostics_.cross_modal_pairs += pair_count;
  return OptimizeAndAdapt();
}

bool AdaptiveWindowBackend::OptimizeAndAdapt()
{
  const auto start = std::chrono::steady_clock::now();
  double cost = 0.0;
  const bool solved = optimizer_.Solve(&cost);
  const auto end = std::chrono::steady_clock::now();
  diagnostics_.solve_time_ms =
      std::chrono::duration<double, std::milli>(end - start).count();
  diagnostics_.last_cost = cost;
  diagnostics_.solve_success = solved;

  while (optimizer_.NumNodes() >
         static_cast<std::size_t>(target_window_size_))
  {
    const PolicyFrameVector frames = optimizer_.PolicyFrames();
    std::uint64_t candidate =
        policy_.SelectMarginalizationCandidate(frames, target_window_size_);
    if (candidate == 0 && !frames.empty()) candidate = frames.front().id;
    if (candidate == 0 || !optimizer_.Marginalize(candidate)) break;
    diagnostics_.last_marginalized_id = candidate;
    associator_.RemoveNode(candidate);
    if (previous_new_node_id_ == candidate)
    {
      const auto ids = optimizer_.NodeIds();
      previous_new_node_id_ = ids.empty() ? 0 : ids.back();
    }
  }

  diagnostics_.window_size = optimizer_.NumNodes();
  diagnostics_.factor_count = optimizer_.NumFactors();
  diagnostics_.target_window_size = target_window_size_;
  const ExtrinsicState estimate = optimizer_.EstimateCameraExtrinsic();
  diagnostics_.camera_extrinsic_translation_change =
      (estimate.p_c_i - camera_extrinsic_initial_.p_c_i).norm();
  diagnostics_.camera_extrinsic_rotation_change_deg = Degrees(
      LogQuaternion(camera_extrinsic_initial_.q_c_i.conjugate() *
                    estimate.q_c_i)
          .norm());
  return solved;
}

bool AdaptiveWindowBackend::LatestEstimate(NavState *state) const
{
  if (state == nullptr || latest_node_id_ == 0 ||
      !optimizer_.HasNode(latest_node_id_))
    return false;
  *state = optimizer_.EstimateNode(latest_node_id_);
  return true;
}

ExtrinsicState AdaptiveWindowBackend::CameraExtrinsicEstimate() const
{
  return optimizer_.EstimateCameraExtrinsic();
}

}  // namespace adaptive_window
}  // namespace fast_livo
