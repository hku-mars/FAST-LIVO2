#include "adaptive_window/adaptive_policy.h"

#include "adaptive_window/math_utils.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fast_livo
{
namespace adaptive_window
{

AdaptivePolicy::AdaptivePolicy(const AdaptiveWindowConfig &config)
    : config_(config), target_window_size_(config.initial_window_size)
{
  target_window_size_ = std::max(
      config_.min_window_size,
      std::min(config_.max_window_size, target_window_size_));
}

double AdaptivePolicy::ModalityQuality(
    const MeasurementSummary *summary) const
{
  if (summary == nullptr || !summary->valid) return 0.0;
  return Clamp(summary->reliability, 0.0, 1.0);
}

ModalityWeights AdaptivePolicy::UpdateWeights(const double lidar_quality,
                                              const double vision_quality)
{
  const double floor = std::max(1.0e-3, config_.min_modality_weight);
  const double lidar_raw = floor + lidar_quality;
  const double vision_raw = floor + vision_quality;
  const double mean = 0.5 * (lidar_raw + vision_raw);

  ModalityWeights target;
  target.lidar = Clamp(lidar_raw / mean, config_.min_modality_weight,
                       config_.max_modality_weight);
  target.vision = Clamp(vision_raw / mean, config_.min_modality_weight,
                        config_.max_modality_weight);
  target.cross_modal = Clamp(
      std::sqrt(std::max(0.0, target.lidar * target.vision)),
      config_.min_modality_weight, config_.max_modality_weight);

  const double alpha = Clamp(config_.weight_smoothing, 0.0, 0.999);
  weights_.lidar = alpha * weights_.lidar + (1.0 - alpha) * target.lidar;
  weights_.vision = alpha * weights_.vision + (1.0 - alpha) * target.vision;
  weights_.cross_modal = alpha * weights_.cross_modal +
                         (1.0 - alpha) * target.cross_modal;
  return weights_;
}

int AdaptivePolicy::UpdateWindowTarget(const double combined_quality,
                                       const double last_solve_time_ms)
{
  const double deficiency = 1.0 - Clamp(combined_quality, 0.0, 1.0);
  const double span = config_.max_window_size - config_.min_window_size;
  int desired = config_.min_window_size +
                static_cast<int>(std::lround(deficiency * span));

  // A soft real-time safeguard. It only contracts the target by one frame per
  // decision, which avoids oscillation when the backend is close to its budget.
  const double nominal_budget_ms = 30.0;
  if (last_solve_time_ms > nominal_budget_ms &&
      desired > config_.min_window_size)
    --desired;

  desired = std::max(config_.min_window_size,
                     std::min(config_.max_window_size, desired));
  if (desired > target_window_size_) ++target_window_size_;
  else if (desired < target_window_size_) --target_window_size_;
  return target_window_size_;
}

PolicyDecision AdaptivePolicy::Decide(
    const NavState &candidate, const MeasurementSummary *lidar,
    const MeasurementSummary *vision, const PolicyFrameVector &window,
    const double last_solve_time_ms)
{
  const double lidar_quality = ModalityQuality(lidar);
  const double vision_quality = ModalityQuality(vision);
  const double combined_quality =
      (lidar != nullptr && vision != nullptr)
          ? 0.5 * (lidar_quality + vision_quality)
          : std::max(lidar_quality, vision_quality);

  PolicyDecision decision;
  decision.weights = UpdateWeights(lidar_quality, vision_quality);
  decision.target_window_size =
      UpdateWindowTarget(combined_quality, last_solve_time_ms);

  if (window.empty())
  {
    decision.select_keyframe = true;
    decision.keyframe_utility = 1.0;
    return decision;
  }

  const NavState &last = window.back().state;
  const double dt = std::max(0.0, candidate.timestamp - last.timestamp);
  const double translation = (candidate.p_w_i - last.p_w_i).norm();
  const double rotation =
      LogQuaternion(last.q_w_i.conjugate() * candidate.q_w_i).norm();

  const double translation_score = Clamp(
      translation / std::max(1.0e-6, config_.keyframe_translation), 0.0,
      1.0);
  const double rotation_score = Clamp(
      Degrees(rotation) / std::max(1.0e-6, config_.keyframe_rotation_deg),
      0.0, 1.0);
  const double time_score = Clamp(
      dt / std::max(config_.min_keyframe_dt, config_.max_keyframe_dt), 0.0,
      1.0);
  const double complement_score =
      (lidar != nullptr && vision != nullptr) ? 1.0 : 0.0;

  decision.keyframe_utility =
      0.30 * std::max(translation_score, rotation_score) +
      0.25 * combined_quality + 0.25 * time_score +
      0.20 * complement_score;

  decision.select_keyframe =
      dt >= config_.max_keyframe_dt ||
      (dt >= config_.min_keyframe_dt &&
       decision.keyframe_utility >= config_.keyframe_utility_threshold);
  return decision;
}

std::uint64_t AdaptivePolicy::SelectMarginalizationCandidate(
    const PolicyFrameVector &window, const int target_window_size) const
{
  if (window.size() <= static_cast<std::size_t>(target_window_size) ||
      window.size() <= 1)
    return 0;

  double best_score = -std::numeric_limits<double>::infinity();
  std::uint64_t best_id = 0;
  const double newest_time = window.back().state.timestamp;

  // Preserve the newest node. All other nodes, including an old cross-modal
  // anchor, may be marginalized; anchors receive a penalty rather than an
  // absolute exemption so the window remains bounded.
  for (std::size_t i = 0; i + 1 < window.size(); ++i)
  {
    const PolicyFrame &frame = window[i];
    const double age = std::max(0.0, newest_time - frame.state.timestamp);
    const double age_score = Clamp(age / std::max(0.1, config_.max_keyframe_dt *
                                                          target_window_size),
                                   0.0, 1.0);

    double redundancy = 0.0;
    if (i > 0 && i + 1 < window.size())
    {
      const double baseline =
          (window[i + 1].state.p_w_i - window[i - 1].state.p_w_i).norm();
      const double detour =
          (frame.state.p_w_i - window[i - 1].state.p_w_i).norm() +
          (window[i + 1].state.p_w_i - frame.state.p_w_i).norm();
      redundancy = Clamp(1.0 - baseline / std::max(1.0e-6, detour), 0.0, 1.0);
    }

    const double unique_penalty = Clamp(frame.unique_information, 0.0, 1.0);
    const double anchor_penalty = frame.has_cross_modal ? 0.20 : 0.0;
    const double score = 0.55 * age_score + 0.35 * redundancy -
                         0.30 * unique_penalty - anchor_penalty;
    if (score > best_score)
    {
      best_score = score;
      best_id = frame.id;
    }
  }
  return best_id;
}

}  // namespace adaptive_window
}  // namespace fast_livo
