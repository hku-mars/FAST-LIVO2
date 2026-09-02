#ifndef FAST_LIVO_ADAPTIVE_WINDOW_ADAPTIVE_POLICY_H_
#define FAST_LIVO_ADAPTIVE_WINDOW_ADAPTIVE_POLICY_H_

#include "adaptive_window/types.h"

#include <cstdint>
#include <vector>

namespace fast_livo
{
namespace adaptive_window
{

struct PolicyFrame
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::uint64_t id = 0;
  NavState state;
  double unique_information = 0.0;
  bool has_lidar = false;
  bool has_vision = false;
  bool has_cross_modal = false;
};

using PolicyFrameVector =
    std::vector<PolicyFrame, Eigen::aligned_allocator<PolicyFrame>>;

class AdaptivePolicy
{
public:
  explicit AdaptivePolicy(const AdaptiveWindowConfig &config);

  PolicyDecision Decide(const NavState &candidate,
                        const MeasurementSummary *lidar,
                        const MeasurementSummary *vision,
                        const PolicyFrameVector &window,
                        double last_solve_time_ms);

  std::uint64_t SelectMarginalizationCandidate(
      const PolicyFrameVector &window, int target_window_size) const;

private:
  double ModalityQuality(const MeasurementSummary *summary) const;
  ModalityWeights UpdateWeights(double lidar_quality,
                                double vision_quality);
  int UpdateWindowTarget(double combined_quality,
                         double last_solve_time_ms);

  AdaptiveWindowConfig config_;
  ModalityWeights weights_;
  int target_window_size_;
};

}  // namespace adaptive_window
}  // namespace fast_livo

#endif  // FAST_LIVO_ADAPTIVE_WINDOW_ADAPTIVE_POLICY_H_
