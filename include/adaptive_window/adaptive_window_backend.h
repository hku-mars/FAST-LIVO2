#ifndef FAST_LIVO_ADAPTIVE_WINDOW_BACKEND_H_
#define FAST_LIVO_ADAPTIVE_WINDOW_BACKEND_H_

#include "adaptive_window/adaptive_policy.h"
#include "adaptive_window/factor_builders.h"
#include "adaptive_window/fixed_lag_optimizer.h"
#include "adaptive_window/information_evaluator.h"
#include "adaptive_window/temporal_associator.h"
#include "adaptive_window/types.h"

#include <cstdint>

namespace fast_livo
{
namespace adaptive_window
{

class AdaptiveWindowBackend
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit AdaptiveWindowBackend(const AdaptiveWindowConfig &config);

  void Initialize(const LidarExtrinsic &lidar_extrinsic,
                  const ExtrinsicState &camera_extrinsic,
                  ProjectionFunction projection);
  void Reset();

  void AddImuSamples(const ImuSampleVector &samples);

  bool ProcessLidar(const NavState &state,
                    const MeasurementSummary &summary,
                    const LidarPlaneObservationVector &observations);

  bool ProcessVision(const NavState &state,
                     const MeasurementSummary &summary,
                     const CrossModalObservationVector &observations,
                     const ImageSampleFunction &image_sampler =
                         ImageSampleFunction());

  bool LatestEstimate(NavState *state) const;
  ExtrinsicState CameraExtrinsicEstimate() const;
  const BackendDiagnostics &diagnostics() const { return diagnostics_; }
  const AdaptiveWindowConfig &config() const { return config_; }

private:
  std::uint64_t AddOrGetNode(const NavState &state, Modality modality,
                             const MeasurementSummary &summary,
                             const PolicyDecision &decision);
  void AddPoseMeasurement(std::uint64_t node_id, const NavState &state,
                          const MeasurementSummary &summary,
                          double modality_weight);
  void AddImuFactorToNewNode(std::uint64_t new_node_id);
  void ConsumeImuThrough(double timestamp);
  bool OptimizeAndAdapt();
  Eigen::Matrix<double, kPoseDim, kPoseDim> DefaultPoseInformation() const;

  AdaptiveWindowConfig config_;
  InformationEvaluator information_evaluator_;
  AdaptivePolicy policy_;
  TemporalAssociator associator_;
  FixedLagOptimizer optimizer_;

  LidarExtrinsic lidar_extrinsic_;
  ExtrinsicState camera_extrinsic_initial_;
  ProjectionFunction projection_;

  ImuSampleVector pending_imu_;
  MeasurementSummary latest_lidar_summary_;
  MeasurementSummary latest_vision_summary_;
  bool has_lidar_summary_ = false;
  bool has_vision_summary_ = false;

  std::uint64_t latest_node_id_ = 0;
  std::uint64_t previous_new_node_id_ = 0;
  int target_window_size_ = 0;
  BackendDiagnostics diagnostics_;
};

}  // namespace adaptive_window
}  // namespace fast_livo

#endif  // FAST_LIVO_ADAPTIVE_WINDOW_BACKEND_H_
