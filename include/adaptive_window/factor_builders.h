#ifndef FAST_LIVO_ADAPTIVE_WINDOW_FACTOR_BUILDERS_H_
#define FAST_LIVO_ADAPTIVE_WINDOW_FACTOR_BUILDERS_H_

#include "adaptive_window/linear_factor.h"
#include "adaptive_window/types.h"

#include <Eigen/Core>

#include <cstdint>
#include <functional>

namespace fast_livo
{
namespace adaptive_window
{

using ProjectionFunction = std::function<bool(
    const Eigen::Vector3d &, Eigen::Vector2d *,
    Eigen::Matrix<double, 2, 3> *)>;
using ImageSampleFunction = std::function<bool(
    const Eigen::Vector2d &, double *, Eigen::Vector2d *)>;

class FactorBuilders
{
public:
  static LinearFactor PosePrior(
      std::uint64_t node_id, const NavState &linearization_state,
      const NavState &measurement,
      const Eigen::Matrix<double, kPoseDim, kPoseDim> &information,
      Modality modality, double modality_weight);

  static LinearFactor ExtrinsicPrior(
      const ExtrinsicState &linearization_extrinsic,
      const ExtrinsicState &prior_extrinsic,
      double rotation_sigma_radians, double translation_sigma);

  static LinearFactor ImuPreintegration(
      std::uint64_t node_i, std::uint64_t node_j,
      const NavState &state_i, const NavState &state_j,
      const ImuSampleVector &samples, const AdaptiveWindowConfig &config);

  static LinearFactor LidarPointToPlane(
      std::uint64_t node_id, const NavState &state,
      const LidarExtrinsic &lidar_extrinsic,
      const LidarPlaneObservation &observation, double modality_weight,
      double huber_delta);

  static LinearFactor CrossModalReprojection(
      std::uint64_t lidar_node_id, std::uint64_t image_node_id,
      const NavState &lidar_state, const NavState &image_state,
      const LidarExtrinsic &lidar_extrinsic,
      const ExtrinsicState &camera_extrinsic,
      const CrossModalObservation &observation,
      const ProjectionFunction &projection, double modality_weight,
      double temporal_confidence, double huber_delta);

  static LinearFactor CrossModalPhotometric(
      std::uint64_t lidar_node_id, std::uint64_t image_node_id,
      const NavState &lidar_state, const NavState &image_state,
      const LidarExtrinsic &lidar_extrinsic,
      const ExtrinsicState &camera_extrinsic,
      const CrossModalObservation &observation,
      const ProjectionFunction &projection,
      const ImageSampleFunction &image_sampler, double modality_weight,
      double temporal_confidence, double huber_delta);
};

}  // namespace adaptive_window
}  // namespace fast_livo

#endif  // FAST_LIVO_ADAPTIVE_WINDOW_FACTOR_BUILDERS_H_
