#ifndef FAST_LIVO_ADAPTIVE_WINDOW_LINEAR_FACTOR_H_
#define FAST_LIVO_ADAPTIVE_WINDOW_LINEAR_FACTOR_H_

#include "adaptive_window/types.h"

#include <Eigen/Core>

#include <cstdint>
#include <vector>

namespace fast_livo
{
namespace adaptive_window
{

enum class ParameterBlockType
{
  kNavState = 0,
  kCameraExtrinsic = 1
};

struct ParameterBlockKey
{
  ParameterBlockType type = ParameterBlockType::kNavState;
  std::uint64_t id = 0;

  static ParameterBlockKey NavState(std::uint64_t node_id)
  {
    return {ParameterBlockType::kNavState, node_id};
  }

  static ParameterBlockKey CameraExtrinsic()
  {
    return {ParameterBlockType::kCameraExtrinsic, 0};
  }

  bool operator==(const ParameterBlockKey &other) const
  {
    return type == other.type && id == other.id;
  }

  bool operator!=(const ParameterBlockKey &other) const
  {
    return !(*this == other);
  }

  bool operator<(const ParameterBlockKey &other) const
  {
    if (type != other.type)
      return static_cast<int>(type) < static_cast<int>(other.type);
    return id < other.id;
  }
};

inline int ParameterBlockSize(const ParameterBlockKey &key)
{
  return key.type == ParameterBlockType::kNavState ? kNavStateDim
                                                   : kExtrinsicDim;
}

struct LinearFactor
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Modality modality = Modality::kPrior;
  std::vector<ParameterBlockKey> keys;
  Eigen::VectorXd residual;
  std::vector<Eigen::MatrixXd> jacobians;
  Eigen::MatrixXd sqrt_information;
  double huber_delta = 0.0;

  bool IsValid() const;
  bool Touches(const ParameterBlockKey &key) const;
};

using LinearFactorVector =
    std::vector<LinearFactor, Eigen::aligned_allocator<LinearFactor>>;

}  // namespace adaptive_window
}  // namespace fast_livo

#endif  // FAST_LIVO_ADAPTIVE_WINDOW_LINEAR_FACTOR_H_
