#ifndef FAST_LIVO_ADAPTIVE_WINDOW_FIXED_LAG_OPTIMIZER_H_
#define FAST_LIVO_ADAPTIVE_WINDOW_FIXED_LAG_OPTIMIZER_H_

#include "adaptive_window/adaptive_policy.h"
#include "adaptive_window/linear_factor.h"
#include "adaptive_window/types.h"

#include <Eigen/Core>

#include <cstdint>
#include <deque>
#include <map>
#include <vector>

namespace fast_livo
{
namespace adaptive_window
{

class FixedLagOptimizer
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit FixedLagOptimizer(const AdaptiveWindowConfig &config);

  void Reset(const ExtrinsicState &camera_extrinsic);
  void SetCameraExtrinsic(const ExtrinsicState &camera_extrinsic);

  std::uint64_t AddNode(const NavState &state);
  bool HasNode(std::uint64_t node_id) const;
  std::uint64_t FindNodeNear(double timestamp, double tolerance) const;
  NavState EstimateNode(std::uint64_t node_id) const;
  ExtrinsicState EstimateCameraExtrinsic() const;

  void MarkNode(std::uint64_t node_id, Modality modality,
                double unique_information);
  bool AddFactor(const LinearFactor &factor,
                 bool linearized_at_current_estimate = true);

  bool Solve(double *final_cost = nullptr);
  bool Marginalize(std::uint64_t node_id);

  PolicyFrameVector PolicyFrames() const;
  std::vector<std::uint64_t> NodeIds() const;
  std::size_t NumNodes() const { return nodes_.size(); }
  std::size_t NumFactors() const { return factors_.size(); }

  Eigen::Matrix<double, kPoseDim, kPoseDim> PoseInformation(
      std::uint64_t node_id) const;

private:
  struct Node
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::uint64_t id = 0;
    NavState base;
    Eigen::Matrix<double, kNavStateDim, 1> delta =
        Eigen::Matrix<double, kNavStateDim, 1>::Zero();
    double unique_information = 0.0;
    bool has_lidar = false;
    bool has_vision = false;
    bool has_cross_modal = false;
  };

  struct Prior
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::vector<ParameterBlockKey> keys;
    Eigen::MatrixXd hessian;
    Eigen::VectorXd gradient;

    bool empty() const { return keys.empty() || hessian.size() == 0; }
    void clear()
    {
      keys.clear();
      hessian.resize(0, 0);
      gradient.resize(0);
    }
  };

  struct Ordering
  {
    std::vector<ParameterBlockKey> keys;
    std::map<ParameterBlockKey, int> offsets;
    int total_size = 0;
  };

  using NodeDeque =
      std::deque<Node, Eigen::aligned_allocator<Node>>;

  Node *MutableNode(std::uint64_t node_id);
  const Node *FindNode(std::uint64_t node_id) const;
  Eigen::VectorXd CurrentDelta(const ParameterBlockKey &key) const;
  void SetDelta(const ParameterBlockKey &key, const Eigen::VectorXd &value);

  Ordering BuildOrdering() const;
  Ordering BuildOrderingFor(const std::vector<ParameterBlockKey> &keys) const;
  void AccumulateFactor(const LinearFactor &factor, const Ordering &ordering,
                        Eigen::MatrixXd *hessian, Eigen::VectorXd *gradient,
                        double *cost) const;
  void AccumulatePrior(const Prior &prior, const Ordering &ordering,
                       Eigen::MatrixXd *hessian,
                       Eigen::VectorXd *gradient) const;

  AdaptiveWindowConfig config_;
  std::uint64_t next_node_id_ = 1;
  NodeDeque nodes_;
  LinearFactorVector factors_;

  ExtrinsicState camera_extrinsic_base_;
  Eigen::Matrix<double, kExtrinsicDim, 1> camera_extrinsic_delta_ =
      Eigen::Matrix<double, kExtrinsicDim, 1>::Zero();
  Prior prior_;
};

}  // namespace adaptive_window
}  // namespace fast_livo

#endif  // FAST_LIVO_ADAPTIVE_WINDOW_FIXED_LAG_OPTIMIZER_H_
