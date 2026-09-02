#include "adaptive_window/fixed_lag_optimizer.h"

#include "adaptive_window/math_utils.h"

#include <Eigen/Cholesky>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <utility>

namespace fast_livo
{
namespace adaptive_window
{

FixedLagOptimizer::FixedLagOptimizer(const AdaptiveWindowConfig &config)
    : config_(config)
{
}

void FixedLagOptimizer::Reset(const ExtrinsicState &camera_extrinsic)
{
  next_node_id_ = 1;
  nodes_.clear();
  factors_.clear();
  prior_.clear();
  camera_extrinsic_base_ = camera_extrinsic;
  camera_extrinsic_delta_.setZero();
}

void FixedLagOptimizer::SetCameraExtrinsic(
    const ExtrinsicState &camera_extrinsic)
{
  camera_extrinsic_base_ = camera_extrinsic;
  camera_extrinsic_delta_.setZero();
}

std::uint64_t FixedLagOptimizer::AddNode(const NavState &state)
{
  Node node;
  node.id = next_node_id_++;
  node.base = state;
  nodes_.push_back(node);
  return node.id;
}

FixedLagOptimizer::Node *FixedLagOptimizer::MutableNode(
    const std::uint64_t node_id)
{
  for (Node &node : nodes_)
    if (node.id == node_id) return &node;
  return nullptr;
}

const FixedLagOptimizer::Node *FixedLagOptimizer::FindNode(
    const std::uint64_t node_id) const
{
  for (const Node &node : nodes_)
    if (node.id == node_id) return &node;
  return nullptr;
}

bool FixedLagOptimizer::HasNode(const std::uint64_t node_id) const
{
  return FindNode(node_id) != nullptr;
}

std::uint64_t FixedLagOptimizer::FindNodeNear(const double timestamp,
                                             const double tolerance) const
{
  double best_distance = std::numeric_limits<double>::infinity();
  std::uint64_t best_id = 0;
  for (const Node &node : nodes_)
  {
    const double distance = std::abs(node.base.timestamp - timestamp);
    if (distance <= tolerance && distance < best_distance)
    {
      best_distance = distance;
      best_id = node.id;
    }
  }
  return best_id;
}

NavState FixedLagOptimizer::EstimateNode(const std::uint64_t node_id) const
{
  const Node *node = FindNode(node_id);
  return node == nullptr ? NavState() : ApplyIncrement(node->base, node->delta);
}

ExtrinsicState FixedLagOptimizer::EstimateCameraExtrinsic() const
{
  return ApplyIncrement(camera_extrinsic_base_, camera_extrinsic_delta_);
}

void FixedLagOptimizer::MarkNode(const std::uint64_t node_id,
                                 const Modality modality,
                                 const double unique_information)
{
  Node *node = MutableNode(node_id);
  if (node == nullptr) return;
  node->unique_information =
      std::max(node->unique_information, unique_information);
  if (modality == Modality::kLidar) node->has_lidar = true;
  if (modality == Modality::kVision) node->has_vision = true;
  if (modality == Modality::kCrossModal) node->has_cross_modal = true;
}

Eigen::VectorXd FixedLagOptimizer::CurrentDelta(
    const ParameterBlockKey &key) const
{
  if (key.type == ParameterBlockType::kCameraExtrinsic)
    return camera_extrinsic_delta_;
  const Node *node = FindNode(key.id);
  return node == nullptr ? Eigen::VectorXd() : Eigen::VectorXd(node->delta);
}

void FixedLagOptimizer::SetDelta(const ParameterBlockKey &key,
                                 const Eigen::VectorXd &value)
{
  if (key.type == ParameterBlockType::kCameraExtrinsic)
  {
    if (value.size() == kExtrinsicDim) camera_extrinsic_delta_ = value;
    return;
  }
  Node *node = MutableNode(key.id);
  if (node != nullptr && value.size() == kNavStateDim) node->delta = value;
}

bool FixedLagOptimizer::AddFactor(const LinearFactor &input,
                                  const bool linearized_at_current_estimate)
{
  if (!input.IsValid()) return false;
  for (const ParameterBlockKey &key : input.keys)
  {
    if (key.type == ParameterBlockType::kNavState && !HasNode(key.id))
      return false;
  }

  LinearFactor factor = input;
  if (linearized_at_current_estimate)
  {
    // Convert r(x_current) + J * d(x_current) into an intercept expressed in
    // the persistent first-estimate coordinates: r0 + J * delta.
    for (std::size_t i = 0; i < factor.keys.size(); ++i)
    {
      const Eigen::VectorXd delta = CurrentDelta(factor.keys[i]);
      if (delta.size() != factor.jacobians[i].cols()) return false;
      factor.residual.noalias() -= factor.jacobians[i] * delta;
    }
  }
  factors_.push_back(std::move(factor));
  return true;
}

FixedLagOptimizer::Ordering FixedLagOptimizer::BuildOrdering() const
{
  std::vector<ParameterBlockKey> keys;
  keys.reserve(nodes_.size() + 1);
  for (const Node &node : nodes_)
    keys.push_back(ParameterBlockKey::NavState(node.id));
  keys.push_back(ParameterBlockKey::CameraExtrinsic());
  return BuildOrderingFor(keys);
}

FixedLagOptimizer::Ordering FixedLagOptimizer::BuildOrderingFor(
    const std::vector<ParameterBlockKey> &input_keys) const
{
  Ordering ordering;
  std::set<ParameterBlockKey> seen;
  for (const ParameterBlockKey &key : input_keys)
  {
    if (!seen.insert(key).second) continue;
    ordering.offsets[key] = ordering.total_size;
    ordering.keys.push_back(key);
    ordering.total_size += ParameterBlockSize(key);
  }
  return ordering;
}

void FixedLagOptimizer::AccumulateFactor(
    const LinearFactor &factor, const Ordering &ordering,
    Eigen::MatrixXd *hessian, Eigen::VectorXd *gradient, double *cost) const
{
  if (!factor.IsValid()) return;
  Eigen::VectorXd current_residual = factor.residual;
  for (std::size_t i = 0; i < factor.keys.size(); ++i)
  {
    const Eigen::VectorXd delta = CurrentDelta(factor.keys[i]);
    if (delta.size() != factor.jacobians[i].cols()) return;
    current_residual.noalias() += factor.jacobians[i] * delta;
  }

  const Eigen::VectorXd whitened_current =
      factor.sqrt_information * current_residual;
  double robust_weight = 1.0;
  const double norm = whitened_current.norm();
  if (factor.huber_delta > 0.0 && norm > factor.huber_delta)
    robust_weight = factor.huber_delta / std::max(1.0e-12, norm);
  const double sqrt_robust_weight = std::sqrt(robust_weight);

  const Eigen::VectorXd weighted_intercept =
      sqrt_robust_weight * factor.sqrt_information * factor.residual;
  std::vector<Eigen::MatrixXd> weighted_jacobians;
  weighted_jacobians.reserve(factor.jacobians.size());
  for (const Eigen::MatrixXd &jacobian : factor.jacobians)
    weighted_jacobians.push_back(
        sqrt_robust_weight * factor.sqrt_information * jacobian);

  for (std::size_t i = 0; i < factor.keys.size(); ++i)
  {
    const auto offset_i_it = ordering.offsets.find(factor.keys[i]);
    if (offset_i_it == ordering.offsets.end()) continue;
    const int offset_i = offset_i_it->second;
    const int size_i = ParameterBlockSize(factor.keys[i]);
    gradient->segment(offset_i, size_i).noalias() +=
        weighted_jacobians[i].transpose() * weighted_intercept;

    for (std::size_t j = i; j < factor.keys.size(); ++j)
    {
      const auto offset_j_it = ordering.offsets.find(factor.keys[j]);
      if (offset_j_it == ordering.offsets.end()) continue;
      const int offset_j = offset_j_it->second;
      const int size_j = ParameterBlockSize(factor.keys[j]);
      const Eigen::MatrixXd block =
          weighted_jacobians[i].transpose() * weighted_jacobians[j];
      hessian->block(offset_i, offset_j, size_i, size_j) += block;
      if (i != j)
        hessian->block(offset_j, offset_i, size_j, size_i) +=
            block.transpose();
    }
  }
  if (cost != nullptr)
    *cost += 0.5 * robust_weight * whitened_current.squaredNorm();
}

void FixedLagOptimizer::AccumulatePrior(
    const Prior &prior, const Ordering &ordering, Eigen::MatrixXd *hessian,
    Eigen::VectorXd *gradient) const
{
  if (prior.empty()) return;
  int prior_row = 0;
  for (const ParameterBlockKey &key_i : prior.keys)
  {
    const int size_i = ParameterBlockSize(key_i);
    const auto output_i = ordering.offsets.find(key_i);
    if (output_i == ordering.offsets.end())
    {
      prior_row += size_i;
      continue;
    }
    gradient->segment(output_i->second, size_i) +=
        prior.gradient.segment(prior_row, size_i);

    int prior_col = 0;
    for (const ParameterBlockKey &key_j : prior.keys)
    {
      const int size_j = ParameterBlockSize(key_j);
      const auto output_j = ordering.offsets.find(key_j);
      if (output_j != ordering.offsets.end())
      {
        hessian->block(output_i->second, output_j->second, size_i, size_j) +=
            prior.hessian.block(prior_row, prior_col, size_i, size_j);
      }
      prior_col += size_j;
    }
    prior_row += size_i;
  }
}

bool FixedLagOptimizer::Solve(double *final_cost)
{
  if (nodes_.empty()) return false;
  const Ordering ordering = BuildOrdering();
  if (ordering.total_size == 0) return false;

  double cost = 0.0;
  for (int iteration = 0; iteration < std::max(1, config_.solver_iterations);
       ++iteration)
  {
    Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(
        ordering.total_size, ordering.total_size);
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(ordering.total_size);
    cost = 0.0;

    AccumulatePrior(prior_, ordering, &hessian, &gradient);
    for (const LinearFactor &factor : factors_)
      AccumulateFactor(factor, ordering, &hessian, &gradient, &cost);

    hessian.diagonal().array() += config_.solver_damping;
    Eigen::LDLT<Eigen::MatrixXd> ldlt(hessian);
    if (ldlt.info() != Eigen::Success) return false;
    const Eigen::VectorXd solution = ldlt.solve(-gradient);
    if (ldlt.info() != Eigen::Success || !solution.allFinite()) return false;

    for (const ParameterBlockKey &key : ordering.keys)
    {
      const int offset = ordering.offsets.at(key);
      SetDelta(key, solution.segment(offset, ParameterBlockSize(key)));
    }
  }

  if (final_cost != nullptr) *final_cost = cost;
  return true;
}

bool FixedLagOptimizer::Marginalize(const std::uint64_t node_id)
{
  const ParameterBlockKey marginal_key =
      ParameterBlockKey::NavState(node_id);
  if (!HasNode(node_id)) return false;

  LinearFactorVector involved;
  std::vector<ParameterBlockKey> keys;
  keys.push_back(marginal_key);
  for (const LinearFactor &factor : factors_)
  {
    if (!factor.Touches(marginal_key)) continue;
    involved.push_back(factor);
    keys.insert(keys.end(), factor.keys.begin(), factor.keys.end());
  }
  if (!prior_.empty())
    keys.insert(keys.end(), prior_.keys.begin(), prior_.keys.end());

  // Keep the marginalized block first so the Schur partition is contiguous.
  std::vector<ParameterBlockKey> ordered_keys;
  ordered_keys.push_back(marginal_key);
  std::set<ParameterBlockKey> seen;
  seen.insert(marginal_key);
  for (const ParameterBlockKey &key : keys)
  {
    if (seen.insert(key).second) ordered_keys.push_back(key);
  }
  const Ordering ordering = BuildOrderingFor(ordered_keys);

  Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(
      ordering.total_size, ordering.total_size);
  Eigen::VectorXd gradient = Eigen::VectorXd::Zero(ordering.total_size);
  AccumulatePrior(prior_, ordering, &hessian, &gradient);
  for (const LinearFactor &factor : involved)
    AccumulateFactor(factor, ordering, &hessian, &gradient, nullptr);

  const int marginalized_size = kNavStateDim;
  const int remaining_size = ordering.total_size - marginalized_size;
  Prior new_prior;
  if (remaining_size > 0)
  {
    Eigen::MatrixXd h_mm = hessian.topLeftCorner(marginalized_size,
                                                 marginalized_size);
    h_mm.diagonal().array() += config_.solver_damping;
    const Eigen::MatrixXd h_mr =
        hessian.block(0, marginalized_size, marginalized_size,
                      remaining_size);
    const Eigen::MatrixXd h_rm = h_mr.transpose();
    const Eigen::MatrixXd h_rr = hessian.bottomRightCorner(
        remaining_size, remaining_size);
    const Eigen::VectorXd b_m = gradient.head(marginalized_size);
    const Eigen::VectorXd b_r = gradient.tail(remaining_size);

    Eigen::LDLT<Eigen::MatrixXd> ldlt(h_mm);
    if (ldlt.info() != Eigen::Success) return false;
    new_prior.hessian =
        h_rr - h_rm * ldlt.solve(h_mr);
    new_prior.gradient = b_r - h_rm * ldlt.solve(b_m);
    new_prior.hessian =
        0.5 * (new_prior.hessian + new_prior.hessian.transpose());
    new_prior.keys.assign(ordered_keys.begin() + 1, ordered_keys.end());
  }

  factors_.erase(
      std::remove_if(factors_.begin(), factors_.end(),
                     [&marginal_key](const LinearFactor &factor) {
                       return factor.Touches(marginal_key);
                     }),
      factors_.end());
  nodes_.erase(std::remove_if(nodes_.begin(), nodes_.end(),
                              [node_id](const Node &node) {
                                return node.id == node_id;
                              }),
               nodes_.end());
  prior_ = std::move(new_prior);
  return true;
}

PolicyFrameVector FixedLagOptimizer::PolicyFrames() const
{
  PolicyFrameVector frames;
  frames.reserve(nodes_.size());
  for (const Node &node : nodes_)
  {
    PolicyFrame frame;
    frame.id = node.id;
    frame.state = ApplyIncrement(node.base, node.delta);
    frame.unique_information = node.unique_information;
    frame.has_lidar = node.has_lidar;
    frame.has_vision = node.has_vision;
    frame.has_cross_modal = node.has_cross_modal;
    frames.push_back(frame);
  }
  return frames;
}

std::vector<std::uint64_t> FixedLagOptimizer::NodeIds() const
{
  std::vector<std::uint64_t> ids;
  ids.reserve(nodes_.size());
  for (const Node &node : nodes_) ids.push_back(node.id);
  return ids;
}

Eigen::Matrix<double, kPoseDim, kPoseDim>
FixedLagOptimizer::PoseInformation(const std::uint64_t node_id) const
{
  Eigen::Matrix<double, kPoseDim, kPoseDim> information =
      Eigen::Matrix<double, kPoseDim, kPoseDim>::Zero();
  const ParameterBlockKey key = ParameterBlockKey::NavState(node_id);
  for (const LinearFactor &factor : factors_)
  {
    for (std::size_t i = 0; i < factor.keys.size(); ++i)
    {
      if (factor.keys[i] != key) continue;
      const Eigen::MatrixXd weighted =
          factor.sqrt_information * factor.jacobians[i];
      information.noalias() +=
          weighted.leftCols(kPoseDim).transpose() *
          weighted.leftCols(kPoseDim);
    }
  }
  return information;
}

}  // namespace adaptive_window
}  // namespace fast_livo
