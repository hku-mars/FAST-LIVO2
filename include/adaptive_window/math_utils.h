#ifndef FAST_LIVO_ADAPTIVE_WINDOW_MATH_UTILS_H_
#define FAST_LIVO_ADAPTIVE_WINDOW_MATH_UTILS_H_

#include "adaptive_window/types.h"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>

namespace fast_livo
{
namespace adaptive_window
{

inline Eigen::Matrix3d Skew(const Eigen::Vector3d &v)
{
  Eigen::Matrix3d result;
  result << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return result;
}

inline Eigen::Quaterniond ExpQuaternion(const Eigen::Vector3d &theta)
{
  const double angle = theta.norm();
  if (angle < 1.0e-10)
  {
    Eigen::Quaterniond q(1.0, 0.5 * theta.x(), 0.5 * theta.y(),
                         0.5 * theta.z());
    return q.normalized();
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, theta / angle));
}

inline Eigen::Vector3d LogQuaternion(const Eigen::Quaterniond &input)
{
  Eigen::Quaterniond q = input.normalized();
  if (q.w() < 0.0) q.coeffs() *= -1.0;
  const Eigen::Vector3d vector_part(q.x(), q.y(), q.z());
  const double vector_norm = vector_part.norm();
  if (vector_norm < 1.0e-10) return 2.0 * vector_part;
  const double angle = 2.0 * std::atan2(vector_norm, q.w());
  return angle * vector_part / vector_norm;
}

inline NavState ApplyIncrement(
    const NavState &state,
    const Eigen::Matrix<double, kNavStateDim, 1> &increment)
{
  NavState result = state;
  result.q_w_i = (state.q_w_i * ExpQuaternion(increment.segment<3>(0))).normalized();
  result.p_w_i += increment.segment<3>(3);
  result.v_w_i += increment.segment<3>(6);
  result.bias_g += increment.segment<3>(9);
  result.bias_a += increment.segment<3>(12);
  return result;
}

inline ExtrinsicState ApplyIncrement(
    const ExtrinsicState &extrinsic,
    const Eigen::Matrix<double, kExtrinsicDim, 1> &increment)
{
  ExtrinsicState result = extrinsic;
  result.q_c_i =
      (extrinsic.q_c_i * ExpQuaternion(increment.segment<3>(0))).normalized();
  result.p_c_i += increment.segment<3>(3);
  return result;
}

inline Eigen::Matrix<double, kNavStateDim, 1> StateDifference(
    const NavState &reference, const NavState &state)
{
  Eigen::Matrix<double, kNavStateDim, 1> difference;
  difference.segment<3>(0) =
      LogQuaternion(reference.q_w_i.conjugate() * state.q_w_i);
  difference.segment<3>(3) = state.p_w_i - reference.p_w_i;
  difference.segment<3>(6) = state.v_w_i - reference.v_w_i;
  difference.segment<3>(9) = state.bias_g - reference.bias_g;
  difference.segment<3>(12) = state.bias_a - reference.bias_a;
  return difference;
}

inline Eigen::Matrix<double, kExtrinsicDim, 1> ExtrinsicDifference(
    const ExtrinsicState &reference, const ExtrinsicState &state)
{
  Eigen::Matrix<double, kExtrinsicDim, 1> difference;
  difference.segment<3>(0) =
      LogQuaternion(reference.q_c_i.conjugate() * state.q_c_i);
  difference.segment<3>(3) = state.p_c_i - reference.p_c_i;
  return difference;
}

inline double Clamp(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(value, upper));
}

inline double Degrees(const double radians)
{
  return radians * 180.0 / 3.14159265358979323846;
}

inline double Radians(const double degrees)
{
  return degrees * 3.14159265358979323846 / 180.0;
}

inline Eigen::MatrixXd SymmetricSqrtInformation(
    const Eigen::MatrixXd &information, const double eigen_floor = 1.0e-12)
{
  if (information.rows() == 0 || information.rows() != information.cols())
    return Eigen::MatrixXd();

  const Eigen::MatrixXd symmetric = 0.5 * (information + information.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(symmetric);
  if (solver.info() != Eigen::Success)
    return Eigen::MatrixXd::Identity(information.rows(), information.cols());

  Eigen::VectorXd eigenvalues = solver.eigenvalues();
  for (int i = 0; i < eigenvalues.size(); ++i)
    eigenvalues(i) = std::sqrt(std::max(eigen_floor, eigenvalues(i)));
  return eigenvalues.asDiagonal() * solver.eigenvectors().transpose();
}

}  // namespace adaptive_window
}  // namespace fast_livo

#endif  // FAST_LIVO_ADAPTIVE_WINDOW_MATH_UTILS_H_
