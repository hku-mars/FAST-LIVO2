#include "adaptive_window/linear_factor.h"

#include <algorithm>

namespace fast_livo
{
namespace adaptive_window
{

bool LinearFactor::IsValid() const
{
  if (residual.size() == 0 || keys.empty() || keys.size() != jacobians.size())
    return false;
  if (sqrt_information.rows() != residual.size() ||
      sqrt_information.cols() != residual.size())
    return false;
  for (std::size_t i = 0; i < keys.size(); ++i)
  {
    if (jacobians[i].rows() != residual.size() ||
        jacobians[i].cols() != ParameterBlockSize(keys[i]))
      return false;
  }
  return residual.allFinite() && sqrt_information.allFinite();
}

bool LinearFactor::Touches(const ParameterBlockKey &key) const
{
  return std::find(keys.begin(), keys.end(), key) != keys.end();
}

}  // namespace adaptive_window
}  // namespace fast_livo
