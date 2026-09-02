#ifndef FAST_LIVO_ADAPTIVE_WINDOW_INFORMATION_EVALUATOR_H_
#define FAST_LIVO_ADAPTIVE_WINDOW_INFORMATION_EVALUATOR_H_

#include "adaptive_window/types.h"

#include <array>

namespace fast_livo
{
namespace adaptive_window
{

class InformationEvaluator
{
public:
  explicit InformationEvaluator(const AdaptiveWindowConfig &config);

  MeasurementSummary Evaluate(const MeasurementSummary &measurement);
  double SmoothedInformation(Modality modality) const;

private:
  struct History
  {
    bool initialized = false;
    double smoothed_information = 0.0;
  };

  static std::size_t Index(Modality modality);

  AdaptiveWindowConfig config_;
  std::array<History, 5> history_;
};

}  // namespace adaptive_window
}  // namespace fast_livo

#endif  // FAST_LIVO_ADAPTIVE_WINDOW_INFORMATION_EVALUATOR_H_
