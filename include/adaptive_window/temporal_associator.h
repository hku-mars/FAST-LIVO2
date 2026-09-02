#ifndef FAST_LIVO_ADAPTIVE_WINDOW_TEMPORAL_ASSOCIATOR_H_
#define FAST_LIVO_ADAPTIVE_WINDOW_TEMPORAL_ASSOCIATOR_H_

#include "adaptive_window/types.h"

#include <cstdint>
#include <deque>

namespace fast_livo
{
namespace adaptive_window
{

struct TemporalMatch
{
  bool valid = false;
  std::uint64_t node_id = 0;
  double frame_timestamp = 0.0;
  double delta_time = 0.0;
  double confidence = 0.0;
};

class TemporalAssociator
{
public:
  explicit TemporalAssociator(const AdaptiveWindowConfig &config);

  void Add(Modality modality, double timestamp, std::uint64_t node_id);
  TemporalMatch FindNearest(Modality modality, double timestamp);
  void RemoveNode(std::uint64_t node_id);
  void Clear();

private:
  struct Entry
  {
    Modality modality = Modality::kPrior;
    double timestamp = 0.0;
    std::uint64_t node_id = 0;
    bool used = false;
  };

  AdaptiveWindowConfig config_;
  std::deque<Entry> entries_;
};

}  // namespace adaptive_window
}  // namespace fast_livo

#endif  // FAST_LIVO_ADAPTIVE_WINDOW_TEMPORAL_ASSOCIATOR_H_
