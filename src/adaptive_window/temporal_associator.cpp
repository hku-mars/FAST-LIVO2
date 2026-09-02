#include "adaptive_window/temporal_associator.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fast_livo
{
namespace adaptive_window
{

TemporalAssociator::TemporalAssociator(const AdaptiveWindowConfig &config)
    : config_(config)
{
}

void TemporalAssociator::Add(const Modality modality, const double timestamp,
                             const std::uint64_t node_id)
{
  entries_.push_back({modality, timestamp, node_id, false});
  const std::size_t capacity =
      static_cast<std::size_t>(std::max(16, 4 * config_.max_window_size));
  while (entries_.size() > capacity) entries_.pop_front();
}

TemporalMatch TemporalAssociator::FindNearest(const Modality modality,
                                              const double timestamp)
{
  auto best = entries_.end();
  double best_dt = std::numeric_limits<double>::infinity();
  for (auto it = entries_.begin(); it != entries_.end(); ++it)
  {
    if (it->modality != modality || (!config_.allow_pair_reuse && it->used))
      continue;
    const double dt = std::abs(it->timestamp - timestamp);
    if (dt < best_dt)
    {
      best_dt = dt;
      best = it;
    }
  }

  TemporalMatch match;
  if (best == entries_.end() || best_dt > config_.association_max_dt)
    return match;

  match.valid = true;
  match.node_id = best->node_id;
  match.frame_timestamp = best->timestamp;
  match.delta_time = best->timestamp - timestamp;
  const double sigma = std::max(1.0e-6, config_.association_time_sigma);
  match.confidence = std::exp(-0.5 * best_dt * best_dt / (sigma * sigma));
  best->used = true;
  return match;
}

void TemporalAssociator::RemoveNode(const std::uint64_t node_id)
{
  entries_.erase(
      std::remove_if(entries_.begin(), entries_.end(),
                     [node_id](const Entry &entry) {
                       return entry.node_id == node_id;
                     }),
      entries_.end());
}

void TemporalAssociator::Clear()
{
  entries_.clear();
}

}  // namespace adaptive_window
}  // namespace fast_livo
