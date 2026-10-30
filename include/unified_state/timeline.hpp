#ifndef UNIFIED_STATE_TIMELINE_HPP
#define UNIFIED_STATE_TIMELINE_HPP

// Unified State is an estimator-agnostic causal history and publication
// layer. State is caller-defined; this library owns time, session, provenance,
// retention, query, and reader/writer semantics rather than estimation.

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>

namespace unified_state
{
using Nanoseconds = std::int64_t;

enum class ClockDomain : std::uint32_t
{
  unspecified = 0,
};

struct Timestamp
{
  Nanoseconds nanoseconds = 0;
  ClockDomain clock = ClockDomain::unspecified;
};

// Timestamps may only be compared when they carry the same explicit clock
// domain. Clock conversion and synchronization belong upstream.
inline bool valid(const Timestamp & timestamp)
{
  return timestamp.nanoseconds >= 0 &&
         timestamp.clock != ClockDomain::unspecified;
}

inline bool sameClock(const Timestamp & lhs, const Timestamp & rhs)
{
  return lhs.clock == rhs.clock &&
         lhs.clock != ClockDomain::unspecified;
}

// Identity of one tracking session. A detector label or numeric armor ID must
// not be used as a session key. Domain namespaces equal numeric values from
// different producers.
struct SessionKey
{
  std::uint64_t value = 0;
  std::uint32_t domain = 0;
  bool valid = false;

  bool operator==(const SessionKey & other) const
  {
    return valid == other.valid && value == other.value &&
           domain == other.domain;
  }

  bool operator!=(const SessionKey & other) const
  {
    return !(*this == other);
  }
};

enum class StateSource
{
  posterior,
  motion_only,
  interpolated,
  predicted,
};

enum class ResetReason
{
  none,
  session_switch,
  source_reset,
  target_lost,
  manual,
};

enum class QueryError
{
  none,
  invalid_input,
  clock_mismatch,
  empty_timeline,
  before_oldest,
  after_newest,
  exact_sample_not_found,
  interpolation_gap_exceeded,
  session_mismatch,
  non_monotonic_state_time,
  model_unavailable,
  prediction_horizon_exceeded,
  prediction_before_anchor,
  future_available,
  measurement_not_found,
  anchor_too_old,
};

struct TimelineConfig
{
  // Values are clamped to at least one sample and nonnegative durations.
  std::size_t max_history = 256;
  Nanoseconds max_interpolation_gap = 200'000'000;
  Nanoseconds max_prediction_horizon = 500'000'000;
};

template <typename State>
struct StateUpdate
{
  // state_time is when the state existed; available_time is when all inputs
  // needed to reproduce it were first available.
  Timestamp state_time;
  std::optional<Timestamp> available_time;
  SessionKey session;
  // State owns domain quantities such as a mean, covariance, physical-slot
  // hypothesis, and source-specific quality metadata.
  State state{};
  bool measurement_backed = false;
  int coast_frames = 0;
};

template <typename State>
struct StateSnapshot
{
  // state_time describes the represented instant. anchor_time is the
  // prediction anchor or the upper enclosing record for interpolation.
  // revision is the source record revision; generated query results do not
  // create new history revisions.
  Timestamp state_time;
  Timestamp anchor_time;
  Timestamp available_time;
  std::uint64_t revision = 0;
  std::uint64_t epoch = 0;
  StateSource source = StateSource::posterior;
  bool measurement_backed = false;
  SessionKey session;
  int coast_frames = 0;
  State state{};
};

template <typename State>
struct TimelinePolicy
{
  // Models are injected so the timeline stays independent of an estimator.
  // Returning nullopt reports that the model cannot answer the query.
  using Interpolate =
    std::function<std::optional<State>(
      const State &, const State &, double)>;
  using Predict =
    std::function<std::optional<State>(const State &, Nanoseconds)>;

  Interpolate interpolate;
  Predict predict;
  std::string interpolation_model = "none";
  std::string prediction_model = "none";
};

template <typename State>
struct QueryResult
{
  std::optional<StateSnapshot<State>> value;
  QueryError error = QueryError::none;

  bool has_value() const { return value.has_value(); }
  explicit operator bool() const { return has_value(); }
};

inline const char * toString(StateSource source)
{
  switch (source) {
  case StateSource::posterior: return "posterior";
  case StateSource::motion_only: return "motion_only";
  case StateSource::interpolated: return "interpolated";
  case StateSource::predicted: return "predicted";
  }
  return "unknown";
}

inline const char * toString(ResetReason reason)
{
  switch (reason) {
  case ResetReason::none: return "none";
  case ResetReason::session_switch: return "session_switch";
  case ResetReason::source_reset: return "source_reset";
  case ResetReason::target_lost: return "target_lost";
  case ResetReason::manual: return "manual";
  }
  return "unknown";
}

inline const char * toString(QueryError error)
{
  switch (error) {
  case QueryError::none: return "none";
  case QueryError::invalid_input: return "invalid_input";
  case QueryError::clock_mismatch: return "clock_mismatch";
  case QueryError::empty_timeline: return "empty_timeline";
  case QueryError::before_oldest: return "before_oldest";
  case QueryError::after_newest: return "after_newest";
  case QueryError::exact_sample_not_found:
    return "exact_sample_not_found";
  case QueryError::interpolation_gap_exceeded:
    return "interpolation_gap_exceeded";
  case QueryError::session_mismatch: return "session_mismatch";
  case QueryError::non_monotonic_state_time:
    return "non_monotonic_state_time";
  case QueryError::model_unavailable: return "model_unavailable";
  case QueryError::prediction_horizon_exceeded:
    return "prediction_horizon_exceeded";
  case QueryError::prediction_before_anchor:
    return "prediction_before_anchor";
  case QueryError::future_available: return "future_available";
  case QueryError::measurement_not_found: return "measurement_not_found";
  case QueryError::anchor_too_old: return "anchor_too_old";
  }
  return "unknown";
}

namespace detail
{
template <typename State>
QueryResult<State> errorResult(QueryError error)
{
  QueryResult<State> result;
  result.error = error;
  return result;
}

template <typename State>
QueryResult<State> valueResult(StateSnapshot<State> value)
{
  QueryResult<State> result;
  result.value = std::move(value);
  return result;
}

inline bool addNonnegative(
  const Timestamp & base, Nanoseconds delta, Timestamp * output)
{
  if (!output || !valid(base) || delta < 0) return false;
  if (base.nanoseconds >
      std::numeric_limits<Nanoseconds>::max() - delta) {
    return false;
  }
  *output = base;
  output->nanoseconds += delta;
  return true;
}

inline bool nonnegativeDifference(
  const Timestamp & later, const Timestamp & earlier,
  Nanoseconds * difference)
{
  if (!difference || !sameClock(later, earlier) ||
      later.nanoseconds < earlier.nanoseconds) {
    return false;
  }
  *difference = later.nanoseconds - earlier.nanoseconds;
  return true;
}

template <typename State>
void normalizeConfig(TimelineConfig * config)
{
  config->max_history = std::max<std::size_t>(1, config->max_history);
  config->max_interpolation_gap =
    std::max<Nanoseconds>(0, config->max_interpolation_gap);
  config->max_prediction_horizon =
    std::max<Nanoseconds>(0, config->max_prediction_horizon);
}
}  // namespace detail

template <typename State>
class StateTimeline
{
  static_assert(
    std::is_copy_constructible<State>::value,
    "State must be copy constructible");
  static_assert(
    std::is_move_constructible<State>::value,
    "State must be move constructible");

public:
  using Snapshot = StateSnapshot<State>;
  using Result = QueryResult<State>;

  StateTimeline(
    TimelineConfig config = {}, TimelinePolicy<State> policy = {})
  : config_(std::move(config)), policy_(std::move(policy))
  {
    detail::normalizeConfig<State>(&config_);
  }

  // Append one causal state. Event time must be strictly increasing and the
  // session and clock domain cannot change without an explicit reset.
  Result record(StateUpdate<State> update)
  {
    const Timestamp available =
      update.available_time.value_or(update.state_time);
    if (!valid(update.state_time) || !valid(available) ||
        !update.session.valid) {
      return detail::errorResult<State>(QueryError::invalid_input);
    }
    if (!sameClock(available, update.state_time)) {
      return detail::errorResult<State>(QueryError::clock_mismatch);
    }
    if (available.nanoseconds < update.state_time.nanoseconds) {
      return detail::errorResult<State>(QueryError::invalid_input);
    }

    if (!history_.empty()) {
      const Snapshot & latest = history_.back();
      if (!sameClock(update.state_time, latest.state_time)) {
        return detail::errorResult<State>(QueryError::clock_mismatch);
      }
      if (update.session != latest.session) {
        return detail::errorResult<State>(QueryError::session_mismatch);
      }
      if (update.state_time.nanoseconds <=
          latest.state_time.nanoseconds) {
        return detail::errorResult<State>(
          QueryError::non_monotonic_state_time);
      }
    }

    Snapshot snapshot;
    snapshot.state_time = update.state_time;
    snapshot.anchor_time = update.state_time;
    snapshot.available_time = available;
    snapshot.revision = next_revision_++;
    snapshot.epoch = epoch_;
    snapshot.source = update.measurement_backed
                        ? StateSource::posterior
                        : StateSource::motion_only;
    snapshot.measurement_backed = update.measurement_backed;
    snapshot.session = update.session;
    snapshot.coast_frames = std::max(0, update.coast_frames);
    snapshot.state = std::move(update.state);

    history_.push_back(std::move(snapshot));
    while (history_.size() > config_.max_history) {
      history_.pop_front();
    }
    return detail::valueResult(history_.back());
  }

  // Offline history semantics. These queries do not enforce available_time.
  Result exactAt(const Timestamp & timestamp) const
  {
    return historicalAt(timestamp, true);
  }

  Result interpolateAt(const Timestamp & timestamp) const
  {
    return historicalAt(timestamp, false);
  }

  // Offline association semantics: event time only.
  Result measurementAtOrBefore(
    const Timestamp & query, Nanoseconds max_age) const
  {
    return measurementBefore(query, max_age, false);
  }

  // Online association semantics: both event time and available_time must
  // precede now.
  Result causalMeasurementAtOrBefore(
    const Timestamp & now, Nanoseconds max_age) const
  {
    return measurementBefore(now, max_age, true);
  }

  // Predictions are read-only and are never inserted into history.
  Result predictAt(const Timestamp & timestamp) const
  {
    if (history_.empty()) {
      return detail::errorResult<State>(QueryError::empty_timeline);
    }
    return predictFrom(history_.back(), timestamp);
  }

  Result predictFrom(
    const Snapshot & anchor, const Timestamp & timestamp) const
  {
    if (!valid(anchor.state_time) || !valid(anchor.available_time) ||
        !valid(timestamp)) {
      return detail::errorResult<State>(QueryError::invalid_input);
    }
    if (!sameClock(anchor.state_time, timestamp) ||
        !sameClock(anchor.available_time, timestamp)) {
      return detail::errorResult<State>(QueryError::clock_mismatch);
    }
    if (timestamp.nanoseconds < anchor.state_time.nanoseconds) {
      return detail::errorResult<State>(
        QueryError::prediction_before_anchor);
    }
    if (timestamp.nanoseconds == anchor.state_time.nanoseconds) {
      return detail::valueResult(anchor);
    }

    Nanoseconds delta = 0;
    detail::nonnegativeDifference(
      timestamp, anchor.state_time, &delta);
    if (delta > config_.max_prediction_horizon) {
      return detail::errorResult<State>(
        QueryError::prediction_horizon_exceeded);
    }
    if (!policy_.predict) {
      return detail::errorResult<State>(QueryError::model_unavailable);
    }
    auto predicted = policy_.predict(anchor.state, delta);
    if (!predicted.has_value()) {
      return detail::errorResult<State>(QueryError::model_unavailable);
    }

    Snapshot result = anchor;
    result.state_time = timestamp;
    result.anchor_time = anchor.state_time;
    result.source = StateSource::predicted;
    result.measurement_backed = false;
    result.coast_frames = anchor.coast_frames + 1;
    result.state = std::move(*predicted);
    return detail::valueResult(std::move(result));
  }

  // Adds downstream latency exactly once to now.
  Result predictForControl(
    const Timestamp & now, Nanoseconds downstream_latency) const
  {
    if (!valid(now) || downstream_latency < 0) {
      return detail::errorResult<State>(QueryError::invalid_input);
    }
    Timestamp target;
    if (!detail::addNonnegative(now, downstream_latency, &target) ||
        !valid(target)) {
      return detail::errorResult<State>(QueryError::invalid_input);
    }
    return predictAt(target);
  }

  // Uses the latest measurement-backed state as the anchor, checks that it was
  // available by now, and then predicts for now + downstream_latency.
  Result predictCausallyForControl(
    const Timestamp & now, Nanoseconds max_anchor_age,
    Nanoseconds downstream_latency) const
  {
    if (!valid(now) || max_anchor_age < 0 ||
        downstream_latency < 0) {
      return detail::errorResult<State>(QueryError::invalid_input);
    }

    const Snapshot * anchor = latestMeasurementPointer();
    if (!anchor) {
      return detail::errorResult<State>(
        QueryError::measurement_not_found);
    }
    if (!sameClock(anchor->state_time, now) ||
        !sameClock(anchor->available_time, now)) {
      return detail::errorResult<State>(QueryError::clock_mismatch);
    }
    if (anchor->state_time.nanoseconds > now.nanoseconds ||
        anchor->available_time.nanoseconds > now.nanoseconds) {
      return detail::errorResult<State>(QueryError::future_available);
    }

    Nanoseconds age = 0;
    detail::nonnegativeDifference(now, anchor->state_time, &age);
    if (age > max_anchor_age) {
      return detail::errorResult<State>(QueryError::anchor_too_old);
    }

    Timestamp target;
    if (!detail::addNonnegative(now, downstream_latency, &target) ||
        !valid(target)) {
      return detail::errorResult<State>(QueryError::invalid_input);
    }
    return predictFrom(*anchor, target);
  }

  std::optional<Snapshot> latest() const
  {
    if (history_.empty()) return std::nullopt;
    return history_.back();
  }

  std::optional<Snapshot> latestMeasurement() const
  {
    const Snapshot * result = latestMeasurementPointer();
    if (!result) return std::nullopt;
    return *result;
  }

  std::optional<Snapshot> oldest() const
  {
    if (history_.empty()) return std::nullopt;
    return history_.front();
  }

  const std::deque<Snapshot> & history() const { return history_; }
  const TimelineConfig & config() const { return config_; }
  const TimelinePolicy<State> & policy() const { return policy_; }
  std::size_t size() const { return history_.size(); }
  bool empty() const { return history_.empty(); }
  std::uint64_t epoch() const { return epoch_; }
  ResetReason lastResetReason() const { return last_reset_reason_; }

  void reset(ResetReason reason = ResetReason::manual)
  {
    history_.clear();
    ++epoch_;
    next_revision_ = 1;
    last_reset_reason_ = reason;
  }

private:
  Result historicalAt(
    const Timestamp & timestamp, bool require_exact) const
  {
    if (!valid(timestamp)) {
      return detail::errorResult<State>(QueryError::invalid_input);
    }
    if (history_.empty()) {
      return detail::errorResult<State>(QueryError::empty_timeline);
    }
    if (!sameClock(timestamp, history_.front().state_time)) {
      return detail::errorResult<State>(QueryError::clock_mismatch);
    }
    if (timestamp.nanoseconds < history_.front().state_time.nanoseconds) {
      return detail::errorResult<State>(QueryError::before_oldest);
    }
    if (timestamp.nanoseconds > history_.back().state_time.nanoseconds) {
      return detail::errorResult<State>(QueryError::after_newest);
    }

    const auto upper = std::lower_bound(
      history_.begin(), history_.end(), timestamp.nanoseconds,
      [](const Snapshot & snapshot, Nanoseconds nanoseconds) {
        return snapshot.state_time.nanoseconds < nanoseconds;
      });

    if (upper != history_.begin() &&
        (upper - 1)->state_time.nanoseconds == timestamp.nanoseconds) {
      return detail::valueResult(*(upper - 1));
    }
    if (upper != history_.end() &&
        upper->state_time.nanoseconds == timestamp.nanoseconds) {
      return detail::valueResult(*upper);
    }
    if (require_exact) {
      return detail::errorResult<State>(
        QueryError::exact_sample_not_found);
    }
    if (upper == history_.begin() || upper == history_.end()) {
      return detail::errorResult<State>(
        QueryError::exact_sample_not_found);
    }

    const Snapshot & lower_snapshot = *(upper - 1);
    const Snapshot & upper_snapshot = *upper;
    if (upper_snapshot.session != lower_snapshot.session) {
      return detail::errorResult<State>(QueryError::session_mismatch);
    }
    const Nanoseconds span =
      upper_snapshot.state_time.nanoseconds -
      lower_snapshot.state_time.nanoseconds;
    if (span > config_.max_interpolation_gap) {
      return detail::errorResult<State>(
        QueryError::interpolation_gap_exceeded);
    }
    if (!policy_.interpolate) {
      return detail::errorResult<State>(QueryError::model_unavailable);
    }

    const double alpha = span > 0
      ? static_cast<double>(
          timestamp.nanoseconds -
          lower_snapshot.state_time.nanoseconds) /
          static_cast<double>(span)
      : 0.0;
    auto interpolated = policy_.interpolate(
      lower_snapshot.state, upper_snapshot.state, alpha);
    if (!interpolated.has_value()) {
      return detail::errorResult<State>(QueryError::model_unavailable);
    }

    Snapshot result = upper_snapshot;
    result.state_time = timestamp;
    result.anchor_time = upper_snapshot.state_time;
    result.available_time =
      lower_snapshot.available_time.nanoseconds >=
          upper_snapshot.available_time.nanoseconds
        ? lower_snapshot.available_time
        : upper_snapshot.available_time;
    result.source = StateSource::interpolated;
    result.measurement_backed =
      lower_snapshot.measurement_backed &&
      upper_snapshot.measurement_backed;
    result.coast_frames = std::max(
      lower_snapshot.coast_frames, upper_snapshot.coast_frames);
    result.state = std::move(*interpolated);
    return detail::valueResult(std::move(result));
  }

  Result measurementBefore(
    const Timestamp & query, Nanoseconds max_age,
    bool require_causal_availability) const
  {
    if (!valid(query) || max_age < 0) {
      return detail::errorResult<State>(QueryError::invalid_input);
    }
    if (history_.empty()) {
      return detail::errorResult<State>(QueryError::empty_timeline);
    }
    if (!sameClock(query, history_.back().state_time)) {
      return detail::errorResult<State>(QueryError::clock_mismatch);
    }

    for (auto iterator = history_.rbegin();
         iterator != history_.rend(); ++iterator) {
      if (iterator->state_time.nanoseconds > query.nanoseconds) continue;
      const Nanoseconds age =
        query.nanoseconds - iterator->state_time.nanoseconds;
      if (age > max_age) break;
      if (!iterator->measurement_backed) continue;
      if (require_causal_availability &&
          (!sameClock(iterator->available_time, query) ||
           iterator->available_time.nanoseconds >
             query.nanoseconds)) {
        continue;
      }
      return detail::valueResult(*iterator);
    }
    return detail::errorResult<State>(
      QueryError::measurement_not_found);
  }

  const Snapshot * latestMeasurementPointer() const
  {
    for (auto iterator = history_.rbegin();
         iterator != history_.rend(); ++iterator) {
      if (iterator->measurement_backed) return &*iterator;
    }
    return nullptr;
  }

  TimelineConfig config_;
  TimelinePolicy<State> policy_;
  std::deque<Snapshot> history_;
  std::uint64_t next_revision_ = 1;
  std::uint64_t epoch_ = 1;
  ResetReason last_reset_reason_ = ResetReason::none;
};

// One writer, multiple readers. record() and reset() must only be called by
// the same writer thread. Readers never access the history deque. The atomic
// shared_ptr publication is thread-safe but is not claimed to be lock-free.
template <typename State>
class SharedTimeline
{
public:
  using Snapshot = StateSnapshot<State>;
  using Result = QueryResult<State>;

  explicit SharedTimeline(
    TimelineConfig config = {}, TimelinePolicy<State> policy = {})
  : timeline_(std::move(config), std::move(policy))
  {
  }

  Result record(StateUpdate<State> update)
  {
    Result result = timeline_.record(std::move(update));
    if (!result) return result;

    const auto published =
      std::make_shared<const Snapshot>(*result.value);
    if (published->measurement_backed) {
      std::atomic_store_explicit(
        &latest_measurement_, published, std::memory_order_release);
    }
    // Publish the measurement anchor first. A reader that can observe a
    // non-empty latest snapshot is therefore not exposed to a transient
    // "latest exists, measurement pointer does not" state on first record.
    std::atomic_store_explicit(
      &latest_, published, std::memory_order_release);
    published_count_.fetch_add(1, std::memory_order_relaxed);
    history_size_.store(
      timeline_.size(), std::memory_order_relaxed);
    return result;
  }

  std::shared_ptr<const Snapshot> latest() const
  {
    return std::atomic_load_explicit(
      &latest_, std::memory_order_acquire);
  }

  std::shared_ptr<const Snapshot> latestMeasurement() const
  {
    return std::atomic_load_explicit(
      &latest_measurement_, std::memory_order_acquire);
  }

  Result predictAt(const Timestamp & timestamp) const
  {
    const auto anchor = latest();
    if (!anchor) {
      return detail::errorResult<State>(QueryError::empty_timeline);
    }
    return timeline_.predictFrom(*anchor, timestamp);
  }

  Result predictCausallyForControl(
    const Timestamp & now, Nanoseconds max_anchor_age,
    Nanoseconds downstream_latency) const
  {
    if (!valid(now) || max_anchor_age < 0 ||
        downstream_latency < 0) {
      return detail::errorResult<State>(QueryError::invalid_input);
    }
    const auto anchor = latestMeasurement();
    if (!anchor) {
      return detail::errorResult<State>(
        QueryError::measurement_not_found);
    }
    if (!sameClock(anchor->state_time, now) ||
        !sameClock(anchor->available_time, now)) {
      return detail::errorResult<State>(QueryError::clock_mismatch);
    }
    if (anchor->state_time.nanoseconds > now.nanoseconds ||
        anchor->available_time.nanoseconds > now.nanoseconds) {
      return detail::errorResult<State>(QueryError::future_available);
    }

    const Nanoseconds age =
      now.nanoseconds - anchor->state_time.nanoseconds;
    if (age > max_anchor_age) {
      return detail::errorResult<State>(QueryError::anchor_too_old);
    }

    Timestamp target;
    if (!detail::addNonnegative(
          now, downstream_latency, &target) ||
        !valid(target)) {
      return detail::errorResult<State>(QueryError::invalid_input);
    }
    return timeline_.predictFrom(*anchor, target);
  }

  bool empty() const { return latest() == nullptr; }
  std::size_t size() const
  {
    return history_size_.load(std::memory_order_acquire);
  }
  std::uint64_t publishedCount() const
  {
    return published_count_.load(std::memory_order_acquire);
  }
  std::uint64_t epoch() const
  {
    return epoch_.load(std::memory_order_acquire);
  }
  ResetReason lastResetReason() const
  {
    return last_reset_reason_.load(std::memory_order_acquire);
  }
  const TimelineConfig & config() const { return timeline_.config(); }
  const TimelinePolicy<State> & policy() const
  {
    return timeline_.policy();
  }

  void reset(ResetReason reason = ResetReason::manual)
  {
    timeline_.reset(reason);
    const std::shared_ptr<const Snapshot> empty;
    std::atomic_store_explicit(
      &latest_measurement_, empty, std::memory_order_release);
    std::atomic_store_explicit(
      &latest_, empty, std::memory_order_release);
    history_size_.store(0, std::memory_order_relaxed);
    epoch_.store(timeline_.epoch(), std::memory_order_release);
    last_reset_reason_.store(reason, std::memory_order_release);
  }

private:
  StateTimeline<State> timeline_;
  std::shared_ptr<const Snapshot> latest_;
  std::shared_ptr<const Snapshot> latest_measurement_;
  std::atomic<std::uint64_t> published_count_{0};
  std::atomic<std::size_t> history_size_{0};
  std::atomic<std::uint64_t> epoch_{1};
  std::atomic<ResetReason> last_reset_reason_{ResetReason::none};
};

}  // namespace unified_state

#endif  // UNIFIED_STATE_TIMELINE_HPP
