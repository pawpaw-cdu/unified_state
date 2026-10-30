#include "unified_state/timeline.hpp"

#include <atomic>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <vector>

namespace
{
using namespace unified_state;

int failures = 0;

void expect(bool condition, const char * message)
{
  if (condition) return;
  std::cerr << "FAIL: " << message << '\n';
  ++failures;
}

Timestamp at(Nanoseconds nanoseconds)
{
  return {nanoseconds, static_cast<ClockDomain>(1)};
}

Timestamp atOtherClock(Nanoseconds nanoseconds)
{
  return {nanoseconds, static_cast<ClockDomain>(2)};
}

SessionKey session(std::uint64_t value)
{
  return {value, 1, true};
}

struct Pose
{
  double x = 0.0;
  double y = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double yaw = 0.0;
  double omega = 0.0;
};

TimelinePolicy<Pose> posePolicy()
{
  TimelinePolicy<Pose> policy;
  policy.interpolation_model = "linear";
  policy.prediction_model = "constant_velocity";
  policy.interpolate =
    [](const Pose & lower, const Pose & upper, double alpha)
      -> std::optional<Pose> {
    Pose result;
    result.x = (1.0 - alpha) * lower.x + alpha * upper.x;
    result.y = (1.0 - alpha) * lower.y + alpha * upper.y;
    result.vx = (1.0 - alpha) * lower.vx + alpha * upper.vx;
    result.vy = (1.0 - alpha) * lower.vy + alpha * upper.vy;
    result.yaw = (1.0 - alpha) * lower.yaw + alpha * upper.yaw;
    result.omega =
      (1.0 - alpha) * lower.omega + alpha * upper.omega;
    return result;
  };
  policy.predict =
    [](const Pose & state, Nanoseconds delta) -> std::optional<Pose> {
    Pose result = state;
    const double seconds =
      static_cast<double>(delta) / 1.0e9;
    result.x += state.vx * seconds;
    result.y += state.vy * seconds;
    result.yaw += state.omega * seconds;
    return result;
  };
  return policy;
}

StateUpdate<Pose> poseUpdate(
  Nanoseconds state_time, double x, double y,
  bool measurement_backed = true)
{
  StateUpdate<Pose> update;
  update.state_time = at(state_time);
  update.available_time = at(state_time + 2'000'000);
  update.session = session(7);
  update.state = {x, y, 1.0, 2.0, 0.1, 0.2};
  update.measurement_backed = measurement_backed;
  return update;
}

void testInputValidation()
{
  StateTimeline<Pose> timeline({}, posePolicy());

  expect(
    timeline.record({}).error == QueryError::invalid_input,
    "an invalid timestamp was accepted");

  auto update = poseUpdate(500'000'000, 0.0, 0.0);
  update.session = {};
  expect(
    timeline.record(update).error == QueryError::invalid_input,
    "an invalid session was accepted");

  update = poseUpdate(500'000'000, 0.0, 0.0);
  update.available_time = at(499'000'000);
  expect(
    timeline.record(update).error == QueryError::invalid_input,
    "availability before event time was accepted");

  update = poseUpdate(500'000'000, 0.0, 0.0);
  update.available_time = atOtherClock(500'000'000);
  expect(
    timeline.record(update).error == QueryError::clock_mismatch,
    "cross-clock availability was accepted");

  expect(timeline.record(poseUpdate(500'000'000, 0.0, 0.0)).has_value(),
         "valid input was rejected");
  expect(
    timeline.record(poseUpdate(500'000'000, 0.0, 0.0)).error ==
      QueryError::non_monotonic_state_time,
    "duplicate event time was accepted");

  auto wrong_clock = poseUpdate(501'000'000, 0.0, 0.0);
  wrong_clock.state_time = atOtherClock(501'000'000);
  wrong_clock.available_time = atOtherClock(501'000'000);
  expect(
    timeline.record(std::move(wrong_clock)).error ==
      QueryError::clock_mismatch,
    "an implicit clock-domain switch was accepted");

  auto other_session = poseUpdate(501'000'000, 0.0, 0.0);
  other_session.session = session(8);
  expect(
    timeline.record(std::move(other_session)).error ==
      QueryError::session_mismatch,
    "an implicit session switch was accepted");
}

void testHistoryAndPrediction()
{
  StateTimeline<Pose> timeline({}, posePolicy());
  expect(
    timeline.record(poseUpdate(1'000'000'000, 0.0, 0.0)).has_value(),
    "first posterior was rejected");
  expect(
    timeline.record(poseUpdate(1'100'000'000, 1.0, 2.0)).has_value(),
    "second posterior was rejected");

  const auto exact = timeline.exactAt(at(1'000'000'000));
  expect(exact && std::abs(exact.value->state.x) < 1.0e-12,
         "exact lookup failed");

  const auto midpoint = timeline.interpolateAt(at(1'050'000'000));
  expect(midpoint && midpoint.value->source == StateSource::interpolated,
         "interpolation failed");
  expect(
    midpoint &&
      std::abs(midpoint.value->state.x - 0.5) < 1.0e-12,
    "interpolation produced the wrong value");

  const auto predicted = timeline.predictAt(at(1'200'000'000));
  expect(predicted &&
           predicted.value->source == StateSource::predicted &&
           !predicted.value->measurement_backed,
         "prediction provenance is incorrect");
  expect(
    predicted &&
      std::abs(predicted.value->state.x - 1.1) < 1.0e-12,
    "constant-velocity prediction is incorrect");

  expect(timeline.size() == 2,
         "prediction mutated the posterior history");
  expect(
    timeline.predictAt(at(1'700'000'000)).error ==
      QueryError::prediction_horizon_exceeded,
    "prediction horizon was not enforced");
}

void testHistoryQueryErrors()
{
  TimelineConfig config;
  config.max_interpolation_gap = 50'000'000;
  StateTimeline<Pose> timeline(config, posePolicy());
  expect(timeline.record(poseUpdate(1'000'000'000, 0.0, 0.0)).has_value(),
         "history query setup failed");
  expect(timeline.record(poseUpdate(1'100'000'000, 1.0, 0.0)).has_value(),
         "history query setup failed");

  expect(
    timeline.exactAt(at(1'050'000'000)).error ==
      QueryError::exact_sample_not_found,
    "exact lookup invented an interpolated sample");
  expect(
    timeline.interpolateAt(at(1'050'000'000)).error ==
      QueryError::interpolation_gap_exceeded,
    "interpolation gap was not enforced");
  expect(
    timeline.exactAt(at(999'000'000)).error ==
      QueryError::before_oldest,
    "query before history returned a value");
  expect(
    timeline.exactAt(at(1'101'000'000)).error ==
      QueryError::after_newest,
    "query after history extrapolated unexpectedly");
  expect(
    timeline.exactAt(atOtherClock(1'000'000'000)).error ==
      QueryError::clock_mismatch,
    "cross-clock history query was accepted");
  expect(
    timeline.predictAt(atOtherClock(1'200'000'000)).error ==
      QueryError::clock_mismatch,
    "cross-clock prediction was accepted");
}

void testCausalAvailability()
{
  StateTimeline<Pose> timeline({}, posePolicy());
  auto update = poseUpdate(3'000'000'000, 3.0, 4.0);
  update.available_time = at(3'100'000'000);
  expect(timeline.record(std::move(update)).has_value(),
         "delayed update was rejected");

  expect(
    timeline.causalMeasurementAtOrBefore(at(3'050'000'000), 200'000'000)
        .error == QueryError::measurement_not_found,
    "future-available measurement was returned");
  expect(
    timeline.causalMeasurementAtOrBefore(at(3'110'000'000), 200'000'000)
      .has_value(),
    "available measurement was not returned");
  expect(
    timeline.measurementAtOrBefore(at(3'050'000'000), 200'000'000)
      .has_value(),
    "offline measurement semantics were lost");
  expect(
    timeline.causalMeasurementAtOrBefore(at(3'110'000'000), 0).error ==
      QueryError::measurement_not_found,
    "zero max age still returned a measurement");
}

void testSessionsResetAndCapacity()
{
  TimelineConfig config;
  config.max_history = 2;
  StateTimeline<Pose> timeline(config, posePolicy());
  expect(timeline.record(poseUpdate(5'000'000'000, 0.0, 0.0)).has_value(),
         "session setup failed");

  auto other = poseUpdate(5'100'000'000, 1.0, 0.0);
  other.session = session(8);
  expect(timeline.record(std::move(other)).error ==
           QueryError::session_mismatch,
         "implicit session switch was accepted");

  timeline.reset(ResetReason::session_switch);
  const std::uint64_t epoch = timeline.epoch();
  expect(timeline.empty(), "reset did not empty the timeline");
  expect(timeline.record(poseUpdate(6'000'000'000, 0.0, 0.0)).has_value(),
         "record after reset failed");
  expect(timeline.latest()->epoch == epoch &&
           timeline.latest()->revision == 1,
         "revision or epoch did not restart");

  expect(timeline.record(poseUpdate(6'100'000'000, 1.0, 0.0)).has_value(),
         "capacity update failed");
  expect(timeline.record(poseUpdate(6'200'000'000, 2.0, 0.0)).has_value(),
         "capacity update failed");
  expect(timeline.size() == 2, "history capacity was not enforced");
  expect(timeline.oldest()->state_time.nanoseconds == 6'100'000'000,
         "the wrong history entry was evicted");

  TimelineConfig latest_only_config;
  latest_only_config.max_history = 0;
  StateTimeline<Pose> latest_only(latest_only_config, posePolicy());
  expect(latest_only.config().max_history == 1,
         "zero capacity did not normalize to one sample");
  expect(
    latest_only.record(poseUpdate(8'000'000'000, 0.0, 0.0))
      .has_value(),
    "latest-only setup failed");
  expect(
    latest_only.record(poseUpdate(8'100'000'000, 1.0, 0.0))
      .has_value(),
    "latest-only update failed");
  expect(latest_only.size() == 1,
         "latest-only timeline retained older samples");
}

void testModelUnavailable()
{
  StateTimeline<Pose> timeline;
  expect(timeline.record(poseUpdate(7'000'000'000, 0.0, 0.0)).has_value(),
         "record without policy failed");
  expect(timeline.predictAt(at(7'100'000'000)).error ==
           QueryError::model_unavailable,
         "empty prediction policy was not reported");
}

struct Scalar
{
  double value = 0.0;
};

TimelinePolicy<Scalar> scalarPolicy()
{
  TimelinePolicy<Scalar> policy;
  policy.interpolate =
    [](const Scalar & lower, const Scalar & upper, double alpha)
      -> std::optional<Scalar> {
    return Scalar{
      (1.0 - alpha) * lower.value + alpha * upper.value};
  };
  policy.predict =
    [](const Scalar & state, Nanoseconds)
      -> std::optional<Scalar> {
    return state;
  };
  return policy;
}

void testSharedTimelineCausality()
{
  SharedTimeline<Scalar> timeline({}, scalarPolicy());
  StateUpdate<Scalar> update;
  update.state_time = at(20'000'000'000);
  update.available_time = at(20'050'000'000);
  update.session = session(11);
  update.state.value = 5.0;
  update.measurement_backed = true;
  expect(timeline.record(update).has_value(),
         "shared posterior was rejected");

  expect(
    timeline.predictCausallyForControl(
      at(20'040'000'000), 200'000'000, 0).error ==
      QueryError::future_available,
    "shared control query used a future anchor");
  const auto control = timeline.predictCausallyForControl(
    at(20'050'000'000), 200'000'000, 10'000'000);
  expect(control &&
           control.value->state_time.nanoseconds == 20'060'000'000,
         "shared causal prediction is incorrect");

  update.state_time = at(20'100'000'000);
  update.available_time = at(20'102'000'000);
  update.state.value = 6.0;
  update.measurement_backed = false;
  expect(timeline.record(std::move(update)).has_value(),
         "motion-only update was rejected");
  expect(timeline.latest()->source == StateSource::motion_only,
         "motion-only provenance was lost");
  expect(
    timeline.latestMeasurement()->state_time.nanoseconds ==
      20'000'000'000,
    "motion-only state replaced the measurement anchor");
  expect(timeline.latest()->state_time.nanoseconds == 20'100'000'000,
         "latest state was not motion-only");
  expect(
    timeline.predictCausallyForControl(
      at(20'105'000'000), 200'000'000, 0).has_value(),
    "causal control could not use the retained measurement anchor");

  const std::uint64_t epoch = timeline.epoch();
  timeline.reset(ResetReason::target_lost);
  expect(timeline.empty() && timeline.epoch() == epoch + 1,
         "shared reset did not advance the epoch");
}

void testSharedTimelineConcurrentReaders()
{
  SharedTimeline<Scalar> timeline({}, scalarPolicy());
  std::atomic<bool> done{false};
  std::atomic<int> torn{0};
  std::vector<std::thread> readers;
  for (int index = 0; index < 2; ++index) {
    readers.emplace_back([&] {
      while (!done.load(std::memory_order_acquire)) {
        const auto snapshot = timeline.latest();
        if (!snapshot) continue;
        if (snapshot->state.value !=
              static_cast<double>(snapshot->revision) ||
            snapshot->revision == 0) {
          torn.fetch_add(1, std::memory_order_relaxed);
        }
        const auto prediction = timeline.predictCausallyForControl(
          at(30'105'000'000), 200'000'000, 0);
        if (!prediction ||
            !std::isfinite(prediction.value->state.value)) {
          torn.fetch_add(1, std::memory_order_relaxed);
        }
      }
    });
  }

  for (std::uint64_t revision = 1; revision <= 100000; ++revision) {
    StateUpdate<Scalar> update;
    update.state_time = at(30'000'000'000 + revision * 1000);
    update.available_time =
      at(30'001'000'000 + revision * 1000);
    update.session = session(12);
    update.state.value = static_cast<double>(revision);
    update.measurement_backed = true;
    expect(timeline.record(std::move(update)).has_value(),
           "concurrent update was rejected");
  }
  done.store(true, std::memory_order_release);
  for (auto & reader : readers) reader.join();

  expect(torn.load(std::memory_order_relaxed) == 0,
         "readers observed a torn snapshot");
  expect(timeline.publishedCount() == 100000,
         "publication count is incorrect");
}
}  // namespace

int main()
{
  testInputValidation();
  testHistoryAndPrediction();
  testHistoryQueryErrors();
  testCausalAvailability();
  testSessionsResetAndCapacity();
  testModelUnavailable();
  testSharedTimelineCausality();
  testSharedTimelineConcurrentReaders();
  if (failures != 0) {
    std::cerr << failures << " unified-state test(s) failed\n";
    return EXIT_FAILURE;
  }
  std::cout << "unified_state_timeline_test: OK\n";
  return EXIT_SUCCESS;
}
