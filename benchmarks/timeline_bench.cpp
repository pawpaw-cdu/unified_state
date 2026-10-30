#include "unified_state/timeline.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace
{
using namespace unified_state;

volatile double sink = 0.0;

Timestamp at(Nanoseconds nanoseconds)
{
  return {nanoseconds, static_cast<ClockDomain>(1)};
}

struct BenchState
{
  std::array<double, 8> values{};
  std::array<double, 64> covariance{};
};

TimelinePolicy<BenchState> benchPolicy()
{
  TimelinePolicy<BenchState> policy;
  policy.interpolation_model = "linear";
  policy.prediction_model = "constant_velocity";
  policy.interpolate =
    [](const BenchState & lower, const BenchState & upper, double alpha)
      -> std::optional<BenchState> {
    BenchState result;
    for (std::size_t index = 0; index < result.values.size(); ++index) {
      result.values[index] =
        (1.0 - alpha) * lower.values[index] +
        alpha * upper.values[index];
    }
    for (std::size_t index = 0;
         index < result.covariance.size(); ++index) {
      result.covariance[index] =
        (1.0 - alpha) * lower.covariance[index] +
        alpha * upper.covariance[index];
    }
    return result;
  };
  policy.predict =
    [](const BenchState & state, Nanoseconds delta)
      -> std::optional<BenchState> {
    BenchState result = state;
    const double seconds =
      static_cast<double>(delta) / 1.0e9;
    result.values[0] += state.values[4] * seconds;
    result.values[1] += state.values[5] * seconds;
    result.values[2] += state.values[6] * seconds;
    result.values[3] += state.values[7] * seconds;
    return result;
  };
  return policy;
}

BenchState state(std::size_t index)
{
  BenchState value;
  value.values[0] = 0.01 * static_cast<double>(index);
  value.values[4] = 1.0;
  for (std::size_t item = 0; item < value.covariance.size(); ++item) {
    value.covariance[item] =
      item % 9 == 0 ? 1.0e-4 : 0.0;
  }
  return value;
}

StateUpdate<BenchState> update(std::size_t index, Nanoseconds nanoseconds)
{
  StateUpdate<BenchState> value;
  value.state_time = at(nanoseconds);
  value.available_time = at(nanoseconds + 2'000'000);
  value.session = {1, 1, true};
  value.state = state(index);
  value.measurement_backed = true;
  return value;
}

void printPercentiles(
  const std::string & name, std::vector<double> samples)
{
  std::sort(samples.begin(), samples.end());
  const auto pick = [&](double quantile) {
    return samples[static_cast<std::size_t>(
      quantile * static_cast<double>(samples.size() - 1))];
  };
  std::cout << std::left << std::setw(30) << name
            << " p50=" << std::fixed << std::setprecision(4)
            << pick(0.50) << " us, p95=" << pick(0.95)
            << " us, p99=" << pick(0.99) << " us\n";
}

template <typename Function>
void measure(
  const std::string & name, std::size_t iterations,
  Function && function)
{
  std::vector<double> samples;
  samples.reserve(iterations);
  for (std::size_t index = 0; index < iterations; ++index) {
    const auto start = std::chrono::steady_clock::now();
    function(index);
    const auto stop = std::chrono::steady_clock::now();
    samples.push_back(
      std::chrono::duration<double, std::micro>(stop - start)
        .count());
  }
  printPercentiles(name, std::move(samples));
}
}  // namespace

int main()
{
  constexpr std::size_t kWarmup = 20000;
  constexpr std::size_t kRecordIterations = 200000;
  constexpr std::size_t kQueryIterations = 1000000;
  const auto policy = benchPolicy();

  {
    StateTimeline<BenchState> timeline({}, policy);
    for (std::size_t index = 0; index < kWarmup; ++index) {
      timeline.record(
        update(index, 1000 * static_cast<Nanoseconds>(index)));
    }
    measure("timeline record", kRecordIterations,
            [&](std::size_t index) {
              const auto result = timeline.record(update(
                index,
                30'000'000'000 +
                  1000 * static_cast<Nanoseconds>(index)));
              sink += result.value->state.values[0];
            });
    measure("exact history", kQueryIterations,
            [&](std::size_t index) {
              const auto result = timeline.exactAt(at(
                30'000'000'000 +
                  1000 * static_cast<Nanoseconds>(
                    index % kRecordIterations)));
              sink += result.value->state.values[0];
            });
    measure("interpolated history", kQueryIterations,
            [&](std::size_t index) {
              const auto result = timeline.interpolateAt(at(
                30'000'500'000 +
                  1000 * static_cast<Nanoseconds>(
                    index % (kRecordIterations - 1))));
              sink += result.value->state.values[0];
            });
    measure("prediction +50 ms", kQueryIterations,
            [&](std::size_t) {
              const auto result = timeline.predictAt(at(
                30'250'000'000));
              sink += result.value->state.values[0];
            });
  }

  {
    SharedTimeline<BenchState> timeline({}, policy);
    for (std::size_t index = 0; index < kWarmup; ++index) {
      timeline.record(
        update(index, 1000 * static_cast<Nanoseconds>(index)));
    }
    measure("shared publish", kRecordIterations,
            [&](std::size_t index) {
              const auto result = timeline.record(update(
                index,
                30'000'000'000 +
                  1000 * static_cast<Nanoseconds>(index)));
              sink += result.value->state.values[0];
            });
    measure("atomic latest read", kQueryIterations,
            [&](std::size_t) {
              const auto snapshot = timeline.latest();
              sink += snapshot->state.values[0];
            });
    measure("causal control prediction", kQueryIterations,
            [&](std::size_t) {
              const auto result =
                timeline.predictCausallyForControl(
                  at(230'010'000'000), 20'000'000, 10'000'000);
              sink += result.value->state.values[0];
            });
  }

  std::cout << "sink=" << sink << '\n';
  return 0;
}
