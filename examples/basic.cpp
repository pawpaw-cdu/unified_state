#include "unified_state/timeline.hpp"

#include <cstdint>
#include <iostream>
#include <optional>

namespace
{
using namespace unified_state;

constexpr ClockDomain kMonotonicClock = static_cast<ClockDomain>(1);

struct PlanarState
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double omega = 0.0;
  double covariance[36]{};
};

Timestamp at(Nanoseconds nanoseconds)
{
  return {nanoseconds, kMonotonicClock};
}

TimelinePolicy<PlanarState> makePolicy()
{
  TimelinePolicy<PlanarState> policy;
  policy.interpolation_model = "linear-planar";
  policy.prediction_model = "constant-velocity-planar";
  policy.interpolate =
    [](const PlanarState & lower, const PlanarState & upper, double alpha)
      -> std::optional<PlanarState> {
    PlanarState result;
    const auto mix = [alpha](double lhs, double rhs) {
      return (1.0 - alpha) * lhs + alpha * rhs;
    };
    result.x = mix(lower.x, upper.x);
    result.y = mix(lower.y, upper.y);
    result.yaw = mix(lower.yaw, upper.yaw);
    result.vx = mix(lower.vx, upper.vx);
    result.vy = mix(lower.vy, upper.vy);
    result.omega = mix(lower.omega, upper.omega);
    return result;
  };
  policy.predict =
    [](const PlanarState & state, Nanoseconds delta)
      -> std::optional<PlanarState> {
    PlanarState result = state;
    const double seconds = static_cast<double>(delta) / 1.0e9;
    result.x += state.vx * seconds;
    result.y += state.vy * seconds;
    result.yaw += state.omega * seconds;
    return result;
  };
  return policy;
}
}  // namespace

int main()
{
  SharedTimeline<PlanarState> timeline({}, makePolicy());

  StateUpdate<PlanarState> update;
  update.state_time = at(1'000'000'000);
  update.available_time = at(1'002'000'000);
  update.session = {42, 1, true};
  update.state = {1.0, 2.0, 0.5, 2.0, 0.0, 0.1, {}};
  update.measurement_backed = true;

  const auto recorded = timeline.record(std::move(update));
  if (!recorded) {
    std::cerr << "record failed: "
              << toString(recorded.error) << '\n';
    return 1;
  }

  const auto command = timeline.predictCausallyForControl(
    at(1'004'000'000), 10'000'000, 12'000'000);
  if (!command) {
    std::cerr << "control query failed: "
              << toString(command.error) << '\n';
    return 1;
  }

  std::cout
    << "state_time_ns=" << command.value->state_time.nanoseconds
    << " x=" << command.value->state.x
    << " y=" << command.value->state.y
    << " source=" << toString(command.value->source)
    << " revision=" << command.value->revision
    << '\n';
  return 0;
}
