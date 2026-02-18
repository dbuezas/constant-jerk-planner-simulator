#include <cmath>
#include <cstdio>

#include "marlin files/trajectory_constant_jerk.h"

namespace {

bool approx_ge(const float a, const float b, const float eps = 1e-3f) {
  return a + eps >= b;
}

bool approx_le(const float a, const float b, const float eps = 1e-3f) {
  return a <= b + eps;
}

void sample_profile(ConstantJerkTrajectoryGenerator &traj, float duration, float v_min, float v_max) {
  const int steps = 2000;
  for (int i = 0; i <= steps; ++i) {
    const float t = duration * (static_cast<float>(i) / static_cast<float>(steps));
    const float x = traj.getDistanceAtTime(t);
    (void)x;
  }
}

void run_case(const char *label,
              float v0, float a0,
              float v1, float a1,
              float a_max, float j,
              float distance,
              float v_nominal) {
  ConstantJerkTrajectoryGenerator traj;
  traj.plan(v0, a0, v1, a1, a_max, j, distance, v_nominal);
  const float duration = traj.getTotalDuration();
  std::printf("%s: duration=%.6f\n", label, duration);
  if (duration <= 0.0f) return;

  if (duration < 1e-3f) {
    std::printf("  duration too short for stable v estimate\n");
    std::printf("  FAIL: duration too short\n");
    return;
  }

  const int steps = 4000;
  float v_min = 1e30f;
  float v_max = -1e30f;
  for (int i = 0; i <= steps; ++i) {
    const float t = duration * (static_cast<float>(i) / static_cast<float>(steps));
    const float v = traj.getVelocityAtTime(t);
    if (v < v_min) v_min = v;
    if (v > v_max) v_max = v;
  }

  std::printf("  v_min=%.6f v_max=%.6f v_nominal=%.6f\n", v_min, v_max, v_nominal);
  if (!approx_ge(v_min, 0.0f))
    std::printf("  FAIL: v_min < 0\n");
  if (!approx_le(v_max, v_nominal))
    std::printf("  FAIL: v_max > nominal\n");
}

} // namespace

int main() {
  run_case("basic_symmetric", 0.0f, 0.0f, 0.0f, 0.0f, 500.0f, 8000.0f, 35.0f, 200.0f);
  run_case("no_const_accel", 0.0f, 0.0f, 0.0f, 0.0f, 500.0f, 8000.0f, 5.0f, 200.0f);
  run_case("a0_above", 0.0f, 800.0f, 0.0f, 0.0f, 500.0f, 8000.0f, 35.0f, 200.0f);
  run_case("a1_below", 0.0f, 0.0f, 0.0f, -800.0f, 500.0f, 8000.0f, 35.0f, 200.0f);
  run_case("short_distance", 0.0f, 0.0f, 0.0f, 0.0f, 500.0f, 8000.0f, 2.0f, 200.0f);
  run_case("nominal_20", 0.0f, 0.0f, 0.0f, 0.0f, 500.0f, 8000.0f, 35.0f, 20.0f);
  run_case("nominal_22", 0.0f, 0.0f, 0.0f, 0.0f, 500.0f, 8000.0f, 35.0f, 22.0f);
  run_case("nominal_26", 0.0f, 0.0f, 0.0f, 0.0f, 500.0f, 8000.0f, 35.0f, 26.0f);
  run_case("nominal_62", 0.0f, 0.0f, 0.0f, 0.0f, 500.0f, 8000.0f, 35.0f, 62.0f);
  run_case("nominal_63", 0.0f, 0.0f, 0.0f, 0.0f, 500.0f, 8000.0f, 35.0f, 63.0f);

  for (int v = 1; v <= 100; ++v) {
    const float v_nom = static_cast<float>(v);
    run_case("nominal_sweep", 0.0f, 0.0f, 0.0f, 0.0f, 500.0f, 8000.0f, 200.0f, v_nom);
  }
  return 0;
}
