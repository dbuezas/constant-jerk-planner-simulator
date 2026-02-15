#include <cmath>
#include <cstdio>

#define PROPAGATOR_TEST
#include "constant-jerk-planner.cpp"

namespace {

void run_case(const char *label, State s, float length, float a_max, float j_max, int jerk_sign, bool expect_complete) {
  const PropagationResult result = propagate_extremal(s, length, a_max, j_max, jerk_sign);
  std::printf("%s: completed=%d v=%.6f a=%.6f\n", label, result.completed ? 1 : 0, result.state.v, result.state.a);

  if (result.completed == expect_complete) return;

  std::printf("  EXPECTED completed=%d\n", expect_complete ? 1 : 0);
}

bool approx_between(const float value, const float lo, const float hi) {
  return value >= lo && value <= hi;
}

void test_reach_backward_basic() {
  Planner planner;
  planner.reset();

  const float length = 10.0f;
  const float a_max = 50.0f;
  const float j_max = 100.0f;
  const float v_cap = 100.0f;

  Block b{};
  b.millimeters = length;
  b.max_entry_speed = v_cap;
  b.nominal = v_cap;
  b.a_max = a_max;
  b.j_max = j_max;
  planner.push(b);

  planner.build_velocity_grid();

  const float dv = (CJP_VELOCITY_BINS > 1) ? (planner.v_grid[1] - planner.v_grid[0]) : 1.0f;
  float target_min[CJP_VELOCITY_BINS];
  float target_max[CJP_VELOCITY_BINS];
  clear_bounds(target_min, target_max);

  for (size_t k = 0; k < CJP_VELOCITY_BINS; ++k) {
    target_min[k] = -a_max;
    target_max[k] = a_max;
  }

  float pre_min[CJP_VELOCITY_BINS];
  float pre_max[CJP_VELOCITY_BINS];
  planner.reach_backward(target_min, target_max, length, a_max, j_max, v_cap, pre_min, pre_max);

  size_t any = 0;
  for (size_t k = 0; k < CJP_VELOCITY_BINS; ++k) {
    if (has_bin(pre_min[k], pre_max[k])) any++;
  }

  if (any == 0) {
    std::printf("reach_backward_basic: empty\n");
  } else {
    std::printf("reach_backward_basic: ok bins=%zu\n", any);
  }
}

void run_all() {
  run_case("pos_v_pos_a", State{10.0f, 5.0f}, 5.0f, 50.0f, 100.0f, 1, true);
  run_case("pos_v_neg_a", State{10.0f, -20.0f}, 5.0f, 50.0f, 100.0f, -1, false);
  run_case("zero_v_neg_a", State{0.0f, -5.0f}, 5.0f, 50.0f, 100.0f, -1, false);
  run_case("zero_v_pos_a", State{0.0f, 5.0f}, 5.0f, 50.0f, 100.0f, 1, true);
  run_case("small_v_big_neg_a", State{0.1f, -50.0f}, 10.0f, 50.0f, 100.0f, -1, false);
  run_case("small_v_pos_a_neg_j", State{0.1f, 20.0f}, 10.0f, 50.0f, 200.0f, -1, false);
  run_case("pos_v_pos_a_neg_j", State{5.0f, 10.0f}, 10.0f, 50.0f, 200.0f, -1, false);

  run_case("pos_v_small_neg_a", State{10.0f, -2.0f}, 5.0f, 50.0f, 100.0f, 1, true);
  run_case("pos_v_zero_a_neg_j", State{10.0f, 0.0f}, 5.0f, 50.0f, 100.0f, -1, false);
  run_case("pos_v_pos_a_small_neg_j", State{10.0f, 5.0f}, 5.0f, 50.0f, 50.0f, -1, true);

  test_reach_backward_basic();
}

} // namespace

int main() {
  run_all();
  return 0;
}
