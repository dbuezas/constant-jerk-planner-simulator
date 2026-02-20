#include "constant-jerk-planner.h"
#include "trajectory_constant_jerk.h"

#include <math.h>
#include <string.h>

namespace {

struct Block {
  float millimeters;
  float max_entry_speed;
  float nominal;
  float a_max;
  float j_max;

  float entry_v;
  float exit_v;
};

struct Planner {
  Block buf[CJP_BLOCK_BUFFER_SIZE];
  size_t head = 0;
  size_t tail = 0;
  size_t count = 0;

  void reset() {
    head = tail = count = 0;
    memset(buf, 0, sizeof(buf));
  }

  bool push(const Block& b) {
    if (count >= CJP_BLOCK_BUFFER_SIZE) return false;
    buf[head] = b;
    head = (head + 1) % CJP_BLOCK_BUFFER_SIZE;
    ++count;
    return true;
  }

  bool pop_front() {
    if (!count) return false;
    tail = (tail + 1) % CJP_BLOCK_BUFFER_SIZE;
    --count;
    return true;
  }

  Block* at(size_t index) {
    if (index >= count) return nullptr;
    return &buf[(tail + index) % CJP_BLOCK_BUFFER_SIZE];
  }

  const Block* at(size_t index) const {
    if (index >= count) return nullptr;
    return &buf[(tail + index) % CJP_BLOCK_BUFFER_SIZE];
  }
};

// Returns the maximum speed reachable at the other end of a block,
// starting from v_from with a=0, ending with a=0.
// Uses ConstantJerkTrajectoryGenerator as a feasibility oracle.
static float max_reachable_speed(float v_from, float mm,
                                  float nominal, float a_max, float j_max) {
  if (mm <= 0.0f) return v_from;

  ConstantJerkTrajectoryGenerator traj;

  // Trapezoidal upper bound (ignoring jerk limits)
  float v_trap = sqrtf(v_from * v_from + 2.0f * a_max * mm);
  float hi = fminf(nominal, v_trap);
  float lo = 0.0f;

  // Quick check: can we reach hi?
  traj.plan(v_from, hi, a_max, j_max, mm, nominal);
  if (traj.getTotalDuration() > 0) return hi;

  // Binary search for max feasible exit speed
  for (int i = 0; i < 32; i++) {
    float mid = 0.5f * (lo + hi);
    traj.plan(v_from, mid, a_max, j_max, mm, nominal);
    if (traj.getTotalDuration() > 0)
      lo = mid;
    else
      hi = mid;
  }
  return lo;
}

Planner planner;

}  // namespace

extern "C" {

void cjp_reset(void) {
  planner.reset();
}

int cjp_push_block(float millimeters, float max_entry_speed, float nominal,
                   float a_max, float j_max) {
  if (!(millimeters > 0.0f) || !(nominal > 0.0f) || !(a_max > 0.0f) || !(j_max > 0.0f))
    return 0;
  if (!(max_entry_speed >= 0.0f))
    return 0;

  Block b{};
  b.millimeters = millimeters;
  b.max_entry_speed = max_entry_speed;
  b.nominal = nominal;
  b.a_max = a_max;
  b.j_max = j_max;
  b.entry_v = 0.0f;
  b.exit_v = 0.0f;

  return planner.push(b) ? 1 : 0;
}

int cjp_recalculate(void) {
  const size_t n = planner.count;
  if (n == 0) return 1;

  const size_t last = n - 1;

  // --- Reverse pass ---
  // Last block exits at v=0 (safe stop).
  // For each block from newest to oldest, compute max feasible entry speed.
  for (size_t rev = 0; rev < n; rev++) {
    size_t i = last - rev;
    Block* b = planner.at(i);

    float required_exit = (i == last) ? 0.0f : planner.at(i + 1)->entry_v;
    float cap = fminf(b->nominal, b->max_entry_speed);

    // Max entry speed: what's the highest v_entry such that
    // plan(v_entry, 0, required_exit, 0, ...) is feasible?
    // Due to time-reversibility with a=0 boundaries, this equals
    // max_reachable_speed(required_exit, mm, ...).
    float max_from_exit = max_reachable_speed(required_exit, b->millimeters,
                                               b->nominal, b->a_max, b->j_max);
    b->entry_v = fminf(cap, max_from_exit);
  }

  // --- Forward pass ---
  // First block starts at v=0.
  planner.at(0)->entry_v = 0.0f;

  for (size_t i = 1; i < n; i++) {
    Block* prev = planner.at(i - 1);
    Block* curr = planner.at(i);

    float max_from_prev = max_reachable_speed(prev->entry_v, prev->millimeters,
                                               prev->nominal, prev->a_max, prev->j_max);
    float cap = fminf(curr->nominal, curr->max_entry_speed);
    float forward_limit = fminf(cap, max_from_prev);

    if (forward_limit < curr->entry_v)
      curr->entry_v = forward_limit;
  }

  // --- Propagate exit speeds ---
  for (size_t i = 0; i < n; i++) {
    Block* b = planner.at(i);
    b->exit_v = (i == last) ? 0.0f : planner.at(i + 1)->entry_v;
  }

  return 1;
}

size_t cjp_size(void) {
  return planner.count;
}

int cjp_get_block(size_t index, CJP_BlockOut* out) {
  if (!out) return 0;
  const Block* b = planner.at(index);
  if (!b) return 0;

  out->millimeters = b->millimeters;
  out->max_entry_speed = b->max_entry_speed;
  out->nominal = b->nominal;
  out->a_max = b->a_max;
  out->j_max = b->j_max;
  out->entry_v = b->entry_v;
  out->exit_v = b->exit_v;

  return 1;
}

int cjp_pop_front(void) {
  return planner.pop_front() ? 1 : 0;
}

}  // extern "C"
