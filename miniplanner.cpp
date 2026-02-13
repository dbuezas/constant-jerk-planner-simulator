#include "miniplanner.h"

#include <math.h>
#include <string.h>

namespace {

struct Block {
  float millimeters;
  float nominal_speed;
  float acceleration;

  float max_entry_speed_sqr;
  float min_entry_speed_sqr;

  float entry_speed_sqr;
  float exit_speed_sqr;
};

constexpr float sqf(const float x) { return x * x; }

inline float clampf(const float x, const float lo, const float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

inline float max_allowable_speed_sqr(const float accel, const float target_velocity_sqr, const float distance) {
  // Matches Marlin's helper:
  // v^2 = v_target^2 - 2*a*d
  return target_velocity_sqr - 2.0f * accel * distance;
}

struct Planner {
  Block buf[MINIPLANNER_BLOCK_BUFFER_SIZE];
  size_t head = 0;
  size_t tail = 0;
  size_t count = 0;

  void reset() {
    head = tail = count = 0;
    memset(buf, 0, sizeof(buf));
  }

  bool push(const Block &b) {
    if (count >= (MINIPLANNER_BLOCK_BUFFER_SIZE)) return false;
    buf[head] = b;
    head = (head + 1) % MINIPLANNER_BLOCK_BUFFER_SIZE;
    ++count;
    return true;
  }

  bool pop_front() {
    if (!count) return false;
    tail = (tail + 1) % MINIPLANNER_BLOCK_BUFFER_SIZE;
    --count;
    return true;
  }

  Block* at(const size_t index) {
    if (index >= count) return nullptr;
    const size_t pos = (tail + index) % MINIPLANNER_BLOCK_BUFFER_SIZE;
    return &buf[pos];
  }

  const Block* at(const size_t index) const {
    if (index >= count) return nullptr;
    const size_t pos = (tail + index) % MINIPLANNER_BLOCK_BUFFER_SIZE;
    return &buf[pos];
  }

  void recalculate(const float safe_exit_speed_sqr) {
    if (count == 0) return;

    const size_t last = count - 1;

    // Reverse pass: maximize entry speeds from newest to oldest.
    for (size_t rev = 0; rev < count; ++rev) {
      const size_t i = last - rev;
      Block * const current = at(i);
      const float next_entry_sqr = (i == last) ? safe_exit_speed_sqr : at(i + 1)->entry_speed_sqr;

      float new_entry_sqr;
      if (current->acceleration > 0 && current->millimeters > 0) {
        // Passing -accel matches Marlin: next + 2*a*d
        new_entry_sqr = max_allowable_speed_sqr(-current->acceleration, next_entry_sqr, current->millimeters);
      }
      else {
        new_entry_sqr = next_entry_sqr;
      }

      const float nominal_sqr = sqf(current->nominal_speed);
      const float max_entry_sqr = clampf(current->max_entry_speed_sqr, 0.0f, nominal_sqr);
      const float min_entry_sqr = clampf(current->min_entry_speed_sqr, 0.0f, max_entry_sqr);

      new_entry_sqr = clampf(new_entry_sqr, min_entry_sqr, max_entry_sqr);
      current->entry_speed_sqr = new_entry_sqr;

      // keep floors sane for later
      current->max_entry_speed_sqr = max_entry_sqr;
      current->min_entry_speed_sqr = min_entry_sqr;
    }

    // Match Marlin's first-block behavior: start from the min entry speed.
    Block * const first = at(0);
    if (first)
      first->entry_speed_sqr = first->min_entry_speed_sqr;

    // Forward pass: ensure each entry speed is reachable from previous entry.
    for (size_t i = 1; i < count; ++i) {
      const Block * const prev = at(i - 1);
      Block * const curr = at(i);

      if (prev->entry_speed_sqr < curr->entry_speed_sqr) {
        float max_reachable = prev->entry_speed_sqr;
        if (prev->acceleration > 0 && prev->millimeters > 0)
          max_reachable = max_allowable_speed_sqr(-prev->acceleration, prev->entry_speed_sqr, prev->millimeters); // prev + 2*a*d

        const float prev_nominal_sqr = sqf(prev->nominal_speed);
        max_reachable = clampf(max_reachable, 0.0f, prev_nominal_sqr);

        if (max_reachable < curr->entry_speed_sqr)
          curr->entry_speed_sqr = clampf(max_reachable, curr->min_entry_speed_sqr, curr->max_entry_speed_sqr);
      }
    }

    // Propagate exit speeds (each exit is the next block's entry).
    for (size_t i = 0; i < count; ++i) {
      Block * const b = at(i);
      b->exit_speed_sqr = (i == last) ? safe_exit_speed_sqr : at(i + 1)->entry_speed_sqr;

      // Clamp to nominal for sanity.
      const float nominal_sqr = sqf(b->nominal_speed);
      b->entry_speed_sqr = clampf(b->entry_speed_sqr, 0.0f, nominal_sqr);
      b->exit_speed_sqr  = clampf(b->exit_speed_sqr,  0.0f, nominal_sqr);
    }
  }
};

Planner planner;

} // namespace

extern "C" {

void mp_reset(void) {
  planner.reset();
}

int mp_push_block(const float millimeters, const float nominal_speed, const float acceleration,
                  const float max_entry_speed, const float min_entry_speed) {
  if (!(millimeters >= 0.0f) || !(nominal_speed >= 0.0f) || !(acceleration >= 0.0f))
    return 0;

  Block b{};
  b.millimeters = millimeters;
  b.nominal_speed = nominal_speed;
  b.acceleration = acceleration;

  const float nominal_sqr = sqf(nominal_speed);
  const float max_entry_sqr = clampf(sqf(max_entry_speed), 0.0f, nominal_sqr);
  const float min_entry_sqr = clampf(sqf(min_entry_speed), 0.0f, max_entry_sqr);

  b.max_entry_speed_sqr = max_entry_sqr;
  b.min_entry_speed_sqr = min_entry_sqr;

  // Start conservative; recalculate() will raise as allowed.
  b.entry_speed_sqr = min_entry_sqr;
  b.exit_speed_sqr = 0.0f;

  if (!planner.push(b)) return 0;

  // Default to safe exit = 0 until caller overrides.
  planner.recalculate(0.0f);
  return 1;
}

void mp_recalculate(const float safe_exit_speed) {
  planner.recalculate(sqf(safe_exit_speed));
}

size_t mp_size(void) {
  return planner.count;
}

int mp_get_block(const size_t index, MP_BlockOut *out) {
  if (!out) return 0;
  const Block * const b = planner.at(index);
  if (!b) return 0;

  MP_BlockOut tmp{};
  tmp.millimeters = b->millimeters;
  tmp.nominal_speed = b->nominal_speed;
  tmp.acceleration = b->acceleration;

  tmp.max_entry_speed = sqrtf(b->max_entry_speed_sqr);
  tmp.min_entry_speed = sqrtf(b->min_entry_speed_sqr);

  tmp.entry_speed_sqr = b->entry_speed_sqr;
  tmp.exit_speed_sqr = b->exit_speed_sqr;
  tmp.entry_speed = sqrtf(b->entry_speed_sqr);
  tmp.exit_speed  = sqrtf(b->exit_speed_sqr);

  *out = tmp;
  return 1;
}

int mp_pop_front(void) {
  return planner.pop_front() ? 1 : 0;
}

} // extern "C"
