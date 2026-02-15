#include "constant-jerk-planner.h"

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
  float entry_a;
  float exit_v;
  float exit_a;
};

struct State {
  float v;
  float a;
};

constexpr float kInf = 1e30f;

inline float clampf(const float x, const float lo, const float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

struct PropagationResult {
  State state;
  bool completed = false;
};

bool would_cross_zero(const float v0, const float a0, const float j, const float dt, float &t_zero) {
  if (dt <= 0.0f) return false;

  const float v_eps = 1e-6f;

  if (fabsf(v0) <= v_eps) {
    t_zero = 0.0f;
    return (a0 < 0.0f) || (fabsf(a0) <= v_eps && j < 0.0f);
  }

  const float v_end = v0 + a0 * dt + 0.5f * j * dt * dt;
  float v_min = fminf(v0, v_end);

  if (fabsf(j) > 1e-9f) {
    const float t_vertex = -a0 / j;
    if (t_vertex > 0.0f && t_vertex < dt) {
      const float v_vertex = v0 + a0 * t_vertex + 0.5f * j * t_vertex * t_vertex;
      if (v_vertex < v_min) v_min = v_vertex;
    }
  }

  if (v_min >= -v_eps) return false;

  if (fabsf(j) < 1e-9f) {
    if (a0 >= 0.0f) return false;
    const float t = -v0 / a0;
    if (t > 0.0f && t <= dt + v_eps) {
      t_zero = t;
      return true;
    }
    return false;
  }

  const float disc = a0 * a0 - 2.0f * j * v0;
  if (disc < 0.0f) return false;

  const float sqrt_disc = sqrtf(disc);
  float t = (-a0 - copysignf(sqrt_disc, j)) / j;
  if (t <= 0.0f || t > dt) {
    t = (-a0 + copysignf(sqrt_disc, j)) / j;
  }

  if (t > 0.0f && t <= dt + v_eps) {
    t_zero = t;
    return true;
  }
  return false;
}

PropagationResult propagate_extremal(State s, const float length, const float a_max, const float j_max, const int jerk_sign) {
  float dist = 0.0f;
  int steps = 0;
  while (dist < length && steps < 2000) {
    const float remaining = length - dist;
    const float v = s.v > 0.0f ? s.v : 0.0f;
    float dt = 0.0f;
    if (v > 1e-3f) {
      dt = remaining / v;
    } else {
      dt = sqrtf(2.0f * remaining / fmaxf(a_max, 1e-3f));
    }
    if (dt > 0.01f) dt = 0.01f;

    float j = static_cast<float>(jerk_sign) * j_max;
    if ((s.a >= a_max && j > 0.0f) || (s.a <= -a_max && j < 0.0f))
      j = 0.0f;

    const float a0 = s.a;
    const float v0 = s.v > 0.0f ? s.v : 0.0f;

    float t_zero = 0.0f;
    if (would_cross_zero(v0, a0, j, dt, t_zero)) {
      return PropagationResult{ s, false };
    }

    const float s_step = v0 * dt + 0.5f * a0 * dt * dt + (1.0f / 6.0f) * j * dt * dt * dt;
    float v1 = v0 + a0 * dt + 0.5f * j * dt * dt;
    float a1 = a0 + j * dt;

    a1 = clampf(a1, -a_max, a_max);
    if (v1 < 0.0f) v1 = 0.0f;

    dist += s_step;
    s.v = v1;
    s.a = a1;
    ++steps;
  }
  return PropagationResult{ s, dist >= length };
}

void clear_bounds(float *a_min, float *a_max) {
  for (size_t k = 0; k < CJP_VELOCITY_BINS; ++k) {
    a_min[k] = kInf;
    a_max[k] = -kInf;
  }
}

bool has_bin(const float a_min, const float a_max) {
  return a_min <= a_max;
}

struct Planner {
  Block buf[CJP_BLOCK_BUFFER_SIZE];
  size_t head = 0;
  size_t tail = 0;
  size_t count = 0;

  float v_grid[CJP_VELOCITY_BINS];
  float v_cap[CJP_BLOCK_BUFFER_SIZE + 1];

  // Forward reachable acceleration bounds per junction/bin.
  float f_min[CJP_BLOCK_BUFFER_SIZE + 1][CJP_VELOCITY_BINS];
  float f_max[CJP_BLOCK_BUFFER_SIZE + 1][CJP_VELOCITY_BINS];
  // Backward (preimage) acceleration bounds per junction/bin.
  float b_min[CJP_BLOCK_BUFFER_SIZE + 1][CJP_VELOCITY_BINS];
  float b_max[CJP_BLOCK_BUFFER_SIZE + 1][CJP_VELOCITY_BINS];
  // Intersected feasible acceleration bounds per junction/bin.
  float s_min[CJP_BLOCK_BUFFER_SIZE + 1][CJP_VELOCITY_BINS];
  float s_max[CJP_BLOCK_BUFFER_SIZE + 1][CJP_VELOCITY_BINS];

  float selected_v[CJP_BLOCK_BUFFER_SIZE + 1];
  float selected_a[CJP_BLOCK_BUFFER_SIZE + 1];

  void reset() {
    head = tail = count = 0;
    memset(buf, 0, sizeof(buf));
  }

  bool push(const Block &b) {
    if (count >= (CJP_BLOCK_BUFFER_SIZE)) return false;
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

  Block* at(const size_t index) {
    if (index >= count) return nullptr;
    const size_t pos = (tail + index) % CJP_BLOCK_BUFFER_SIZE;
    return &buf[pos];
  }

  const Block* at(const size_t index) const {
    if (index >= count) return nullptr;
    const size_t pos = (tail + index) % CJP_BLOCK_BUFFER_SIZE;
    return &buf[pos];
  }

  void build_velocity_grid() {
    float vmax = 0.0f;
    for (size_t i = 0; i < count; ++i) {
      const Block *b = at(i);
      if (!b) continue;
      const float cap = fminf(b->nominal, b->max_entry_speed);
      if (cap > vmax) vmax = cap;
    }
    const float dv = (CJP_VELOCITY_BINS > 1) ? (vmax / static_cast<float>(CJP_VELOCITY_BINS - 1)) : 0.0f;
    for (size_t k = 0; k < CJP_VELOCITY_BINS; ++k)
      v_grid[k] = dv * static_cast<float>(k);
  }

  void build_caps() {
    if (!count) return;
    for (size_t i = 0; i < count; ++i) {
      const Block *b = at(i);
      if (!b) continue;
      v_cap[i + 1] = fminf(b->nominal, b->max_entry_speed);
    }
    v_cap[0] = fminf(at(0)->nominal, at(0)->max_entry_speed);
  }

  void reach_forward(const float *in_min, const float *in_max, const float length,
                     const float a_max, const float j_max, const float v_cap_next,
                     float *out_min, float *out_max) {
    clear_bounds(out_min, out_max);

    if (CJP_VELOCITY_BINS == 0) return;
    const float dv = (CJP_VELOCITY_BINS > 1) ? (v_grid[1] - v_grid[0]) : 1.0f;

    const float split_fracs[] = { 0.25f, 0.5f, 0.75f };
    const float split_fracs_count = sizeof(split_fracs) / sizeof(split_fracs[0]);

    auto deposit_state = [&](State s) {
      if (s.v > v_cap_next) return;
      s.a = clampf(s.a, -a_max, a_max);

      const int idx = static_cast<int>(floorf((s.v / dv) + 0.5f));
      if (idx < 0 || idx >= static_cast<int>(CJP_VELOCITY_BINS)) return;

      if (s.a < out_min[idx]) out_min[idx] = s.a;
      if (s.a > out_max[idx]) out_max[idx] = s.a;
    };

    for (size_t k = 0; k < CJP_VELOCITY_BINS; ++k) {
      if (!has_bin(in_min[k], in_max[k])) continue;

      const float v0 = v_grid[k];
      const float a_vals[2] = { in_min[k], in_max[k] };
      for (int ai = 0; ai < 2; ++ai) {
        const float a0 = a_vals[ai];

        for (int jsign = -1; jsign <= 1; jsign += 2) {
          State s{ v0, a0 };
          const PropagationResult result = propagate_extremal(s, length, a_max, j_max, jsign);
          if (!result.completed) continue;
          deposit_state(result.state);
        }

        for (int jsign = -1; jsign <= 1; jsign += 2) {
          for (size_t fi = 0; fi < static_cast<size_t>(split_fracs_count); ++fi) {
            const float frac = split_fracs[fi];
            const float len1 = length * frac;
            const float len2 = length - len1;

            State s{ v0, a0 };
            const PropagationResult first = propagate_extremal(s, len1, a_max, j_max, jsign);
            if (!first.completed) continue;
            const PropagationResult second = propagate_extremal(first.state, len2, a_max, j_max, -jsign);
            if (!second.completed) continue;

            deposit_state(second.state);
          }
        }
      }
    }
  }

  void reach_backward(const float *in_min, const float *in_max, const float length,
                      const float a_max, const float j_max, const float v_cap_start,
                      float *out_min, float *out_max) {
    clear_bounds(out_min, out_max);

    if (CJP_VELOCITY_BINS == 0) return;
    const float dv = (CJP_VELOCITY_BINS > 1) ? (v_grid[1] - v_grid[0]) : 1.0f;

    const float split_fracs[] = { 0.25f, 0.5f, 0.75f };
    const float split_fracs_count = sizeof(split_fracs) / sizeof(split_fracs[0]);

    auto deposit_start = [&](float v0, float a0) {
      if (v0 > v_cap_start) return;
      const int idx = static_cast<int>(floorf((v0 / dv) + 0.5f));
      if (idx < 0 || idx >= static_cast<int>(CJP_VELOCITY_BINS)) return;

      if (a0 < out_min[idx]) out_min[idx] = a0;
      if (a0 > out_max[idx]) out_max[idx] = a0;
    };

    auto endpoint_allowed = [&](const State &s) {
      const float v = s.v;
      if (v < 0.0f) return false;
      const int idx = static_cast<int>(floorf((v / dv) + 0.5f));
      if (idx < 0 || idx >= static_cast<int>(CJP_VELOCITY_BINS)) return false;
      if (!has_bin(in_min[idx], in_max[idx])) return false;
      return s.a >= in_min[idx] && s.a <= in_max[idx];
    };

    for (size_t k = 0; k < CJP_VELOCITY_BINS; ++k) {
      const float v0 = v_grid[k];
      if (v0 > v_cap_start) continue;

      const int idx0 = static_cast<int>(floorf((v0 / dv) + 0.5f));
      if (idx0 < 0 || idx0 >= static_cast<int>(CJP_VELOCITY_BINS)) continue;
      if (!has_bin(in_min[idx0], in_max[idx0])) continue;

      const float a_vals[2] = { in_min[idx0], in_max[idx0] };
      for (int ai = 0; ai < 2; ++ai) {
        const float a0 = a_vals[ai];

        for (int jsign = -1; jsign <= 1; jsign += 2) {
          State s{ v0, a0 };
          const PropagationResult result = propagate_extremal(s, length, a_max, j_max, jsign);
          if (!result.completed) continue;
          if (endpoint_allowed(result.state)) deposit_start(v0, a0);
        }

        for (int jsign = -1; jsign <= 1; jsign += 2) {
          for (size_t fi = 0; fi < static_cast<size_t>(split_fracs_count); ++fi) {
            const float frac = split_fracs[fi];
            const float len1 = length * frac;
            const float len2 = length - len1;

            State s{ v0, a0 };
            const PropagationResult first = propagate_extremal(s, len1, a_max, j_max, jsign);
            if (!first.completed) continue;
            const PropagationResult second = propagate_extremal(first.state, len2, a_max, j_max, -jsign);
            if (!second.completed) continue;

            if (endpoint_allowed(second.state)) deposit_start(v0, a0);
          }
        }
      }
    }
  }

  bool recalculate() {
    if (count == 0) return true;

    build_velocity_grid();
    build_caps();

    const size_t junctions = count + 1;

    for (size_t i = 0; i < junctions; ++i) {
      clear_bounds(f_min[i], f_max[i]);
      clear_bounds(b_min[i], b_max[i]);
      clear_bounds(s_min[i], s_max[i]);
    }

    // Initialize forward at junction 0.
    const Block *first = at(0);
    for (size_t k = 0; k < CJP_VELOCITY_BINS; ++k) {
      if (v_grid[k] <= v_cap[0]) {
        f_min[0][k] = -first->a_max;
        f_max[0][k] = first->a_max;
      }
    }

    // Forward pass.
    for (size_t i = 0; i < count; ++i) {
      const Block *b = at(i);
      reach_forward(f_min[i], f_max[i], b->millimeters, b->a_max, b->j_max,
                    v_cap[i + 1], f_min[i + 1], f_max[i + 1]);
    }

    // Initialize backward at final junction.
    const Block *last = at(count - 1);
    for (size_t k = 0; k < CJP_VELOCITY_BINS; ++k) {
      if (v_grid[k] <= v_cap[count]) {
        b_min[count][k] = -last->a_max;
        b_max[count][k] = last->a_max;
      }
    }

    // Backward pass.
    for (size_t rev = 0; rev < count; ++rev) {
      const size_t i = count - 1 - rev;
      const Block *b = at(i);
      reach_backward(b_min[i + 1], b_max[i + 1], b->millimeters, b->a_max, b->j_max,
                     v_cap[i], b_min[i], b_max[i]);
    }

    bool feasible = true;
    for (size_t i = 0; i < junctions; ++i) {
      bool any = false;
      for (size_t k = 0; k < CJP_VELOCITY_BINS; ++k) {
        s_min[i][k] = fmaxf(f_min[i][k], b_min[i][k]);
        s_max[i][k] = fminf(f_max[i][k], b_max[i][k]);
        if (has_bin(s_min[i][k], s_max[i][k])) any = true;
      }
      if (!any) feasible = false;
    }

    if (!feasible) return false;

    // Greedy selection at each junction.
    for (size_t i = 0; i < junctions; ++i) {
      int best = -1;
      for (int k = static_cast<int>(CJP_VELOCITY_BINS) - 1; k >= 0; --k) {
        if (v_grid[k] > v_cap[i]) continue;
        if (has_bin(s_min[i][k], s_max[i][k])) { best = k; break; }
      }
      if (best < 0) {
        selected_v[i] = 0.0f;
        selected_a[i] = 0.0f;
        continue;
      }

      const float a_lo = s_min[i][best];
      const float a_hi = s_max[i][best];
      float a_pick = 0.0f;
      if (a_pick < a_lo) a_pick = a_lo;
      if (a_pick > a_hi) a_pick = a_hi;

      selected_v[i] = v_grid[best];
      selected_a[i] = a_pick;
    }

    // Store entry/exit states per block.
    for (size_t i = 0; i < count; ++i) {
      Block *b = at(i);
      b->entry_v = selected_v[i];
      b->entry_a = selected_a[i];
      b->exit_v = selected_v[i + 1];
      b->exit_a = selected_a[i + 1];
    }

    return true;
  }
};

Planner planner;

} // namespace

extern "C" {

void cjp_reset(void) {
  planner.reset();
}

int cjp_push_block(const float millimeters, const float max_entry_speed, const float nominal,
                   const float a_max, const float j_max) {
  if (!(millimeters >= 0.0f) || !(max_entry_speed >= 0.0f) || !(nominal >= 0.0f) || !(a_max > 0.0f) || !(j_max > 0.0f))
    return 0;

  Block b{};
  b.millimeters = millimeters;
  b.max_entry_speed = max_entry_speed;
  b.nominal = nominal;
  b.a_max = a_max;
  b.j_max = j_max;
  b.entry_v = 0.0f;
  b.entry_a = 0.0f;
  b.exit_v = 0.0f;
  b.exit_a = 0.0f;

  if (!planner.push(b)) return 0;
  return 1;
}

int cjp_recalculate(void) {
  return planner.recalculate() ? 1 : 0;
}

size_t cjp_size(void) {
  return planner.count;
}

int cjp_get_block(const size_t index, CJP_BlockOut *out) {
  if (!out) return 0;
  const Block * const b = planner.at(index);
  if (!b) return 0;

  CJP_BlockOut tmp{};
  tmp.millimeters = b->millimeters;
  tmp.max_entry_speed = b->max_entry_speed;
  tmp.nominal = b->nominal;
  tmp.a_max = b->a_max;
  tmp.j_max = b->j_max;
  tmp.entry_v = b->entry_v;
  tmp.entry_a = b->entry_a;
  tmp.exit_v = b->exit_v;
  tmp.exit_a = b->exit_a;

  *out = tmp;
  return 1;
}

int cjp_pop_front(void) {
  return planner.pop_front() ? 1 : 0;
}

} // extern "C"
