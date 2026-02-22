#include "constant-jerk-planner.h"

#include <math.h>
#include <string.h>

#include "trajectory_constant_jerk.h"

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

struct MergedBlock {
  float millimeters;
  float nominal;
  float a_max;
  float j_max;
  float max_entry_speed;
  float entry_v;
  float exit_v;
  size_t orig_start;
  size_t orig_count;
};

struct ExecState {
  size_t merged_idx;
  size_t orig_idx;
  float time_in_merged;
  float cumulative_time;
  float cumulative_pos;
  float merged_start_pos;      // cumulative position at start of current merged block
  float orig_block_end_dist;   // distance within merged block where current orig block ends
  ConstantJerkTrajectoryGenerator traj;
  bool done;
};

#define CJP_BLOCK_BUFFER_SIZE 32

struct Planner {
  Block buf[CJP_BLOCK_BUFFER_SIZE];
  size_t head = 0;
  size_t tail = 0;
  size_t count = 0;

  MergedBlock merged[CJP_BLOCK_BUFFER_SIZE];
  size_t merged_count = 0;

  ExecState exec = {};

  void reset() {
    head = tail = count = 0;
    merged_count = 0;
    memset(buf, 0, sizeof(buf));
    memset(merged, 0, sizeof(merged));
    memset(&exec, 0, sizeof(exec));
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

static float max_reachable_speed(float v_from, float mm,
                                 float nominal, float a_max, float j_max) {
  if (mm <= 0.0f) return v_from;

  float v_trap = sqrtf(v_from * v_from + 2.0f * a_max * mm);
  float hi = fminf(nominal, v_trap);
  float lo = v_from;

  if (hi <= lo) return lo;

  float pa, pb, pc;
  float s = planRamp(v_from, hi, j_max, a_max, false, pa, pb, pc);
  if (s <= mm) return hi;

  for (int i = 0; i < 32; i++) {
    float mid = 0.5f * (lo + hi);
    s = planRamp(v_from, mid, j_max, a_max, false, pa, pb, pc);
    if (s <= mm)
      lo = mid;
    else
      hi = mid;
    if (hi - lo < 0.01f) break;
  }
  return lo;
}

// Compute the peak velocity of an S-curve trajectory (without building phases).
static float peak_speed(float v_entry, float v_exit, float a_max_val,
                        float j_max_val, float dist, float v_nominal) {
  float v_small = fminf(v_entry, v_exit);
  float v_large = fmaxf(v_entry, v_exit);
  float v_peak = fmaxf(v_large, v_nominal);
  float s_ramps = totalRampDist(v_peak, v_small, v_large, j_max_val, a_max_val);

  if (s_ramps > dist) {
    float v_hi = v_peak, v_lo = v_large;
    if (totalRampDist(v_lo, v_small, v_large, j_max_val, a_max_val) > dist)
      return v_lo;
    for (int i = 0; i < 16; i++) {
      float mid = 0.5f * (v_lo + v_hi);
      float s = totalRampDist(mid, v_small, v_large, j_max_val, a_max_val);
      if (s > dist) v_hi = mid; else v_lo = mid;
      if (v_hi - v_lo < 0.01f) break;
    }
    v_peak = v_lo;
  }
  return v_peak;
}

#define CJP_MERGE_AMAX_RATIO 1.1f

static float sum_dist(const float* mm_arr, size_t count) {
  float total = 0;
  for (size_t i = 0; i < count; i++) total += mm_arr[i];
  return total;
}

static float min_val(const float* arr, size_t from, size_t to) {
  float v = arr[from];
  for (size_t i = from + 1; i < to; i++) v = fminf(v, arr[i]);
  return v;
}

// Run the planner: reverse/forward pass + superblock merge
static void recalculate(Planner& p) {
  p.merged_count = 0;
  const size_t n = p.count;
  if (n == 0) return;

  // Gather block parameters into local arrays
  float mm[CJP_BLOCK_BUFFER_SIZE];
  float nominal[CJP_BLOCK_BUFFER_SIZE];
  float accel[CJP_BLOCK_BUFFER_SIZE];
  float max_junction_v[CJP_BLOCK_BUFFER_SIZE];

  for (size_t i = 0; i < n; i++) {
    const Block* b = p.at(i);
    mm[i] = b->millimeters;
    nominal[i] = b->nominal;
    accel[i] = b->a_max;
    max_junction_v[i] = b->max_entry_speed;
  }

  // Use the first block's j_max (all blocks assumed same j_max for merging)
  float j_max = p.at(0)->j_max;

  // --- Jerk-aware reverse/forward pass ---
  float entry_v[CJP_BLOCK_BUFFER_SIZE];
  float exit_v[CJP_BLOCK_BUFFER_SIZE];

  if (n == 1) {
    // Single block: must decelerate to 0
    exit_v[0] = 0.0f;
    entry_v[0] = fminf(fminf(max_junction_v[0], nominal[0]),
                        max_reachable_speed(0.0f, mm[0], nominal[0], accel[0], j_max));
  }
  else {
    // Reverse pass
    exit_v[n - 1] = 0.0f;
    for (int i = (int)n - 1; i >= 0; i--) {
      float v_reachable = max_reachable_speed(exit_v[i], mm[i], nominal[i], accel[i], j_max);
      entry_v[i] = fminf(fminf(v_reachable, max_junction_v[i]), nominal[i]);
      if (i > 0) exit_v[i - 1] = entry_v[i];
    }

    // Forward pass: first block starts from rest (no prior motion in web version)
    entry_v[0] = 0.0f;
    for (size_t i = 0; i < n; i++) {
      float v_reachable = max_reachable_speed(entry_v[i], mm[i], nominal[i], accel[i], j_max);
      if (i < n - 1) {
        exit_v[i] = fminf(fminf(v_reachable, entry_v[i + 1]), nominal[i]);
        entry_v[i + 1] = fminf(entry_v[i + 1], exit_v[i]);
      }
      else {
        exit_v[i] = fminf(fminf(v_reachable, exit_v[i]), nominal[i]);
      }
    }
  }

  // --- Build merged groups with left/right superblock algorithm ---
  // Same algorithm as Marlin: find left-compatible group + right-compatible
  // group. Right group acts as superblock for better left exit speed.
  // v_peak check against min interior junction, binary split on failure.
  size_t pos = 0;
  while (pos < n) {
    size_t remaining = n - pos;
    size_t merge_count = 1;

    if (remaining >= 2) {
      // Find left-compatible group, tracking cumulative mm, min accel, min junction
      float group_a_min = accel[pos], group_a_max_val = accel[pos];
      float cum_mm[CJP_BLOCK_BUFFER_SIZE];
      float cum_min_a[CJP_BLOCK_BUFFER_SIZE];
      float cum_min_jv[CJP_BLOCK_BUFFER_SIZE];
      cum_mm[0] = mm[pos];
      cum_min_a[0] = accel[pos];
      cum_min_jv[0] = max_junction_v[pos]; // unused but initialized

      size_t left_end = 1;
      for (size_t i = 1; i < remaining; i++) {
        size_t bi = pos + i;
        if (nominal[bi] != nominal[pos]) break;
        float new_min = fminf(group_a_min, accel[bi]);
        float new_max = fmaxf(group_a_max_val, accel[bi]);
        if (new_max > new_min * CJP_MERGE_AMAX_RATIO) break;
        group_a_min = new_min;
        group_a_max_val = new_max;
        cum_mm[i] = cum_mm[i - 1] + mm[bi];
        cum_min_a[i] = fminf(cum_min_a[i - 1], accel[bi]);
        cum_min_jv[i] = (i == 1) ? max_junction_v[pos + 1]
                                  : fminf(cum_min_jv[i - 1], max_junction_v[bi]);
        left_end = i + 1;
      }

      // No cap needed in batch mode (all blocks known, reverse/forward pass
      // already ran on everything). Marlin caps to block_count/2 because it's
      // streaming and needs blocks beyond the merge for look-ahead.

      if (left_end >= 2) {
        // Find right-compatible group starting at pos + left_end
        size_t right_end = left_end;
        if (pos + left_end < n) {
          size_t ri = pos + left_end;
          float r_a_min = accel[ri], r_a_max = accel[ri];
          right_end = left_end + 1;
          for (size_t i = left_end + 1; i < remaining; i++) {
            size_t bi = pos + i;
            if (nominal[bi] != nominal[ri]) break;
            float new_min = fminf(r_a_min, accel[bi]);
            float new_max = fmaxf(r_a_max, accel[bi]);
            if (new_max > new_min * CJP_MERGE_AMAX_RATIO) break;
            r_a_min = new_min;
            r_a_max = new_max;
            right_end = i + 1;
          }
        }

        // Iteratively refine: split groups until junction constraints are met
        while (true) {
          if (left_end < 2) break;

          float left_mm = cum_mm[left_end - 1];
          float left_a = cum_min_a[left_end - 1];
          float left_nominal = nominal[pos];

          // Compute the exit speed for the left group.
          // If the right side is a superblock (>1 block), it can accept higher
          // entry speed since it has more distance to decelerate.
          float right_mm_val = 0, right_a = 0, right_nominal = 0;
          bool has_right_super = (right_end > left_end + 1);
          if (has_right_super) {
            right_mm_val = sum_dist(mm + pos + left_end, right_end - left_end);
            right_a = min_val(accel, pos + left_end, pos + right_end);
            right_nominal = nominal[pos + left_end];
          }

          float after_left_entry;
          if (has_right_super) {
            float tail_entry = (pos + right_end < n) ? entry_v[pos + right_end] : 0.0f;
            float v_reach = max_reachable_speed(tail_entry, right_mm_val, right_nominal, right_a, j_max);
            after_left_entry = fminf(fminf(v_reach, max_junction_v[pos + left_end]), right_nominal);
          }
          else if (pos + left_end < n) {
            after_left_entry = entry_v[pos + left_end];
          }
          else {
            after_left_entry = 0.0f;
          }

          // Left superblock: reverse then forward
          // Cap entry by pass-computed entry_v[pos] (web-specific: respects start-from-rest)
          float left_exit = after_left_entry;
          float v_reach = max_reachable_speed(left_exit, left_mm, left_nominal, left_a, j_max);
          float left_entry = fminf(fminf(fminf(v_reach, max_junction_v[pos]), left_nominal), entry_v[pos]);

          // Forward: cap exit by what's reachable from entry
          float v_fwd = max_reachable_speed(left_entry, left_mm, left_nominal, left_a, j_max);
          left_exit = fminf(fminf(v_fwd, left_exit), left_nominal);

          // Check v_peak against min interior junction limit
          float v_peak = peak_speed(left_entry, left_exit, left_a, j_max, left_mm, left_nominal);
          float min_jv = cum_min_jv[left_end - 1];

          if (v_peak <= min_jv) {
            // Left group valid. Check right group if it's a superblock.
            if (has_right_super) {
              float right_entry = left_exit;
              float right_exit_v = (pos + right_end < n) ? entry_v[pos + right_end] : 0.0f;

              float rv_fwd = max_reachable_speed(right_entry, right_mm_val, right_nominal, right_a, j_max);
              right_exit_v = fminf(fminf(rv_fwd, right_exit_v), right_nominal);

              float right_vpeak = peak_speed(right_entry, right_exit_v, right_a, j_max, right_mm_val, right_nominal);
              float right_min_jv = min_val(max_junction_v, pos + left_end + 1, pos + right_end);

              if (right_vpeak > right_min_jv) {
                size_t right_len = right_end - left_end;
                right_end = left_end + right_len / 2;
                if (right_end <= left_end) right_end = left_end + 1;
                continue;
              }
            }
            // Both groups valid
            merge_count = left_end;
            entry_v[pos] = left_entry;
            exit_v[pos + left_end - 1] = left_exit;
            break;
          }
          else {
            // Left too aggressive — split in half
            size_t new_left_end = left_end / 2;
            if (new_left_end < 2) new_left_end = 1;
            right_end = left_end;
            left_end = new_left_end;
          }
        }
      }
    }

    // Emit the merged block (merge_count >= 1)
    MergedBlock& m = p.merged[p.merged_count++];
    m.millimeters = sum_dist(mm + pos, merge_count);
    m.nominal = nominal[pos];
    m.a_max = min_val(accel, pos, pos + merge_count);
    m.j_max = j_max;
    m.max_entry_speed = max_junction_v[pos];
    m.entry_v = entry_v[pos];
    m.exit_v = exit_v[pos + merge_count - 1];
    m.orig_start = pos;
    m.orig_count = merge_count;

    pos += merge_count;
  }

  // Propagate merged entry/exit velocities back to original blocks
  for (size_t mi = 0; mi < p.merged_count; mi++) {
    const MergedBlock& m = p.merged[mi];
    p.at(m.orig_start)->entry_v = m.entry_v;
    p.at(m.orig_start + m.orig_count - 1)->exit_v = m.exit_v;

    // Interior original blocks: set entry_v = exit_v = 0 (marker for "merged away")
    for (size_t k = 1; k < m.orig_count; k++) {
      size_t idx = m.orig_start + k;
      p.at(idx)->entry_v = 0;
      p.at(idx - 1)->exit_v = 0;
    }
  }
}

// Plan trajectory for a merged block and set it on the exec state's traj generator
static void plan_merged_trajectory(Planner& p, size_t merged_idx) {
  const MergedBlock& m = p.merged[merged_idx];
  p.exec.traj.plan(m.entry_v, m.exit_v, m.a_max, m.j_max, m.millimeters, m.nominal);
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
  if (planner.count == 0) return 1;
  recalculate(planner);
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

size_t cjp_merged_size(void) {
  return planner.merged_count;
}

int cjp_get_merged_block(size_t index, CJP_BlockOut* out) {
  if (!out || index >= planner.merged_count) return 0;
  const MergedBlock& m = planner.merged[index];

  out->millimeters = m.millimeters;
  out->max_entry_speed = m.max_entry_speed;
  out->nominal = m.nominal;
  out->a_max = m.a_max;
  out->j_max = m.j_max;
  out->entry_v = m.entry_v;
  out->exit_v = m.exit_v;

  return 1;
}

void cjp_exec_reset(void) {
  ExecState& e = planner.exec;
  e.merged_idx = 0;
  e.orig_idx = 0;
  e.time_in_merged = 0;
  e.cumulative_time = 0;
  e.cumulative_pos = 0;
  e.merged_start_pos = 0;
  e.done = (planner.merged_count == 0);

  if (!e.done) {
    plan_merged_trajectory(planner, 0);
    // Set up first original block end distance within this merged block
    e.orig_block_end_dist = planner.at(planner.merged[0].orig_start)->millimeters;
  }
}

int cjp_exec_step(float dt) {
  ExecState& e = planner.exec;
  if (e.done) return 0;

  e.time_in_merged += dt;
  e.cumulative_time += dt;

  // Check if we've passed the end of current merged block
  float dur = e.traj.getTotalDuration();
  while (e.time_in_merged >= dur) {
    // Move to next merged block
    e.merged_start_pos += e.traj.getDistanceAtTime(dur);
    e.time_in_merged -= dur;
    e.merged_idx++;

    if (e.merged_idx >= planner.merged_count) {
      e.done = true;
      return 0;
    }

    plan_merged_trajectory(planner, e.merged_idx);
    dur = e.traj.getTotalDuration();

    // Reset original block tracking for new merged group
    const MergedBlock& m = planner.merged[e.merged_idx];
    e.orig_idx = m.orig_start;
    e.orig_block_end_dist = planner.at(e.orig_idx)->millimeters;
  }

  // Check if we've crossed into the next original block within this merged group
  float pos_in_merged = e.traj.getDistanceAtTime(e.time_in_merged);
  const MergedBlock& m = planner.merged[e.merged_idx];
  while (pos_in_merged >= e.orig_block_end_dist &&
         e.orig_idx < m.orig_start + m.orig_count - 1) {
    e.orig_idx++;
    e.orig_block_end_dist += planner.at(e.orig_idx)->millimeters;
  }

  e.cumulative_pos = e.merged_start_pos + pos_in_merged;
  return 1;
}

float cjp_exec_time(void) {
  return planner.exec.cumulative_time;
}

float cjp_exec_position(void) {
  return planner.exec.cumulative_pos;
}

float cjp_exec_velocity(void) {
  return planner.exec.traj.getVelocityAtTime(planner.exec.time_in_merged);
}

float cjp_exec_acceleration(void) {
  return planner.exec.traj.getAccelerationAtTime(planner.exec.time_in_merged);
}

float cjp_exec_jerk(void) {
  return planner.exec.traj.getJerkAtTime(planner.exec.time_in_merged);
}

size_t cjp_exec_original_block(void) {
  return planner.exec.orig_idx;
}

size_t cjp_exec_merged_block(void) {
  return planner.exec.merged_idx;
}

}  // extern "C"
