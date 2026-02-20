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

struct JunctionConstraint {
  float cumulative_dist;  // distance from start of merged group
  float max_speed;
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
  JunctionConstraint junctions[CJP_BLOCK_BUFFER_SIZE];
  size_t junction_count;
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
  }
  return lo;
}

#define CJP_MERGE_AMAX_RATIO 1.1f  // max/min a_max ratio allowed for merging

// Check if a candidate block can merge into a group with the given constraints.
// nominal and j_max must match exactly; a_max must be within ratio threshold
// of the group's current min/max a_max range.
static bool can_merge(const Block& candidate, float group_nominal, float group_j_max,
                      float group_a_max_min, float group_a_max_max) {
  if (candidate.nominal != group_nominal || candidate.j_max != group_j_max)
    return false;
  float new_min = fminf(group_a_max_min, candidate.a_max);
  float new_max = fmaxf(group_a_max_max, candidate.a_max);
  return new_max <= new_min * CJP_MERGE_AMAX_RATIO;
}

// Build merged groups from original blocks
static void build_merged_groups(Planner& p) {
  p.merged_count = 0;
  if (p.count == 0) return;

  size_t i = 0;
  while (i < p.count) {
    MergedBlock& m = p.merged[p.merged_count];
    const Block* first = p.at(i);
    m.millimeters = first->millimeters;
    m.nominal = first->nominal;
    m.a_max = first->a_max;
    m.j_max = first->j_max;
    m.max_entry_speed = first->max_entry_speed;
    m.orig_start = i;
    m.orig_count = 1;
    m.junction_count = 0;
    m.entry_v = 0;
    m.exit_v = 0;

    float a_max_min = first->a_max;
    float a_max_max = first->a_max;

    float cum_dist = first->millimeters;
    size_t j = i + 1;
    while (j < p.count && can_merge(*p.at(j), m.nominal, m.j_max, a_max_min, a_max_max)) {
      const Block* blk = p.at(j);

      a_max_min = fminf(a_max_min, blk->a_max);
      a_max_max = fmaxf(a_max_max, blk->a_max);

      // Record interior junction constraint
      m.junctions[m.junction_count].cumulative_dist = cum_dist;
      m.junctions[m.junction_count].max_speed = blk->max_entry_speed;
      m.junction_count++;

      cum_dist += blk->millimeters;
      m.orig_count++;
      j++;
    }
    // Use the most conservative a_max in the group
    m.a_max = a_max_min;
    m.millimeters = cum_dist;
    p.merged_count++;
    i = j;
  }
}

// Run reverse/forward pass on merged blocks
static void plan_merged_passes(Planner& p) {
  const size_t n = p.merged_count;
  if (n == 0) return;
  const size_t last = n - 1;

  // Reverse pass
  for (size_t rev = 0; rev < n; rev++) {
    size_t i = last - rev;
    MergedBlock& m = p.merged[i];

    float required_exit = (i == last) ? 0.0f : p.merged[i + 1].entry_v;
    float cap = fminf(m.nominal, m.max_entry_speed);
    float max_from_exit = max_reachable_speed(required_exit, m.millimeters,
                                              m.nominal, m.a_max, m.j_max);
    m.entry_v = fminf(cap, max_from_exit);
  }

  // Forward pass
  p.merged[0].entry_v = 0.0f;
  for (size_t i = 1; i < n; i++) {
    MergedBlock& prev = p.merged[i - 1];
    MergedBlock& curr = p.merged[i];

    float max_from_prev = max_reachable_speed(prev.entry_v, prev.millimeters,
                                              prev.nominal, prev.a_max, prev.j_max);
    float cap = fminf(curr.nominal, curr.max_entry_speed);
    float forward_limit = fminf(cap, max_from_prev);
    if (forward_limit < curr.entry_v)
      curr.entry_v = forward_limit;
  }

  // Propagate exit speeds
  for (size_t i = 0; i < n; i++) {
    p.merged[i].exit_v = (i == last) ? 0.0f : p.merged[i + 1].entry_v;
  }
}

// Verify interior junctions and split if violated.
// Returns true if a split occurred.
static bool verify_and_split(Planner& p) {
  for (size_t mi = 0; mi < p.merged_count; mi++) {
    MergedBlock& m = p.merged[mi];
    if (m.junction_count == 0) continue;

    float D = m.millimeters;

    // Find the most-violated junction
    size_t worst_ji = 0;
    float worst_excess = 0;
    bool found_violation = false;

    for (size_t ji = 0; ji < m.junction_count; ji++) {
      float d = m.junctions[ji].cumulative_dist;
      float v_from_entry = max_reachable_speed(m.entry_v, d, m.nominal, m.a_max, m.j_max);
      float v_from_exit = max_reachable_speed(m.exit_v, D - d, m.nominal, m.a_max, m.j_max);
      float v_upper = fminf(v_from_entry, v_from_exit);
      float excess = v_upper - m.junctions[ji].max_speed;

      if (excess > worst_excess) {
        worst_excess = excess;
        worst_ji = ji;
        found_violation = true;
      }
    }

    if (!found_violation) continue;

    // Split at worst_ji: this junction becomes a real a=0 boundary
    // Original blocks: merged[mi] covers orig_start..orig_start+orig_count-1
    // Junction worst_ji is between original block (orig_start + worst_ji) and (orig_start + worst_ji + 1)
    // So first half has (worst_ji + 1) original blocks, second half has the rest.

    size_t first_orig_count = worst_ji + 1;
    size_t second_orig_count = m.orig_count - first_orig_count;

    // Make room: shift merged blocks after mi to the right
    if (p.merged_count >= CJP_BLOCK_BUFFER_SIZE) return false;  // can't split further
    for (size_t k = p.merged_count; k > mi + 1; k--) {
      p.merged[k] = p.merged[k - 1];
    }
    p.merged_count++;

    // Build first half (reuse slot mi)
    MergedBlock& first = p.merged[mi];
    MergedBlock& second = p.merged[mi + 1];

    // Save original values before modifying
    size_t orig_start = m.orig_start;
    float nominal = m.nominal, a_max = m.a_max, j_max = m.j_max;

    // Compute first half distance
    float first_dist = m.junctions[worst_ji].cumulative_dist;

    // Build second half
    second.nominal = nominal;
    second.a_max = a_max;
    second.j_max = j_max;
    second.millimeters = m.millimeters - first_dist;
    second.orig_start = orig_start + first_orig_count;
    second.orig_count = second_orig_count;
    second.max_entry_speed = m.junctions[worst_ji].max_speed;
    second.entry_v = 0;
    second.exit_v = 0;

    // Copy junctions after the split point to the second half, adjusting distances
    second.junction_count = 0;
    for (size_t ji = worst_ji + 1; ji < m.junction_count; ji++) {
      second.junctions[second.junction_count].cumulative_dist =
          m.junctions[ji].cumulative_dist - first_dist;
      second.junctions[second.junction_count].max_speed = m.junctions[ji].max_speed;
      second.junction_count++;
    }

    // Shrink first half
    first.millimeters = first_dist;
    first.orig_start = orig_start;
    first.orig_count = first_orig_count;
    first.max_entry_speed = p.at(orig_start)->max_entry_speed;
    first.junction_count = worst_ji;  // junctions before the split point stay

    return true;  // signal that we split, need to re-plan
  }
  return false;
}

// Propagate merged entry/exit velocities back to original blocks
static void propagate_to_original(Planner& p) {
  for (size_t mi = 0; mi < p.merged_count; mi++) {
    const MergedBlock& m = p.merged[mi];
    // First original block in group gets group's entry_v
    p.at(m.orig_start)->entry_v = m.entry_v;
    // Last original block in group gets group's exit_v
    p.at(m.orig_start + m.orig_count - 1)->exit_v = m.exit_v;

    // Interior original blocks: set entry_v = exit_v of previous = 0 (marker for "merged away")
    for (size_t k = 1; k < m.orig_count; k++) {
      size_t idx = m.orig_start + k;
      // These junctions don't physically exist in the merged trajectory
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

  build_merged_groups(planner);
  plan_merged_passes(planner);

  // Verify and split until stable (max iterations = number of original blocks)
  for (size_t iter = 0; iter < planner.count; iter++) {
    if (!verify_and_split(planner)) break;
    plan_merged_passes(planner);
  }

  propagate_to_original(planner);
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
