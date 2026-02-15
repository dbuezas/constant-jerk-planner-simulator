#pragma once

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Constant-Jerk Planner — Envelope + Selection Architecture
//
// 1. State Model
//
// 1D path parameterization with jerk limits.
//
// State at each junction:
//   x = (v, a)
//
// Dynamics:
//   v_dot = a,
//   a_dot = j
//
// Constraints:
//   0 <= v <= nominal (velocity never goes negative within a block)
//   |a| <= a_max
//   |j| <= j_max
//   Per-junction velocity cap: v <= max_entry_speed[i]
//
// 2. Feasibility Representation (Per Junction)
//
// Instead of storing arbitrary 2D sets, represent feasible states as:
//   a in [a_min(v_k), a_max(v_k)]
// over a fixed velocity grid (v_k).
//
// So each junction stores two arrays:
//   a_min[i][k]
//   a_max[i][k]
//
// This represents the 2D feasible set compactly.
//
// 3. Forward Pass (Reachable Set Propagation)
//
// For each block of length L_i:
//   For every velocity bin v_k:
//     Take boundary accelerations:
//       a = a_min(v_k)
//       a = a_max(v_k)
//     Propagate each boundary state through block length L_i
//     using jerk-limited bang-bang extremals (±J, accel saturation).
//     Deposit resulting endpoint (v', a') into output grid.
//     Enforce velocity cap at next junction and accel limits.
//
// Result:
//   F_{i+1} = ReachForward(F_i, L_i)
//
// 4. Backward Pass (Preimage Propagation)
//
// Same mechanism, but propagating from end to start.
// Because system is symmetric (±J, ±A), same propagation logic can be reused.
//
// Result:
//   B_i = ReachBackward(B_{i+1}, L_i)
//
// 5. Intersection
//
// Final feasible set at junction i:
//   S_i = F_i ∩ B_i
//
// Intersection is pointwise on velocity bins:
//   a_min[i][k] = max(F_min, B_min)
//   a_max[i][k] = min(F_max, B_max)
//
// If no bin has a_min <= a_max, path is infeasible.
//
// 6. Velocity Selection (Greedy Version)
//
// Greedy time-biased choice at each junction:
//   Pick highest velocity bin with nonempty interval.
//   Choose acceleration:
//     Prefer 0 if inside interval.
//     Else clamp to nearest boundary.
//
// This is not globally optimal, but near time-optimal in many cases.
// For guaranteed optimality:
//   Run dynamic programming on velocity bins
//   Minimize total time using block transition costs

#ifndef CJP_BLOCK_BUFFER_SIZE
#define CJP_BLOCK_BUFFER_SIZE 32
#endif

#ifndef CJP_VELOCITY_BINS
#define CJP_VELOCITY_BINS 64
#endif

typedef struct CJP_BlockOut {
  float millimeters;
  float max_entry_speed;
  float nominal;
  float a_max;
  float j_max;

  float entry_v;
  float entry_a;
  float exit_v;
  float exit_a;
} CJP_BlockOut;

void cjp_reset(void);

// Returns 1 on success, 0 if buffer is full or invalid args.
int cjp_push_block(float millimeters, float max_entry_speed, float nominal, float a_max, float j_max);

// Recalculate all entry/exit states. Returns 1 if feasible, 0 if infeasible.
int cjp_recalculate(void);

// Number of queued blocks.
size_t cjp_size(void);

// Returns 1 on success, 0 if index out of range.
int cjp_get_block(size_t index, CJP_BlockOut *out);

// Drop the oldest block. Returns 1 if a block was dropped.
int cjp_pop_front(void);

#ifdef __cplusplus
}
#endif
