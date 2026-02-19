
#pragma once

#include <math.h>

/**
 * Constant-jerk trajectory generator.
 * Builds a 7-phase S-curve using bang-bang jerk and accel limits.
 * ROOT: Plan 7-phase S-curve
 * │
 * ├─ STEP 1: Determine feasible peak velocity v_peak
 * │   │
 * │   │  [Binary search or closed-form: find v_peak ∈ [v_lo, v_nom]
 * │   │   such that dist(v0,a0 → v_peak) + dist(v_peak → v1,a1) ≤ distance]
 * │   │
 * │   ├─ Can reach v_nom within distance?
 * │   │   ├─ YES → v_peak = v_nom, cruise phase exists (phase 4 duration > 0)
 * │   │   └─ NO  → v_peak < v_nom, cruise phase collapses (phase 4 = 0)
 * │   │               │
 * │   │               └─ Is even v_peak = max(v0,v1) feasible?
 * │   │                   ├─ YES → v_peak somewhere in (min,max) of {v0,v1}
 * │   │                   └─ NO  → v_peak = min(v0,v1), profile is monotone decel
 * │
 * ├─ STEP 2: Plan the ACCEL ramp (v0,a0) → (v_peak, 0)
 * │   │
 * │   │  Δv = v_peak - v0,  entry accel = a0
 * │   │
 * │   ├─ CASE A: Δv = 0 and a0 = 0
 * │   │   └─ Ramp is trivial (zero duration), skip to Step 3
 * │   │
 * │   ├─ CASE B: a0 ≥ 0  (entering with zero or positive accel)
 * │   │   │
 * │   │   ├─ Sub-case: Δv ≥ 0  (need to go faster)
 * │   │   │   │
 * │   │   │   ├─ Does accel saturate?
 * │   │   │   │   [i.e. can we reach a_max before needing to jerk back down]
 * │   │   │   │   ├─ YES → full S: [+jerk → cruise_accel → -jerk]  (phases 1,2,3)
 * │   │   │   │   └─ NO  → triangle: [+jerk → -jerk]  (phases 1,3 only, phase 2=0)
 * │   │   │   │
 * │   │   │   └─ Note: phase 1 may be shortened if a0 > 0
 * │   │   │       (we start partway up the jerk ramp)
 * │   │   │
 * │   │   └─ Sub-case: Δv < 0  (need to go slower, but a0 ≥ 0)
 * │   │       │  Must decelerate while unwinding positive entry accel first
 * │   │       │
 * │   │       ├─ Does -accel saturate?
 * │   │       │   ├─ YES → [-jerk → cruise_decel → +jerk]
 * │   │       │   └─ NO  → triangle: [-jerk → +jerk]
 * │   │       │
 * │   │       └─ Note: phase 1 is -jerk to bring a0 down to 0 then to -a_max
 * │
 * │   └─ CASE C: a0 < 0  (entering with negative accel, decelerating)
 * │       │
 * │       ├─ Sub-case: Δv ≥ 0  (need to accelerate, but we're currently decelerating)
 * │       │   │  Must first undo negative accel, then accelerate
 * │       │   │
 * │       │   ├─ Does +accel saturate?
 * │       │   │   ├─ YES → [+jerk → cruise_accel → -jerk]
 * │       │   │   └─ NO  → triangle: [+jerk → -jerk]
 * │       │   │
 * │       │   └─ Phase 1 is +jerk starting from a0 < 0
 * │       │
 * │       └─ Sub-case: Δv < 0  (need to go slower, already decelerating)
 * │           │
 * │           ├─ Does -accel saturate?
 * │           │   ├─ YES → [-jerk → cruise_decel → +jerk]
 * │           │   └─ NO  → triangle: [-jerk → +jerk]  (or trivial if a0 already optimal)
 * │           │
 * │           └─ Phase 1 shortened/skipped if a0 already at -a_max
 * │
 * ├─ STEP 3: Cruise phase (phase 4)
 * │   │
 * │   ├─ v_peak = v_nom AND distance remaining > 0 → phase 4 duration > 0
 * │   └─ otherwise → phase 4 duration = 0  (skip)
 * │
 * └─ STEP 4: Plan the DECEL ramp (v_peak, 0) → (v1, a1)
 *     │
 *     │  Mirror of Step 2 with: entry=(v_peak,0), exit=(v1,a1)
 *     │  Δv = v1 - v_peak  (always ≤ 0 since v_peak ≥ v1 by construction)
 *     │
 *     ├─ CASE A: Δv = 0 and a1 = 0 → trivial, skip
 *     │
 *     ├─ Sub-case: a1 = 0  (standard exit)
 *     │   ├─ Does -accel saturate?
 *     │   │   ├─ YES → [-jerk → cruise_decel → +jerk]  (phases 5,6,7)
 *     │   │   └─ NO  → triangle: [-jerk → +jerk]  (phases 5,7 only)
 *     │
 *     ├─ Sub-case: a1 > 0  (exit while accelerating — handing off to next block)
 *     │   │  Need to end at positive accel: decel ramp must leave a1 > 0 at end
 *     │   ├─ Does -accel saturate?
 *     │   │   ├─ YES → [-jerk → cruise_decel → +jerk]  phase 7 shortened
 *     │   │   └─ NO  → triangle, phase 7 shortened (don't jerk all the way back to 0)
 *     │
 *     └─ Sub-case: a1 < 0  (exit while still decelerating)
 *         │  Phase 7 (+jerk) is shortened or eliminated
 *         ├─ Does -accel saturate?
 *         │   ├─ YES → [-jerk → cruise_decel → +jerk_partial]  phase 7 < full
 *         │   └─ NO  → triangle with partial or zero phase 7
 *         │
 *         └─ Degenerate: a1 = -a_max → phase 7 = 0 entirely
 */
class ConstantJerkTrajectoryGenerator {
public:
  ConstantJerkTrajectoryGenerator() = default;

  void plan(float initial_speed_in, float initial_accel_in,
            float final_speed_in, float final_accel_in,
            float accel_max_in, float jerk_in,
            float distance_in, float v_nominal_in) {}

  float getDistanceAtTime(float t) const { return 0.0f; }
  float getTotalDuration() const { return 0.0f; }
  float getVelocityAtTime(float t) const { return 0.0f; }
  float getAccelerationAtTime(float t) const { return 0.0f; }
  float getJerkAtTime(float t) const { return 0.0f; }
  void reset() {}

};
