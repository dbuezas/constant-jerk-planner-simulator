
#pragma once

#ifndef CJP_DEBUG_STRINGS
#define CJP_DEBUG_STRINGS 1
#endif

#if CJP_DEBUG_STRINGS
#include <stdio.h>
#define CJP_SET_STATUS(...) snprintf(status_buf, sizeof(status_buf), __VA_ARGS__)
#else
#define CJP_SET_STATUS(...) ((void)0)
#endif

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
            float distance_in, float v_nominal_in) {
    reset();

    v0 = initial_speed_in;
    a0 = fmaxf(-accel_max_in, fminf(initial_accel_in, accel_max_in));
    v1 = final_speed_in;
    a1 = fmaxf(-accel_max_in, fminf(final_accel_in, accel_max_in));
    a_max = accel_max_in;
    j = jerk_in;
    distance = distance_in;

    if (distance <= 0.0f) {
      CJP_SET_STATUS("distance <= 0");
      return;
    }
    if (j <= 0.0f) {
      CJP_SET_STATUS("j_max <= 0");
      return;
    }
    if (a_max <= 0.0f) {
      CJP_SET_STATUS("a_max <= 0");
      return;
    }

    const float v_nominal = fmaxf(0.0f, v_nominal_in);

    // 3-phase accel ramp (v0 → v_peak) + cruise + 3-phase decel ramp (v_peak → v1)

    // Accel ramp: (v0, a0) → (v_peak, 0), phases [+j, 0, -j]
    // a_up = peak accel during ramp. t1=(a_up-a0)/j, t3=a_up/j
    // Triangle: a_up = sqrt(j*Δv + 0.5*a0²)
    // Trapezoidal: a_up = a_max, t2 fills remaining Δv
    auto planAccel = [&](float v_peak, float& pa, float& pb, float& pc) -> float {
      float dv = v_peak - v0;
      float a_up_sq = j * dv + 0.5f * a0 * a0;
      if (a_up_sq < 0) {
        pa = pb = pc = 0;
        return 0;
      }
      float a_up = sqrtf(a_up_sq);

      if (a_up <= a_max) {
        pa = fmaxf(0.0f, (a_up - a0) / j);
        pb = 0;
        pc = a_up / j;
      } else {
        pa = fmaxf(0.0f, (a_max - a0) / j);
        pc = a_max / j;
        float dv_no_hold = (a_max * a_max - 0.5f * a0 * a0) / j;
        pb = fmaxf(0.0f, (dv - dv_no_hold) / a_max);
      }

      float v = v0, a_v = a0, s = 0;
      simulatePhase(j, pa, v, a_v, s);
      simulatePhase(0, pb, v, a_v, s);
      simulatePhase(-j, pc, v, a_v, s);
      return s;
    };

    // Decel ramp: (v_peak, 0) → (v1, a1), phases [-j, 0, +j]
    // |a_dn| = peak decel magnitude. t5=|a_dn|/j, t7=(a1+|a_dn|)/j
    // Triangle: |a_dn| = sqrt(j*Δv + 0.5*a1²)
    // Trapezoidal: |a_dn| = a_max, t6 fills remaining Δv
    auto planDecel = [&](float v_peak, float& pa, float& pb, float& pc) -> float {
      float dv = v_peak - v1;
      float a_dn_abs_sq = j * dv + 0.5f * a1 * a1;
      if (a_dn_abs_sq < 0) {
        pa = pb = pc = 0;
        return 0;
      }
      float a_dn_abs = sqrtf(a_dn_abs_sq);

      if (a_dn_abs <= a_max) {
        pa = a_dn_abs / j;
        pb = 0;
        pc = fmaxf(0.0f, (a_dn_abs + a1) / j);
      } else {
        pa = a_max / j;
        pc = fmaxf(0.0f, (a_max + a1) / j);
        float dv_no_hold = (a_max * a_max - 0.5f * a1 * a1) / j;
        pb = fmaxf(0.0f, (dv - dv_no_hold) / a_max);
      }

      float v = v_peak, a_v = 0, s = 0;
      simulatePhase(-j, pa, v, a_v, s);
      simulatePhase(0, pb, v, a_v, s);
      simulatePhase(j, pc, v, a_v, s);
      return s;
    };

    auto totalRampDist = [&](float vp) -> float {
      float a, b, c;
      return planAccel(vp, a, b, c) + planDecel(vp, a, b, c);
    };

    // Minimum feasible v_peak from each ramp constraint
    float v_min_accel = (a0 >= 0) ? v0 + 0.5f * a0 * a0 / j : v0 - 0.5f * a0 * a0 / j;
    float v_min_decel = (a1 <= 0) ? v1 + 0.5f * a1 * a1 / j : v1 - 0.5f * a1 * a1 / j;
    float v_lo = fmaxf(0.0f, fmaxf(v_min_accel, v_min_decel));

    float v_peak = fmaxf(v_lo, v_nominal);
    float s_ramps = totalRampDist(v_peak);

    if (s_ramps > distance) {
      // v_peak doesn't fit — binary search until undershoot < tolerance
      float hi = v_peak;
      if (totalRampDist(v_lo) > distance) {
        CJP_SET_STATUS("minimum ramp distance exceeds block distance");
        return;
      }
      for (int i = 0; i < 48; i++) {
        // CJP_SET_STATUS("iterations: %d", i + 1);
        float mid = 0.5f * (v_lo + hi);
        float s_mid = totalRampDist(mid);
        if (s_mid > distance)
          hi = mid;
        else
          v_lo = mid;
        if (distance - s_mid >= 0 && distance - s_mid < 0.01f) break;
      }
      v_peak = v_lo;
    }

    // Set phase durations and compute ramp distances
    float s_accel = planAccel(v_peak, t1, t2, t3);
    float s_decel = planDecel(v_peak, t5, t6, t7);

    // Cruise phase absorbs any remaining distance
    s_ramps = s_accel + s_decel;
    if (v_peak > 0.0f && distance > s_ramps)
      t4 = (distance - s_ramps) / v_peak;

    total_duration = t1 + t2 + t3 + t4 + t5 + t6 + t7;
    buildPhaseCache();
    CJP_SET_STATUS("OK");
  }

  float getTotalDuration() const { return total_duration; }
#if CJP_DEBUG_STRINGS
  const char* getStatus() const { return status_buf; }
#else
  const char* getStatus() const { return ""; }
#endif

  float getDistanceAtTime(float t) const {
    if (t <= 0.0f) return 0.0f;
    if (t >= total_duration) return distance;
    const int ph = findPhase(t);
    const float dt = t - phase_start_time[ph];
    const float v = phase_start_v[ph];
    const float a = phase_start_a[ph];
    const float jk = phaseJerk(ph);
    return phase_start_pos[ph] + v * dt + 0.5f * a * dt * dt + (1.0f / 6.0f) * jk * dt * dt * dt;
  }

  float getVelocityAtTime(float t) const {
    if (t <= 0.0f) return v0;
    if (t >= total_duration) return v1;
    const int ph = findPhase(t);
    const float dt = t - phase_start_time[ph];
    return phase_start_v[ph] + phase_start_a[ph] * dt + 0.5f * phaseJerk(ph) * dt * dt;
  }

  float getAccelerationAtTime(float t) const {
    if (t <= 0.0f) return a0;
    if (t >= total_duration) return a1;
    const int ph = findPhase(t);
    const float dt = t - phase_start_time[ph];
    return phase_start_a[ph] + phaseJerk(ph) * dt;
  }

  float getJerkAtTime(float t) const {
    if (t <= 0.0f || t >= total_duration) return 0.0f;
    return phaseJerk(findPhase(t));
  }

  void reset() {
    v0 = a0 = v1 = a1 = 0.0f;
    a_max = j = distance = 0.0f;
    t1 = t2 = t3 = t4 = t5 = t6 = t7 = 0.0f;
    total_duration = 0.0f;
    for (int i = 0; i < 7; ++i) {
      phase_dt[i] = 0.0f;
      phase_start_time[i] = 0.0f;
      phase_start_pos[i] = 0.0f;
      phase_start_v[i] = 0.0f;
      phase_start_a[i] = 0.0f;
    }
  }

 private:
  static void simulatePhase(float jerk, float dt, float& v, float& a, float& s) {
    if (dt <= 0.0f) return;
    s += v * dt + 0.5f * a * dt * dt + (1.0f / 6.0f) * jerk * dt * dt * dt;
    v += a * dt + 0.5f * jerk * dt * dt;
    a += jerk * dt;
  }

  void buildPhaseCache() {
    phase_dt[0] = t1;
    phase_dt[1] = t2;
    phase_dt[2] = t3;
    phase_dt[3] = t4;
    phase_dt[4] = t5;
    phase_dt[5] = t6;
    phase_dt[6] = t7;

    float v = v0, a = a0, s = 0.0f, t = 0.0f;
    for (int i = 0; i < 7; ++i) {
      phase_start_time[i] = t;
      phase_start_pos[i] = s;
      phase_start_v[i] = v;
      phase_start_a[i] = a;
      simulatePhase(phaseJerk(i), phase_dt[i], v, a, s);
      t += phase_dt[i];
    }
  }

  int findPhase(float t) const {
    for (int i = 0; i < 7; ++i)
      if (t < phase_start_time[i] + phase_dt[i]) return i;
    return 6;
  }

  float phaseJerk(int phase) const {
    // phases: +j, 0, -j, 0, -j, 0, +j
    switch (phase) {
      case 0:
        return j;
      case 2:
        return -j;
      case 4:
        return -j;
      case 6:
        return j;
      default:
        return 0.0f;
    }
  }

  float v0 = 0, a0 = 0, v1 = 0, a1 = 0;
  float a_max = 0, j = 0, distance = 0;
#if CJP_DEBUG_STRINGS
  char status_buf[128] = "";
#endif
  float t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0, t6 = 0, t7 = 0;
  float total_duration = 0;
  float phase_dt[7] = {};
  float phase_start_time[7] = {};
  float phase_start_pos[7] = {};
  float phase_start_v[7] = {};
  float phase_start_a[7] = {};
};
