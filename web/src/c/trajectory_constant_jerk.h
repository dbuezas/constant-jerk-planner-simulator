
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
 * All junctions have a=0 (no non-zero boundary accelerations).
 *
 * ROOT: Plan 7-phase S-curve
 * |
 * +- STEP 1: Determine feasible peak velocity v_peak
 * |   |
 * |   |  [Binary search: find v_peak in [v_lo, v_nom]
 * |   |   such that dist(v0 -> v_peak) + dist(v_peak -> v1) <= distance]
 * |   |
 * |   +- Can reach v_nom within distance?
 * |       +- YES -> v_peak = v_nom, cruise phase exists (phase 4 duration > 0)
 * |       +- NO  -> v_peak < v_nom, cruise phase collapses (phase 4 = 0)
 * |
 * +- STEP 2: Plan the ACCEL ramp v0 -> v_peak (phases 1,2,3)
 * |   |
 * |   +- Does accel saturate at a_max?
 * |       +- YES -> full S: [+jerk, cruise_accel, -jerk]
 * |       +- NO  -> triangle: [+jerk, -jerk] (phase 2 = 0)
 * |
 * +- STEP 3: Cruise phase (phase 4)
 * |   +- v_peak = v_nom AND distance remaining > 0 -> phase 4 duration > 0
 * |   +- otherwise -> phase 4 duration = 0
 * |
 * +- STEP 4: Plan the DECEL ramp v_peak -> v1 (phases 5,6,7)
 *     |
 *     +- Does decel saturate at a_max?
 *         +- YES -> full S: [-jerk, cruise_decel, +jerk]
 *         +- NO  -> triangle: [-jerk, +jerk] (phase 6 = 0)
 */
class ConstantJerkTrajectoryGenerator {
 public:
  ConstantJerkTrajectoryGenerator() = default;

  void plan(float initial_speed_in,
            float final_speed_in,
            float accel_max_in, float jerk_in,
            float distance_in, float v_nominal_in) {
    reset();

    v0 = initial_speed_in;
    v1 = final_speed_in;
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

    // 3-phase accel ramp (v0 -> v_peak) + cruise + 3-phase decel ramp (v_peak -> v1)

    // Accel ramp: v0 -> v_peak, phases [+j, 0, -j]
    // Triangle: a_up = sqrt(j * dv)
    // Trapezoidal: a_up = a_max, t2 fills remaining dv
    auto planAccel = [&](float v_peak, float& pa, float& pb, float& pc) -> float {
      float dv = v_peak - v0;
      float a_up_sq = j * dv;
      if (a_up_sq < 0) {
        pa = pb = pc = 0;
        return 0;
      }
      float a_up = sqrtf(a_up_sq);

      if (a_up <= a_max) {
        pa = a_up / j;
        pb = 0;
        pc = a_up / j;
      } else {
        pa = a_max / j;
        pc = a_max / j;
        float dv_no_hold = (a_max * a_max) / j;
        pb = fmaxf(0.0f, (dv - dv_no_hold) / a_max);
      }

      float v = v0, a_v = 0, s = 0;
      simulatePhase(j, pa, v, a_v, s);
      simulatePhase(0, pb, v, a_v, s);
      simulatePhase(-j, pc, v, a_v, s);
      return s;
    };

    // Decel ramp: v_peak -> v1, phases [-j, 0, +j]
    // Triangle: a_dn = sqrt(j * dv)
    // Trapezoidal: a_dn = a_max, t6 fills remaining dv
    auto planDecel = [&](float v_peak, float& pa, float& pb, float& pc) -> float {
      float dv = v_peak - v1;
      float a_dn_abs_sq = j * dv;
      if (a_dn_abs_sq < 0) {
        pa = pb = pc = 0;
        return 0;
      }
      float a_dn_abs = sqrtf(a_dn_abs_sq);

      if (a_dn_abs <= a_max) {
        pa = a_dn_abs / j;
        pb = 0;
        pc = a_dn_abs / j;
      } else {
        pa = a_max / j;
        pc = a_max / j;
        float dv_no_hold = (a_max * a_max) / j;
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

    // Minimum feasible v_peak: must be >= both v0 and v1
    float v_lo = fmaxf(0.0f, fmaxf(v0, v1));

    float v_peak = fmaxf(v_lo, v_nominal);
    float s_ramps = totalRampDist(v_peak);

    if (s_ramps > distance) {
      // v_peak doesn't fit -- binary search until undershoot < tolerance
      float hi = v_peak;
      if (totalRampDist(v_lo) > distance) {
        CJP_SET_STATUS("minimum ramp distance exceeds block distance");
        return;
      }
      for (int i = 0; i < 48; i++) {
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
    if (t <= 0.0f) return 0.0f;
    if (t >= total_duration) return 0.0f;
    const int ph = findPhase(t);
    const float dt = t - phase_start_time[ph];
    return phase_start_a[ph] + phaseJerk(ph) * dt;
  }

  float getJerkAtTime(float t) const {
    if (t <= 0.0f || t >= total_duration) return 0.0f;
    return phaseJerk(findPhase(t));
  }

  void reset() {
    v0 = v1 = 0.0f;
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

    float v = v0, a = 0.0f, s = 0.0f, t = 0.0f;
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

  float v0 = 0, v1 = 0;
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
