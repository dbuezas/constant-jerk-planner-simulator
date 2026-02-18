
#pragma once

#include <math.h>

/**
 * Constant-jerk trajectory generator.
 * Builds a 7-phase S-curve using bang-bang jerk and accel limits.
 */
class ConstantJerkTrajectoryGenerator {
public:
  ConstantJerkTrajectoryGenerator() = default;

  // Plan a 7-phase S-curve with jerk/accel limits and endpoint (v,a).
  // distance is the block length. jerk_in is |j|. v_nominal is peak target speed.
  void plan(const float initial_speed_in, const float initial_accel_in,
            const float final_speed_in, const float final_accel_in,
            const float accel_max_in, const float jerk_in,
            const float distance_in, const float v_nominal_in) {

    reset();

    v0 = initial_speed_in;
    a0 = fmaxf(-accel_max_in, fminf(initial_accel_in, accel_max_in));
    v1 = final_speed_in;
    a1 = fmaxf(-accel_max_in, fminf(final_accel_in, accel_max_in));
    a_max = accel_max_in;
    j = jerk_in;
    distance = distance_in;

    if (distance <= 0.0f || j <= 0.0f || a_max <= 0.0f) {
      total_duration = 0.0f;
      return;
    }

    const float v_nominal = fmaxf(0.0f, v_nominal_in);
    const float v_eps = 1e-6f;

    auto compute_v_after3_no_hold = [&](const float a_up) {
      float v_temp = v0;
      float a_temp = a0;
      float s_temp = 0.0f;
      const float t1_local = fmaxf(0.0f, (a_up - a0) / j);
      const float t3_local = fmaxf(0.0f, a_up / j);
      simulatePhase(j, t1_local, v_temp, a_temp, s_temp);
      simulatePhase(-j, t3_local, v_temp, a_temp, s_temp);
      return v_temp;
    };

    auto compute_v_after3_with_hold = [&](const float a_up, const float v_peak) {
      float v_temp = v0;
      float a_temp = a0;
      float s_temp = 0.0f;
      const float t1_local = fmaxf(0.0f, (a_up - a0) / j);
      const float t3_local = fmaxf(0.0f, a_up / j);
      simulatePhase(j, t1_local, v_temp, a_temp, s_temp);
      const float v_after3_no_hold = v_temp;
      float t2_local = 0.0f;
      if (v_peak > v_after3_no_hold + v_eps && a_up > 0.0f) {
        t2_local = (v_peak - v_after3_no_hold) / a_up;
      }
      simulatePhase(0.0f, t2_local, v_temp, a_temp, s_temp);
      simulatePhase(-j, t3_local, v_temp, a_temp, s_temp);
      return v_temp;
    };

    auto compute_profile = [&](const float v_peak, const float a_up, const float a_dn, const bool allow_holds,
                               float &out_s_no_cruise, float &out_v_after3) {
      const float t1_local = fmaxf(0.0f, (a_up - a0) / j);
      const float t3_local = fmaxf(0.0f, a_up / j);
      const float t5_local = fmaxf(0.0f, (-a_dn) / j);
      const float t7_local = fmaxf(0.0f, (a1 - a_dn) / j);

      t1 = t1_local;
      t3 = t3_local;
      t5 = t5_local;
      t7 = t7_local;
      t2 = 0.0f;
      t4 = 0.0f;
      t6 = 0.0f;

      float v_temp = v0;
      float a_temp = a0;
      float s_temp = 0.0f;
      simulatePhase(j, t1, v_temp, a_temp, s_temp);
      simulatePhase(0.0f, t2, v_temp, a_temp, s_temp);
      simulatePhase(-j, t3, v_temp, a_temp, s_temp);
      const float v_after3_no_hold = v_temp;

      if (allow_holds && v_peak > v_after3_no_hold + v_eps && a_up > 0.0f) {
        t2 = (v_peak - v_after3_no_hold) / a_up;
      }

      v_temp = v0;
      a_temp = a0;
      s_temp = 0.0f;
      simulatePhase(j, t1, v_temp, a_temp, s_temp);
      simulatePhase(0.0f, t2, v_temp, a_temp, s_temp);
      simulatePhase(-j, t3, v_temp, a_temp, s_temp);
      const float v_after3 = v_temp;

      simulatePhase(-j, t5, v_temp, a_temp, s_temp);
      simulatePhase(0.0f, t6, v_temp, a_temp, s_temp);
      simulatePhase(j, t7, v_temp, a_temp, s_temp);
      const float v_no_hold = v_temp;

      const float dv_needed = v_no_hold - v1;
      if (allow_holds && dv_needed > v_eps && a_dn < 0.0f) {
        t6 = dv_needed / (-a_dn);
        if (t6 < 0.0f) t6 = 0.0f;
      }

      v_temp = v0;
      a_temp = a0;
      s_temp = 0.0f;
      simulatePhase(j, t1, v_temp, a_temp, s_temp);
      simulatePhase(0.0f, t2, v_temp, a_temp, s_temp);
      simulatePhase(-j, t3, v_temp, a_temp, s_temp);
      out_v_after3 = v_temp;
      simulatePhase(-j, t5, v_temp, a_temp, s_temp);
      simulatePhase(0.0f, t6, v_temp, a_temp, s_temp);
      simulatePhase(j, t7, v_temp, a_temp, s_temp);
      out_s_no_cruise = s_temp;
    };

    const float v_limit_eps = 1e-3f;

    auto phase_extrema_ok = [&](const float v_start, const float a_start, const float jerk, const float dt) {
      float v_min = v_start;
      float v_max = v_start;
      const float v_end = v_start + a_start * dt + 0.5f * jerk * dt * dt;
      if (v_end < v_min) v_min = v_end;
      if (v_end > v_max) v_max = v_end;
      if (fabsf(jerk) > v_eps) {
        const float t_vertex = -a_start / jerk;
        if (t_vertex > 0.0f && t_vertex < dt) {
          const float v_vertex = v_start + a_start * t_vertex + 0.5f * jerk * t_vertex * t_vertex;
          if (v_vertex < v_min) v_min = v_vertex;
          if (v_vertex > v_max) v_max = v_vertex;
        }
      }
      return v_min >= -v_limit_eps && v_max <= v_nominal + v_limit_eps;
    };

    auto solve_accel_plateau = [&](const float v_peak, float &a_up_out, float &a_dn_out) {
      float a_up_lo = fmaxf(0.0f, a0);
      float a_up_hi = a_max;
      float a_dn_lo = -a_max;
      float a_dn_hi = fminf(0.0f, a1);

      if (a_up_lo > a_up_hi || a_dn_lo > a_dn_hi) return false;

      const float v_after3_lo = compute_v_after3_no_hold(a_up_lo);
      const float v_after3_hi = compute_v_after3_with_hold(a_up_hi, v_peak);

      if (v_peak + v_eps < v_after3_lo) return false;

      a_up_out = a_up_hi;
      if (v_peak <= v_after3_hi + v_eps) {
        for (int iter = 0; iter < 32; ++iter) {
          const float a_mid = 0.5f * (a_up_lo + a_up_hi);
          const float v_after3 = compute_v_after3_no_hold(a_mid);
          if (v_after3 > v_peak) {
            a_up_hi = a_mid;
          } else {
            a_up_lo = a_mid;
          }
        }
        a_up_out = a_up_lo;
      }

      a_dn_out = a_dn_lo;
      if (a_dn_lo == a_dn_hi) return true;

      float v_temp = v0;
      float a_temp = a0;
      float s_temp = 0.0f;
      const float t1_local = fmaxf(0.0f, (a_up_out - a0) / j);
      const float t3_local = fmaxf(0.0f, a_up_out / j);
      simulatePhase(j, t1_local, v_temp, a_temp, s_temp);
      const float v_after3_with_hold = compute_v_after3_with_hold(a_up_out, v_peak);

      auto v_after7_no_hold = [&](const float a_dn) {
        const float t5_local = fmaxf(0.0f, (-a_dn) / j);
        const float t7_local = fmaxf(0.0f, (a1 - a_dn) / j);
        float v_local = v_after3_with_hold;
        float a_local = 0.0f;
        float s_local = 0.0f;
        simulatePhase(-j, t5_local, v_local, a_local, s_local);
        simulatePhase(j, t7_local, v_local, a_local, s_local);
        return v_local;
      };

      float best_dn = a_dn_hi;
      for (int iter = 0; iter < 32; ++iter) {
        const float a_mid = 0.5f * (a_dn_lo + a_dn_hi);
        const float v_end = v_after7_no_hold(a_mid);
        if (v_end < v1) {
          a_dn_lo = a_mid;
        } else {
          a_dn_hi = a_mid;
          best_dn = a_mid;
        }
      }
      a_dn_out = best_dn;
      return true;
    };

    auto validate_profile = [&]() {
      float v_temp = v0;
      float a_temp = a0;
      float s_temp = 0.0f;
      if (!phase_extrema_ok(v_temp, a_temp, j, t1)) return false;
      simulatePhase(j, t1, v_temp, a_temp, s_temp);
      if (!phase_extrema_ok(v_temp, a_temp, 0.0f, t2)) return false;
      simulatePhase(0.0f, t2, v_temp, a_temp, s_temp);
      if (!phase_extrema_ok(v_temp, a_temp, -j, t3)) return false;
      simulatePhase(-j, t3, v_temp, a_temp, s_temp);
      if (!phase_extrema_ok(v_temp, a_temp, 0.0f, t4)) return false;
      simulatePhase(0.0f, t4, v_temp, a_temp, s_temp);
      if (!phase_extrema_ok(v_temp, a_temp, -j, t5)) return false;
      simulatePhase(-j, t5, v_temp, a_temp, s_temp);
      if (!phase_extrema_ok(v_temp, a_temp, 0.0f, t6)) return false;
      simulatePhase(0.0f, t6, v_temp, a_temp, s_temp);
      if (!phase_extrema_ok(v_temp, a_temp, j, t7)) return false;
      return true;
    };

    auto distance_feasible = [&](const float v_after3_local, const float s_no_cruise_local) {
      if (v_after3_local <= v_eps) {
        return distance <= s_no_cruise_local + 1e-3f;
      }
      return true;
    };

    float v_peak_lo = fmaxf(0.0f, v0);
    float v_peak_hi = fmaxf(v_peak_lo, v_nominal - 1e-5f);
    float s_no_cruise = 0.0f;
    float v_after3 = v0;
    float a_up = fmaxf(0.0f, a0);
    float a_dn = fminf(0.0f, a1);

    // Fast path: if nominal is reachable with cruise, lock v_peak = v_nominal.
    {
      if (solve_accel_plateau(v_nominal, a_up, a_dn)) {
        compute_profile(v_nominal, a_up, a_dn, true, s_no_cruise, v_after3);
        if (validate_profile() && s_no_cruise <= distance && v_after3 > v_eps) {
          t4 = (distance - s_no_cruise) / v_after3;
          total_duration = t1 + t2 + t3 + t4 + t5 + t6 + t7;
          buildPhaseCache();
          return;
        }
      }
    }

    for (int iter = 0; iter < 48; ++iter) {
      const float v_mid = 0.5f * (v_peak_lo + v_peak_hi);
      if (!solve_accel_plateau(v_mid, a_up, a_dn)) {
        v_peak_hi = v_mid;
        continue;
      }
      compute_profile(v_mid, a_up, a_dn, true, s_no_cruise, v_after3);
      const bool profile_ok = validate_profile();
      const bool distance_ok = distance_feasible(v_after3, s_no_cruise);
      if (!profile_ok) {
        v_peak_hi = v_mid;
      } else if (!distance_ok) {
        v_peak_lo = v_mid;
      } else if (s_no_cruise > distance) {
        v_peak_hi = v_mid;
      } else {
        v_peak_lo = v_mid;
      }
    }

    if (s_no_cruise > distance && v_peak_lo > v_eps) {
      float v_tri_lo = fmaxf(0.0f, v0);
      float v_tri_hi = v_peak_lo;
      for (int iter = 0; iter < 48; ++iter) {
        const float v_mid = 0.5f * (v_tri_lo + v_tri_hi);
        if (!solve_accel_plateau(v_mid, a_up, a_dn)) {
          v_tri_hi = v_mid;
          continue;
        }
        compute_profile(v_mid, a_up, a_dn, false, s_no_cruise, v_after3);
        const bool valid = validate_profile();
        if (!valid || s_no_cruise > distance) {
          v_tri_hi = v_mid;
        } else {
          v_tri_lo = v_mid;
        }
      }
      v_peak_lo = v_tri_lo;
    }

    solve_accel_plateau(v_peak_lo, a_up, a_dn);
    compute_profile(v_peak_lo, a_up, a_dn, true, s_no_cruise, v_after3);

    if (v_after3 > v_eps) {
      t4 = (distance - s_no_cruise) / v_after3;
      if (t4 < 0.0f) t4 = 0.0f;
    }

    if (!validate_profile()) {
      bool recovered = false;
      float v_try_hi = v_peak_lo;
      float v_try_lo = fmaxf(0.0f, v0);
      for (int iter = 0; iter < 32; ++iter) {
        const float v_try = 0.5f * (v_try_lo + v_try_hi);
        if (!solve_accel_plateau(v_try, a_up, a_dn)) {
          v_try_hi = v_try;
          continue;
        }
        compute_profile(v_try, a_up, a_dn, true, s_no_cruise, v_after3);
        if (v_after3 > v_eps) {
          t4 = (distance - s_no_cruise) / v_after3;
          if (t4 < 0.0f) t4 = 0.0f;
        }
        if (validate_profile() && distance_feasible(v_after3, s_no_cruise)) {
          recovered = true;
          break;
        }
        v_try_hi = v_try;
      }
      if (!recovered) {
        total_duration = 0.0f;
        return;
      }
    }

    total_duration = t1 + t2 + t3 + t4 + t5 + t6 + t7;
    buildPhaseCache();
  }

  float getDistanceAtTime(const float t) const {
    if (t <= 0.0f) return 0.0f;
    if (t >= total_duration) return distance;

    const int phase = findPhase(t);
    const float dt = t - phase_start_time[phase];
    return phase_start_pos[phase] + distanceFromPhaseStart(phase, dt);
  }

  float getTotalDuration() const {
    return total_duration;
  }

  float getVelocityAtTime(const float t) const {
    if (t <= 0.0f) return v0;
    if (t >= total_duration) return v1;
    const int phase = findPhase(t);
    const float dt = t - phase_start_time[phase];
    const float v = phase_start_v[phase];
    const float a = phase_start_a[phase];
    const float jv = phaseJerk(phase);
    return v + a * dt + 0.5f * jv * dt * dt;
  }

  float getAccelerationAtTime(const float t) const {
    if (t <= 0.0f) return a0;
    if (t >= total_duration) return a1;
    const int phase = findPhase(t);
    const float dt = t - phase_start_time[phase];
    const float a = phase_start_a[phase];
    const float jv = phaseJerk(phase);
    return a + jv * dt;
  }

  float getJerkAtTime(const float t) const {
    if (t <= 0.0f || t >= total_duration) return 0.0f;
    const int phase = findPhase(t);
    return phaseJerk(phase);
  }

  void reset() {
    v0 = a0 = v1 = a1 = 0.0f;
    a_max = j = 0.0f;
    distance = 0.0f;
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

protected:
  static inline void simulatePhase(const float jerk, const float dt, float &v, float &a, float &s) {
    if (dt <= 0.0f) return;
    s += v * dt + 0.5f * a * dt * dt + (1.0f / 6.0f) * jerk * dt * dt * dt;
    v += a * dt + 0.5f * jerk * dt * dt;
    a += jerk * dt;
  }

  static inline void stepPhase(const float jerk, const float phase_dt, float &time, float &v, float &a, float &s) {
    if (time <= 0.0f || phase_dt <= 0.0f) return;
    const float dt = time < phase_dt ? time : phase_dt;
    simulatePhase(jerk, dt, v, a, s);
    time -= dt;
  }

  void buildPhaseCache() {
    phase_dt[0] = t1;
    phase_dt[1] = t2;
    phase_dt[2] = t3;
    phase_dt[3] = t4;
    phase_dt[4] = t5;
    phase_dt[5] = t6;
    phase_dt[6] = t7;

    float v = v0;
    float a = a0;
    float s = 0.0f;
    float t = 0.0f;

    for (int i = 0; i < 7; ++i) {
      const float jerk = phaseJerk(i);
      phase_start_time[i] = t;
      phase_start_pos[i] = s;
      phase_start_v[i] = v;
      phase_start_a[i] = a;
      simulatePhase(jerk, phase_dt[i], v, a, s);
      t += phase_dt[i];
    }
  }

  int findPhase(const float t) const {
    for (int i = 0; i < 7; ++i) {
      if (t < phase_start_time[i] + phase_dt[i])
        return i;
    }
    return 6;
  }

  float distanceFromPhaseStart(const int phase, const float dt) const {
    const float v = phase_start_v[phase];
    const float a = phase_start_a[phase];
    const float jv = phaseJerk(phase);
    return v * dt + 0.5f * a * dt * dt + (1.0f / 6.0f) * jv * dt * dt * dt;
  }

  float phaseJerk(const int phase) const {
    switch (phase) {
      case 0: return j;
      case 1: return 0.0f;
      case 2: return -j;
      case 3: return 0.0f;
      case 4: return -j;
      case 5: return 0.0f;
      case 6: return j;
      default: return 0.0f;
    }
  }

  float v0 = 0.0f;
  float a0 = 0.0f;
  float v1 = 0.0f;
  float a1 = 0.0f;
  float a_max = 0.0f;
  float j = 0.0f;
  float distance = 0.0f;

  float t1 = 0.0f;
  float t2 = 0.0f;
  float t3 = 0.0f;
  float t4 = 0.0f;
  float t5 = 0.0f;
  float t6 = 0.0f;
  float t7 = 0.0f;

  float total_duration = 0.0f;

  float phase_dt[7] = {};
  float phase_start_time[7] = {};
  float phase_start_pos[7] = {};
  float phase_start_v[7] = {};
  float phase_start_a[7] = {};
};
