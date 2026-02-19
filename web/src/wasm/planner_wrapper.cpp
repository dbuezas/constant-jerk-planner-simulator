#include "planner_wrapper.h"

#include "constant-jerk-planner.h"
#include "trajectory_constant_jerk.h"
#include <chrono>

static ConstantJerkTrajectoryGenerator g_traj;
static float g_plan_time_us = 0;

int cjp_plan_single_block(float mm, float max_entry_speed, float nominal, float a_max, float j_max) {
  cjp_reset();
  const int ok = cjp_push_block(mm, max_entry_speed, nominal, a_max, j_max);
  if (!ok) return 0;
  return cjp_recalculate();
}

int cjp_get_first_block(float *out9) {
  if (!out9 || cjp_size() == 0) return 0;
  CJP_BlockOut out;
  const int ok = cjp_get_block(0, &out);
  if (!ok) return 0;
  out9[0] = out.millimeters;
  out9[1] = out.max_entry_speed;
  out9[2] = out.nominal;
  out9[3] = out.a_max;
  out9[4] = out.j_max;
  out9[5] = out.entry_v;
  out9[6] = out.entry_a;
  out9[7] = out.exit_v;
  out9[8] = out.exit_a;
  return 1;
}

void cjp_traj_reset(void) {
  g_traj.reset();
}

int cjp_traj_plan(float entry_v, float entry_a, float exit_v, float exit_a,
                  float a_max, float j_max, float mm, float nominal) {
  auto start = std::chrono::high_resolution_clock::now();
  int n = 0;
  float elapsed_us;
  do {
    g_traj.plan(entry_v, entry_a, exit_v, exit_a, a_max, j_max, mm, nominal);
    n++;
    elapsed_us = std::chrono::duration<float, std::micro>(
        std::chrono::high_resolution_clock::now() - start).count();
  } while (elapsed_us < 1000.0f);
  g_plan_time_us = elapsed_us / n;
  return g_traj.getTotalDuration() > 0.0f ? 1 : 0;
}

float cjp_traj_plan_time_us(void) {
  return g_plan_time_us;
}

const char *cjp_traj_status(void) {
  return g_traj.getStatus();
}

float cjp_traj_duration(void) {
  return g_traj.getTotalDuration();
}

float cjp_traj_position(float t) {
  return g_traj.getDistanceAtTime(t);
}

float cjp_traj_velocity(float t) {
  return g_traj.getVelocityAtTime(t);
}

float cjp_traj_acceleration(float t) {
  return g_traj.getAccelerationAtTime(t);
}

float cjp_traj_jerk(float t) {
  return g_traj.getJerkAtTime(t);
}
