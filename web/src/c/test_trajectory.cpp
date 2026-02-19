#include "trajectory_constant_jerk.h"
#include <stdio.h>
#include <math.h>

static int tests_passed = 0;
static int tests_failed = 0;

static void check(const char *name, float actual, float expected, float tol = 1e-3f) {
  if (fabsf(actual - expected) <= tol) {
    tests_passed++;
  } else {
    tests_failed++;
    printf("  FAIL %s: expected %.6f, got %.6f\n", name, expected, actual);
  }
}

// Simplest case: pure cruise
// v0 = v1 = nominal, a0 = a1 = 0, just constant velocity
void test_pure_cruise() {
  printf("test_pure_cruise\n");
  ConstantJerkTrajectoryGenerator traj;
  // entry_v=10, entry_a=0, exit_v=10, exit_a=0, a_max=500, j_max=8000, dist=35, nominal=10
  traj.plan(10, 0, 10, 0, 500, 8000, 35, 10);

  check("duration", traj.getTotalDuration(), 3.5f);       // 35 / 10 = 3.5s
  check("pos(0)",   traj.getDistanceAtTime(0.0f), 0.0f);
  check("pos(end)", traj.getDistanceAtTime(3.5f), 35.0f);
  check("pos(mid)", traj.getDistanceAtTime(1.75f), 17.5f);
  check("vel(0)",   traj.getVelocityAtTime(0.0f), 10.0f);
  check("vel(mid)", traj.getVelocityAtTime(1.75f), 10.0f);
  check("vel(end)", traj.getVelocityAtTime(3.5f), 10.0f);
  check("acc(mid)", traj.getAccelerationAtTime(1.75f), 0.0f);
  check("jerk(mid)", traj.getJerkAtTime(1.75f), 0.0f);
}

// From rest to rest, with cruise phase
// v0=0, v1=0, a0=0, a1=0, nominal=31, a_max=500, j=8000, dist=35
void test_rest_to_rest_with_cruise() {
  printf("test_rest_to_rest_with_cruise\n");
  ConstantJerkTrajectoryGenerator traj;
  traj.plan(0, 0, 0, 0, 500, 8000, 35, 31);
  float dur = traj.getTotalDuration();

  check("duration>0",  dur > 0 ? 1 : 0, 1);
  check("pos(0)",      traj.getDistanceAtTime(0), 0);
  check("pos(end)",    traj.getDistanceAtTime(dur), 35);
  check("vel(0)",      traj.getVelocityAtTime(0), 0);
  check("vel(end)",    traj.getVelocityAtTime(dur), 0);
  check("acc(0)",      traj.getAccelerationAtTime(0), 0);
  check("acc(end)",    traj.getAccelerationAtTime(dur), 0);
  // Peak velocity should be close to nominal (31)
  check("vel(mid)",    traj.getVelocityAtTime(dur / 2) > 25 ? 1 : 0, 1);
}

// From rest to rest, short distance — no cruise, triangle accel
void test_rest_to_rest_no_cruise() {
  printf("test_rest_to_rest_no_cruise\n");
  ConstantJerkTrajectoryGenerator traj;
  traj.plan(0, 0, 0, 0, 500, 8000, 2, 31);
  float dur = traj.getTotalDuration();

  check("duration>0",  dur > 0 ? 1 : 0, 1);
  check("pos(0)",      traj.getDistanceAtTime(0), 0);
  check("pos(end)",    traj.getDistanceAtTime(dur), 2);
  check("vel(0)",      traj.getVelocityAtTime(0), 0);
  check("vel(end)",    traj.getVelocityAtTime(dur), 0);
  // Peak velocity should be well below nominal
  float v_mid = traj.getVelocityAtTime(dur / 2);
  check("vel(mid)<nom", v_mid < 31 ? 1 : 0, 1);
  check("vel(mid)>0",   v_mid > 0 ? 1 : 0, 1);
}

// Asymmetric: start at rest, exit at speed
// v0=0, v1=15, a0=0, a1=0
void test_rest_to_moving() {
  printf("test_rest_to_moving\n");
  ConstantJerkTrajectoryGenerator traj;
  traj.plan(0, 0, 15, 0, 500, 8000, 35, 31);
  float dur = traj.getTotalDuration();

  check("duration>0",  dur > 0 ? 1 : 0, 1);
  check("pos(end)",    traj.getDistanceAtTime(dur), 35);
  check("vel(0)",      traj.getVelocityAtTime(0), 0);
  check("vel(end)",    traj.getVelocityAtTime(dur), 15);
}

// Decel only: v0=20, v1=0, cruise at v0 then decel
void test_moving_to_rest() {
  printf("test_moving_to_rest\n");
  ConstantJerkTrajectoryGenerator traj;
  traj.plan(20, 0, 0, 0, 500, 8000, 35, 31);
  float dur = traj.getTotalDuration();

  check("duration>0",  dur > 0 ? 1 : 0, 1);
  check("pos(end)",    traj.getDistanceAtTime(dur), 35);
  check("vel(0)",      traj.getVelocityAtTime(0), 20);
  check("vel(end)",    traj.getVelocityAtTime(dur), 0);
}

// Positive entry accel: v0=0, a0=200, v1=0, a1=0
// Must account for velocity gained while unwinding a0
void test_positive_entry_accel() {
  printf("test_positive_entry_accel\n");
  ConstantJerkTrajectoryGenerator traj;
  traj.plan(0, 200, 0, 0, 500, 8000, 35, 31);
  float dur = traj.getTotalDuration();

  check("duration>0", dur > 0 ? 1 : 0, 1);
  check("pos(end)",   traj.getDistanceAtTime(dur), 35);
  check("vel(0)",     traj.getVelocityAtTime(0), 0);
  check("vel(end)",   traj.getVelocityAtTime(dur), 0);
  check("acc(0)",     traj.getAccelerationAtTime(0), 200);
  check("acc(end)",   traj.getAccelerationAtTime(dur), 0);
}

// Negative exit accel: v0=0, a0=0, v1=0, a1=-200
// Decel ramp must end with a = -200
void test_negative_exit_accel() {
  printf("test_negative_exit_accel\n");
  ConstantJerkTrajectoryGenerator traj;
  traj.plan(0, 0, 0, -200, 500, 8000, 35, 31);
  float dur = traj.getTotalDuration();

  check("duration>0", dur > 0 ? 1 : 0, 1);
  check("pos(end)",   traj.getDistanceAtTime(dur), 35);
  check("vel(0)",     traj.getVelocityAtTime(0), 0);
  check("vel(end)",   traj.getVelocityAtTime(dur), 0);
  check("acc(0)",     traj.getAccelerationAtTime(0), 0);
  check("acc(end)",   traj.getAccelerationAtTime(dur), -200);
}

// Negative entry accel: entering while decelerating
// v0=15, a0=-300, v1=0, a1=0
void test_negative_entry_accel() {
  printf("test_negative_entry_accel\n");
  ConstantJerkTrajectoryGenerator traj;
  traj.plan(15, -300, 0, 0, 500, 8000, 35, 31);
  float dur = traj.getTotalDuration();

  check("duration>0", dur > 0 ? 1 : 0, 1);
  check("pos(end)",   traj.getDistanceAtTime(dur), 35);
  check("vel(0)",     traj.getVelocityAtTime(0), 15);
  check("vel(end)",   traj.getVelocityAtTime(dur), 0);
  check("acc(0)",     traj.getAccelerationAtTime(0), -300);
  check("acc(end)",   traj.getAccelerationAtTime(dur), 0);
}

// Both nonzero: v0=5, a0=100, v1=5, a1=-100
void test_both_nonzero_accel() {
  printf("test_both_nonzero_accel\n");
  ConstantJerkTrajectoryGenerator traj;
  traj.plan(5, 100, 5, -100, 500, 8000, 35, 31);
  float dur = traj.getTotalDuration();

  check("duration>0", dur > 0 ? 1 : 0, 1);
  check("pos(end)",   traj.getDistanceAtTime(dur), 35);
  check("vel(0)",     traj.getVelocityAtTime(0), 5);
  check("vel(end)",   traj.getVelocityAtTime(dur), 5);
  check("acc(0)",     traj.getAccelerationAtTime(0), 100);
  check("acc(end)",   traj.getAccelerationAtTime(dur), -100);
}

// Positive exit accel: handing off to next block while accelerating
// v0=0, a0=0, v1=10, a1=200
void test_positive_exit_accel() {
  printf("test_positive_exit_accel\n");
  ConstantJerkTrajectoryGenerator traj;
  traj.plan(0, 0, 10, 200, 500, 8000, 35, 31);
  float dur = traj.getTotalDuration();

  check("duration>0", dur > 0 ? 1 : 0, 1);
  check("pos(end)",   traj.getDistanceAtTime(dur), 35);
  check("vel(0)",     traj.getVelocityAtTime(0), 0);
  check("vel(end)",   traj.getVelocityAtTime(dur), 10);
  check("acc(0)",     traj.getAccelerationAtTime(0), 0);
  check("acc(end)",   traj.getAccelerationAtTime(dur), 200);
}

int main() {
  test_pure_cruise();
  test_rest_to_rest_with_cruise();
  test_rest_to_rest_no_cruise();
  test_rest_to_moving();
  test_moving_to_rest();
  test_positive_entry_accel();
  test_negative_exit_accel();
  test_negative_entry_accel();
  test_both_nonzero_accel();
  test_positive_exit_accel();

  printf("\n%d passed, %d failed\n", tests_passed, tests_failed);
  return tests_failed > 0 ? 1 : 0;
}
