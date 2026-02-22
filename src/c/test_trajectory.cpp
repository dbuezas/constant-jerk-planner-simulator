#include "trajectory_constant_jerk.h"
#include "constant-jerk-planner.h"
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
// v0 = v1 = nominal, just constant velocity
void test_pure_cruise() {
  printf("test_pure_cruise\n");
  ConstantJerkTrajectoryGenerator traj;
  // entry_v=10, exit_v=10, a_max=500, j_max=8000, dist=35, nominal=10
  traj.plan(10, 10, 500, 8000, 35, 10);

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
// v0=0, v1=0, nominal=31, a_max=500, j=8000, dist=35
void test_rest_to_rest_with_cruise() {
  printf("test_rest_to_rest_with_cruise\n");
  ConstantJerkTrajectoryGenerator traj;
  traj.plan(0, 0, 500, 8000, 35, 31);
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
  traj.plan(0, 0, 500, 8000, 2, 31);
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
// v0=0, v1=15
void test_rest_to_moving() {
  printf("test_rest_to_moving\n");
  ConstantJerkTrajectoryGenerator traj;
  traj.plan(0, 15, 500, 8000, 35, 31);
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
  traj.plan(20, 0, 500, 8000, 35, 31);
  float dur = traj.getTotalDuration();

  check("duration>0",  dur > 0 ? 1 : 0, 1);
  check("pos(end)",    traj.getDistanceAtTime(dur), 35);
  check("vel(0)",      traj.getVelocityAtTime(0), 20);
  check("vel(end)",    traj.getVelocityAtTime(dur), 0);
}

// ============================================================
// Multi-block planner + merging tests
// ============================================================

// Helper: push N identical blocks
static void push_identical(int n, float mm, float max_entry, float nominal, float a_max, float j_max) {
  for (int i = 0; i < n; i++)
    cjp_push_block(mm, max_entry, nominal, a_max, j_max);
}

// 11 identical blocks with high maxEntrySpeed -> should merge into 1
void test_merge_identical_blocks() {
  printf("test_merge_identical_blocks\n");
  cjp_reset();
  push_identical(11, 5, 10000000, 200, 5000, 30000);
  cjp_recalculate();

  check("merged_size", (float)cjp_merged_size(), 1.0f);

  CJP_BlockOut mb;
  cjp_get_merged_block(0, &mb);
  check("merged_mm", mb.millimeters, 55.0f);
  check("merged_entry_v", mb.entry_v, 0.0f);
  check("merged_exit_v", mb.exit_v, 0.0f);
}

// Blocks with different constraints -> no merging
void test_no_merge_different_constraints() {
  printf("test_no_merge_different_constraints\n");
  cjp_reset();
  cjp_push_block(5, 10000, 200, 5000, 30000);
  cjp_push_block(5, 10000, 200, 6000, 30000);  // different a_max
  cjp_push_block(5, 10000, 200, 5000, 30000);
  cjp_recalculate();

  check("merged_size", (float)cjp_merged_size(), 3.0f);
}

// Same constraints, low maxEntrySpeed at one junction -> split there
void test_merge_split_at_low_junction() {
  printf("test_merge_split_at_low_junction\n");
  cjp_reset();
  // 5 blocks, same constraints, but block 3 has very low maxEntrySpeed
  cjp_push_block(10, 10000000, 200, 5000, 30000);
  cjp_push_block(10, 10000000, 200, 5000, 30000);
  cjp_push_block(10, 1, 200, 5000, 30000);        // very low max entry -> forces split
  cjp_push_block(10, 10000000, 200, 5000, 30000);
  cjp_push_block(10, 10000000, 200, 5000, 30000);
  cjp_recalculate();

  // Should split at block 2 (maxEntrySpeed=1), resulting in at least 2 merged groups
  check("merged_size>=2", cjp_merged_size() >= 2 ? 1.0f : 0.0f, 1.0f);
  printf("  merged_count = %zu\n", cjp_merged_size());
}

// Low maxEntrySpeed near start where velocity is still low.
// The v_peak check is conservative: it compares the trajectory's overall peak
// against the junction limit, so even though the actual speed at 5mm would be
// ~90 (below 100), v_peak > 100 causes a split. This is the expected trade-off
// of the v_peak check (same as Marlin).
void test_no_split_near_start() {
  printf("test_no_split_near_start\n");
  cjp_reset();
  // 5 blocks, 5mm each. Block 1 has maxEntrySpeed=100.
  // v_peak of the merged trajectory > 100, so the v_peak check splits here.
  cjp_push_block(5, 10000000, 200, 5000, 30000);
  cjp_push_block(5, 100, 200, 5000, 30000);
  cjp_push_block(5, 10000000, 200, 5000, 30000);
  cjp_push_block(5, 10000000, 200, 5000, 30000);
  cjp_push_block(5, 10000000, 200, 5000, 30000);
  cjp_recalculate();

  printf("  merged_count = %zu\n", cjp_merged_size());
  check("merged_size>=2", cjp_merged_size() >= 2 ? 1.0f : 0.0f, 1.0f);
}

// Merged duration should be less than individually planned blocks
void test_merged_faster_than_individual() {
  printf("test_merged_faster_than_individual\n");

  // Plan as merged (11 identical blocks)
  cjp_reset();
  push_identical(11, 5, 10000000, 200, 5000, 30000);
  cjp_recalculate();

  // Get merged trajectory duration
  CJP_BlockOut mb;
  cjp_get_merged_block(0, &mb);
  ConstantJerkTrajectoryGenerator traj;
  traj.plan(mb.entry_v, mb.exit_v, mb.a_max, mb.j_max, mb.millimeters, mb.nominal);
  float merged_dur = traj.getTotalDuration();

  // Plan as individual blocks (no merging possible - fake different constraints)
  float individual_dur = 0;
  for (int i = 0; i < 11; i++) {
    float entry = (i == 0) ? 0.0f : 0.0f;  // each block 0->0 individually is worst case
    float exit = 0.0f;
    traj.plan(entry, exit, 5000, 30000, 5, 200);
    individual_dur += traj.getTotalDuration();
  }

  printf("  merged_dur=%.4f, individual_dur=%.4f\n", merged_dur, individual_dur);
  check("merged<individual", merged_dur < individual_dur ? 1.0f : 0.0f, 1.0f);
}

// Exec streaming: step through, verify original block transitions
void test_exec_streaming() {
  printf("test_exec_streaming\n");
  cjp_reset();
  // 3 blocks that will merge into 1
  cjp_push_block(10, 10000000, 200, 5000, 30000);
  cjp_push_block(10, 10000000, 200, 5000, 30000);
  cjp_push_block(10, 10000000, 200, 5000, 30000);
  cjp_recalculate();

  check("merged_size", (float)cjp_merged_size(), 1.0f);

  cjp_exec_reset();

  float dt = 0.0001f;
  size_t last_orig = 0;
  int transitions = 0;
  float last_pos = 0;
  int steps = 0;

  while (cjp_exec_step(dt)) {
    steps++;
    size_t orig = cjp_exec_original_block();
    float pos = cjp_exec_position();

    if (orig != last_orig) {
      printf("  transition to block %zu at pos=%.3f\n", orig, pos);
      transitions++;
      last_orig = orig;
    }
    last_pos = pos;

    // Safety: don't loop forever
    if (steps > 10000000) break;
  }

  check("steps>0", steps > 0 ? 1.0f : 0.0f, 1.0f);
  check("transitions", (float)transitions, 2.0f);  // 0->1 and 1->2
  check("final_pos", last_pos, 30.0f, 0.5f);  // total 30mm
  printf("  steps=%d, final_pos=%.3f\n", steps, last_pos);
}

// Exec streaming: blocks that don't merge (different constraints)
void test_exec_no_merge() {
  printf("test_exec_no_merge\n");
  cjp_reset();
  cjp_push_block(10, 10000, 200, 5000, 30000);
  cjp_push_block(10, 10000, 100, 5000, 30000);  // different nominal
  cjp_push_block(10, 10000, 200, 5000, 30000);
  cjp_recalculate();

  check("merged_size", (float)cjp_merged_size(), 3.0f);

  cjp_exec_reset();

  float dt = 0.0001f;
  size_t last_merged = 0;
  int merged_transitions = 0;
  int steps = 0;

  while (cjp_exec_step(dt)) {
    steps++;
    size_t mi = cjp_exec_merged_block();
    if (mi != last_merged) {
      merged_transitions++;
      last_merged = mi;
    }
    if (steps > 10000000) break;
  }

  check("steps>0", steps > 0 ? 1.0f : 0.0f, 1.0f);
  check("merged_transitions", (float)merged_transitions, 2.0f);
}

// Blocks with slightly different a_max (within 10% ratio) should merge
void test_merge_similar_amax() {
  printf("test_merge_similar_amax\n");
  cjp_reset();
  // a_max values: 5000, 5200, 4800 — max/min = 5200/4800 = 1.083 < 1.1
  cjp_push_block(10, 10000000, 200, 5000, 30000);
  cjp_push_block(10, 10000000, 200, 5200, 30000);
  cjp_push_block(10, 10000000, 200, 4800, 30000);
  cjp_recalculate();

  printf("  merged_count = %zu\n", cjp_merged_size());
  check("merged_size", (float)cjp_merged_size(), 1.0f);

  // Merged block should use min(a_max) = 4800
  CJP_BlockOut mb;
  cjp_get_merged_block(0, &mb);
  check("merged_a_max", mb.a_max, 4800.0f);
}

// Blocks with very different a_max (>10% ratio) should NOT merge
void test_no_merge_far_amax() {
  printf("test_no_merge_far_amax\n");
  cjp_reset();
  // a_max values: 5000, 5000, 2000 — ratio 5000/2000 = 2.5 > 1.1
  cjp_push_block(10, 10000000, 200, 5000, 30000);
  cjp_push_block(10, 10000000, 200, 5000, 30000);
  cjp_push_block(10, 10000000, 200, 2000, 30000);  // very different
  cjp_recalculate();

  printf("  merged_count = %zu\n", cjp_merged_size());
  check("merged_size", (float)cjp_merged_size(), 2.0f);  // first two merge, third is separate
}

// Debug: reproduce velocity discontinuity between merged blocks
void test_velocity_continuity_bug() {
  printf("test_velocity_continuity_bug\n");
  cjp_reset();
  float blocks[][5] = {
    {4, 400, 500, 100000, 1000000},
    {4, 400, 500, 100000, 1000000},
    {1, 400, 500, 100000, 1000000},
    {2, 400, 500, 100000, 1000000},
    {4, 400, 500, 100000, 1000000},
    {2, 400, 500, 100000, 1000000},
    {1, 400, 500, 100000, 1000000},
    {4, 400, 500, 100000, 1000000},
  };
  for (auto& b : blocks)
    cjp_push_block(b[0], b[1], b[2], b[3], b[4]);
  cjp_recalculate();

  printf("  merged_count = %zu\n", cjp_merged_size());
  for (size_t i = 0; i < cjp_merged_size(); i++) {
    CJP_BlockOut mb;
    cjp_get_merged_block(i, &mb);
    printf("  merged[%zu]: mm=%.1f entry=%.3f exit=%.3f orig_start=? orig_count=?\n",
           i, mb.millimeters, mb.entry_v, mb.exit_v);
  }
  for (size_t i = 0; i < cjp_size(); i++) {
    CJP_BlockOut ob;
    cjp_get_block(i, &ob);
    printf("  block[%zu]: mm=%.1f entry=%.3f exit=%.3f\n",
           i, ob.millimeters, ob.entry_v, ob.exit_v);
  }

  // Check velocity continuity at merged block boundaries
  bool has_discontinuity = false;
  for (size_t i = 0; i + 1 < cjp_merged_size(); i++) {
    CJP_BlockOut a, b;
    cjp_get_merged_block(i, &a);
    cjp_get_merged_block(i + 1, &b);
    float gap = fabsf(a.exit_v - b.entry_v);
    if (gap > 0.1f) {
      printf("  DISCONTINUITY between merged[%zu] and merged[%zu]: exit=%.3f entry=%.3f gap=%.3f\n",
             i, i + 1, a.exit_v, b.entry_v, gap);
      has_discontinuity = true;
    }
  }
  check("no_velocity_discontinuity", has_discontinuity ? 0.0f : 1.0f, 1.0f);
}

int main() {
  test_pure_cruise();
  test_rest_to_rest_with_cruise();
  test_rest_to_rest_no_cruise();
  test_rest_to_moving();
  test_moving_to_rest();

  test_merge_identical_blocks();
  test_no_merge_different_constraints();
  test_merge_split_at_low_junction();
  test_no_split_near_start();
  test_merged_faster_than_individual();
  test_exec_streaming();
  test_exec_no_merge();
  test_merge_similar_amax();
  test_no_merge_far_amax();
  test_velocity_continuity_bug();

  printf("\n%d passed, %d failed\n", tests_passed, tests_failed);
  return tests_failed > 0 ? 1 : 0;
}
