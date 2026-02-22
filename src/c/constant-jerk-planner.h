#pragma once

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Constant-Jerk Planner
//
// Multi-block motion planner using reverse/forward passes.
// All junctions have a=0. The planner only selects junction velocities.
// The trajectory generator handles S-curves within each block.

#ifndef CJP_BLOCK_BUFFER_SIZE
#define CJP_BLOCK_BUFFER_SIZE 32
#endif

typedef struct CJP_BlockOut {
  float millimeters;
  float max_entry_speed;
  float nominal;
  float a_max;
  float j_max;

  float entry_v;
  float exit_v;
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

// Number of merged blocks after recalculate.
size_t cjp_merged_size(void);

// Returns 1 on success, 0 if index out of range.
int cjp_get_merged_block(size_t index, CJP_BlockOut *out);

// --- Streaming execution API ---

// Prepare for stepping through the planned trajectory. Call after cjp_recalculate.
void cjp_exec_reset(void);

// Advance by dt seconds. Returns 1 if more data, 0 if done.
int cjp_exec_step(float dt);

// Current state getters (valid after cjp_exec_step returns 1).
float cjp_exec_time(void);
float cjp_exec_position(void);
float cjp_exec_velocity(void);
float cjp_exec_acceleration(void);
float cjp_exec_jerk(void);
size_t cjp_exec_original_block(void);
size_t cjp_exec_merged_block(void);

#ifdef __cplusplus
}
#endif
