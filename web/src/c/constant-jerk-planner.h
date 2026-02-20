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

#ifdef __cplusplus
}
#endif
