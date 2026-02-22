#pragma once

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Minimal, 1D Marlin-style motion planner.
// Blocks are appended; planner computes per-block entry/exit speeds.
// All speeds are in mm/s. Accelerations are in mm/s^2. Distances are in mm.

#ifndef MINIPLANNER_BLOCK_BUFFER_SIZE
#define MINIPLANNER_BLOCK_BUFFER_SIZE 32
#endif

typedef struct MP_BlockOut {
  float millimeters;
  float nominal_speed;
  float acceleration;

  float max_entry_speed;
  float min_entry_speed;

  float entry_speed;
  float exit_speed;

  float entry_speed_sqr;
  float exit_speed_sqr;
} MP_BlockOut;

void mp_reset(void);

// Returns 1 on success, 0 if buffer is full or invalid args.
int mp_push_block(float millimeters, float nominal_speed, float acceleration,
                  float max_entry_speed, float min_entry_speed);

// Recalculate all entry/exit speeds. safe_exit_speed is for the newest block.
void mp_recalculate(float safe_exit_speed);

// Number of queued blocks.
size_t mp_size(void);

// Returns 1 on success, 0 if index out of range.
int mp_get_block(size_t index, MP_BlockOut *out);

// Drop the oldest block. Returns 1 if a block was dropped.
int mp_pop_front(void);

#ifdef __cplusplus
}
#endif
