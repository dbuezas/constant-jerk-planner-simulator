#!/usr/bin/env bash
source "/Users/davidbuezas/code/emsdk/emsdk_env.sh"

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PROJECT_ROOT="$(cd "${ROOT_DIR}/.." && pwd)"
OUT_DIR="${ROOT_DIR}/src/wasm"
C_ROOT="${ROOT_DIR}/src/c"

EMCC=${EMCC:-emcc}

${EMCC} \
  "${ROOT_DIR}/src/wasm/planner_wrapper.cpp" \
  "${C_ROOT}/constant-jerk-planner.cpp" \
  -I"${C_ROOT}" \
  -I"${C_ROOT}/inc" \
  -I"${C_ROOT}" \
  -I"${C_ROOT}/marlin files" \
  -std=c++17 \
  -O2 \
  -s MODULARIZE=1 \
  -s EXPORT_ES6=1 \
  -s ENVIRONMENT=web \
  -s ALLOW_MEMORY_GROWTH=1 \
  -s EXPORTED_FUNCTIONS='["_cjp_plan_single_block","_cjp_get_first_block","_cjp_traj_reset","_cjp_traj_plan","_cjp_traj_duration","_cjp_traj_position","_cjp_traj_velocity","_cjp_traj_acceleration","_cjp_traj_jerk","_cjp_traj_plan_time_us","_cjp_traj_status","_cjp_reset","_cjp_push_block","_cjp_recalculate","_cjp_size","_cjp_get_block_data","_cjp_plan_block_trajectory","_cjp_merged_size","_cjp_get_merged_block","_cjp_exec_reset","_cjp_exec_step","_cjp_exec_time","_cjp_exec_position","_cjp_exec_velocity","_cjp_exec_acceleration","_cjp_exec_jerk","_cjp_exec_original_block","_cjp_exec_merged_block","_malloc","_free"]' \
  -s EXPORTED_RUNTIME_METHODS='["cwrap","getValue","setValue","HEAPF32"]' \
  -o "${OUT_DIR}/planner_wasm.js"
