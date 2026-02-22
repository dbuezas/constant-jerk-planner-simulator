#pragma once

#ifdef __cplusplus
extern "C" {
#endif

int cjp_plan_single_block(float mm, float max_entry_speed, float nominal, float a_max, float j_max);
int cjp_get_first_block(float *out7);

void cjp_traj_reset(void);
int cjp_traj_plan(float entry_v, float exit_v,
                  float a_max, float j_max, float mm, float nominal);
float cjp_traj_duration(void);
float cjp_traj_position(float t);
float cjp_traj_velocity(float t);
float cjp_traj_acceleration(float t);
float cjp_traj_jerk(float t);
float cjp_traj_plan_time_us(void);
const char *cjp_traj_status(void);

int cjp_plan_block_trajectory(int index);
int cjp_get_block_data(int index, float *out7);

#ifdef __cplusplus
}
#endif
