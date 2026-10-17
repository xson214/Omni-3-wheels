#pragma once

#include "config.h"

enum ControlMode {
  MODE_STOP = 0,
  MODE_MANUAL = 1,
  MODE_AUTO = 2,
  MODE_VELOCITY_GLOBAL = 3,
  MODE_VELOCITY_BODY = 4,
  MODE_POSE = 5,
};

enum RunState {
  STATE_IDLE = 0,
  STATE_RUNNING = 1,
  STATE_PAUSED = 2,
};

struct Pose2D {
  float x;
  float y;
  float theta;
};

struct TrajPoint {
  float x;
  float y;
};

void control_init(void);
void control_loop(void);

void control_set_mode(ControlMode mode);
ControlMode control_get_mode(void);
RunState control_get_state(void);

void control_command_stop(void);
void control_command_start(void);
void control_command_pause(void);
void control_command_resume(void);

void control_set_joystick(float x, float y);
void control_joystick_tick(void);

void control_set_velocity_global(float vx, float vy, float w);
void control_set_velocity_body(float vx, float vy, float w);
void control_set_pose_target(float x, float y, float yaw_rad);

bool control_set_trajectory(const TrajPoint* pts, int n);
int control_get_trajectory(TrajPoint* pts, int max_n);
int control_traj_count(void);
int control_traj_index(void);

Pose2D control_get_pose(void);
void control_get_omega_meas(float w[3]);
void control_get_pwm(int pwm[3]);
float control_get_speed(void);
