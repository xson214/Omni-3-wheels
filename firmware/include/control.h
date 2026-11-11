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

struct ControlSnapshot {
  Pose2D pose;
  float speed;
  ControlMode mode;
  RunState state;
  int traj_index;
  int traj_count;
  float omega_meas[3];
};

enum class TrajectoryResult { Ok, Invalid, Busy };

void control_init(void);
void control_loop(float dt_s);

bool control_set_mode(ControlMode mode);
void control_command_stop(void);
bool control_command_start(void);
bool control_command_pause(void);
bool control_command_resume(void);

bool control_set_joystick(float x, float y);
void control_set_velocity_global(float vx, float vy, float w);
void control_set_velocity_body(float vx, float vy, float w);
void control_set_pose_target(float x, float y, float yaw_rad);

TrajectoryResult control_set_trajectory(const TrajPoint* pts, int n);
int control_get_trajectory(TrajPoint* pts, int max_n);
ControlSnapshot control_get_snapshot(void);
