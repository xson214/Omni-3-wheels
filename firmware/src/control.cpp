#include "control.h"
#include "kinematics.h"
#include "pid_speed.h"
#include "encoders.h"
#include "motors.h"
#include "config.h"

#include <Arduino.h>
#include <string.h>
#include <math.h>

static ControlMode g_mode = MODE_STOP;
static RunState g_state = STATE_IDLE;

static Pose2D g_pose = {0, 0, 0};
static float g_omega_meas[3] = {0, 0, 0};
static float g_consigne_tps[3] = {0, 0, 0};
static int g_pwm[3] = {0, 0, 0};
static float g_speed = 0.0f;

static float g_cmd_vx = 0, g_cmd_vy = 0, g_cmd_w = 0;
static float g_joy_x = 0, g_joy_y = 0;
static uint32_t g_last_joy_ms = 0;
static uint32_t g_last_cmd_ms = 0;

static Pose2D g_pdes = {0, 0, 0};
static bool g_have_pose_target = false;

static TrajPoint g_traj[TRAJ_MAX_POINTS];
static int g_traj_n = 0;
static int g_traj_i = 0;

void control_init(void) {
  g_mode = MODE_STOP;
  g_state = STATE_IDLE;
  g_pose = {0, 0, 0};
  g_traj_n = 0;
  g_traj_i = 0;
  g_have_pose_target = false;
  pid_init();
  encoders_reset();
  motors_stop_all();
}

ControlMode control_get_mode(void) { return g_mode; }
RunState control_get_state(void) { return g_state; }
Pose2D control_get_pose(void) { return g_pose; }
float control_get_speed(void) { return g_speed; }
int control_traj_count(void) { return g_traj_n; }
int control_traj_index(void) { return g_traj_i; }

void control_get_omega_meas(float w[3]) {
  memcpy(w, g_omega_meas, sizeof(g_omega_meas));
}

void control_get_pwm(int pwm[3]) {
  memcpy(pwm, g_pwm, sizeof(g_pwm));
}

void control_set_mode(ControlMode mode) {
  g_mode = mode;
  if (mode == MODE_STOP) {
    control_command_stop();
  } else if (mode == MODE_MANUAL) {
    g_state = STATE_RUNNING;
    g_last_joy_ms = millis();
  } else if (mode == MODE_AUTO) {
    g_state = STATE_IDLE;
    g_traj_i = 0;
  }
  g_last_cmd_ms = millis();
}

void control_command_stop(void) {
  g_state = STATE_IDLE;
  g_cmd_vx = g_cmd_vy = g_cmd_w = 0;
  g_joy_x = g_joy_y = 0;
  g_have_pose_target = false;
  pid_reset();
  motors_stop_all();
  for (int i = 0; i < 3; i++) {
    g_pwm[i] = 0;
    g_consigne_tps[i] = 0;
  }
}

void control_command_start(void) {
  if (g_mode == MODE_AUTO && g_traj_n > 0) {
    g_traj_i = 0;
    g_state = STATE_RUNNING;
  } else if (g_mode == MODE_MANUAL) {
    g_state = STATE_RUNNING;
  } else if (g_mode == MODE_POSE || g_mode == MODE_VELOCITY_GLOBAL ||
             g_mode == MODE_VELOCITY_BODY) {
    g_state = STATE_RUNNING;
  }
  g_last_cmd_ms = millis();
}

void control_command_pause(void) {
  if (g_state == STATE_RUNNING) {
    g_state = STATE_PAUSED;
    pid_reset();
    motors_stop_all();
  }
}

void control_command_resume(void) {
  if (g_state == STATE_PAUSED) {
    g_state = STATE_RUNNING;
    g_last_cmd_ms = millis();
    g_last_joy_ms = millis();
  }
}

void control_set_joystick(float x, float y) {
  if (x > 1.0f) x = 1.0f;
  if (x < -1.0f) x = -1.0f;
  if (y > 1.0f) y = 1.0f;
  if (y < -1.0f) y = -1.0f;
  g_joy_x = x;
  g_joy_y = y;
  g_last_joy_ms = millis();
  if (g_mode != MODE_MANUAL) {
    g_mode = MODE_MANUAL;
  }
  if (g_state != STATE_PAUSED) {
    g_state = STATE_RUNNING;
  }
}

void control_joystick_tick(void) {
  // kept for API symmetry; timeout checked in loop
}

void control_set_velocity_global(float vx, float vy, float w) {
  g_mode = MODE_VELOCITY_GLOBAL;
  g_cmd_vx = vx;
  g_cmd_vy = vy;
  g_cmd_w = w;
  g_state = STATE_RUNNING;
  g_last_cmd_ms = millis();
}

void control_set_velocity_body(float vx, float vy, float w) {
  g_mode = MODE_VELOCITY_BODY;
  g_cmd_vx = vx;
  g_cmd_vy = vy;
  g_cmd_w = w;
  g_state = STATE_RUNNING;
  g_last_cmd_ms = millis();
}

void control_set_pose_target(float x, float y, float yaw_rad) {
  g_mode = MODE_POSE;
  g_pdes.x = x;
  g_pdes.y = y;
  g_pdes.theta = wrap_pi(yaw_rad);
  g_have_pose_target = true;
  g_state = STATE_RUNNING;
  g_last_cmd_ms = millis();
}

bool control_set_trajectory(const TrajPoint* pts, int n) {
  if (pts == nullptr || n <= 0 || n > TRAJ_MAX_POINTS) return false;
  memcpy(g_traj, pts, sizeof(TrajPoint) * n);
  g_traj_n = n;
  g_traj_i = 0;
  return true;
}

int control_get_trajectory(TrajPoint* pts, int max_n) {
  if (pts == nullptr || max_n <= 0) return 0;
  int n = g_traj_n < max_n ? g_traj_n : max_n;
  memcpy(pts, g_traj, sizeof(TrajPoint) * n);
  return n;
}

static void compute_cmd(float* xd, float* yd, float* phid, bool* use_body) {
  *use_body = false;
  *xd = *yd = *phid = 0.0f;

  if (g_state != STATE_RUNNING) {
    return;
  }

  switch (g_mode) {
    case MODE_MANUAL: {
      if (millis() - g_last_joy_ms > JOYSTICK_TIMEOUT_MS) {
        control_command_stop();
        g_mode = MODE_MANUAL;
        return;
      }
      *xd = g_joy_x * V_BODY_MAX_M_S;
      *yd = g_joy_y * V_BODY_MAX_M_S;
      *phid = 0.0f;
      *use_body = true;
      break;
    }
    case MODE_VELOCITY_GLOBAL:
      *xd = g_cmd_vx;
      *yd = g_cmd_vy;
      *phid = g_cmd_w;
      if (millis() - g_last_cmd_ms > CMD_WATCHDOG_MS) {
        control_command_stop();
      }
      break;
    case MODE_VELOCITY_BODY:
      *xd = g_cmd_vx;
      *yd = g_cmd_vy;
      *phid = g_cmd_w;
      *use_body = true;
      if (millis() - g_last_cmd_ms > CMD_WATCHDOG_MS) {
        control_command_stop();
      }
      break;
    case MODE_POSE: {
      if (!g_have_pose_target) return;
      float ex = g_pdes.x - g_pose.x;
      float ey = g_pdes.y - g_pose.y;
      float eth = wrap_pi(g_pdes.theta - g_pose.theta);
      float nxy = sqrtf(ex * ex + ey * ey);
      if (nxy < POSE_TOL && fabsf(eth) < 0.02f) {
        control_command_stop();
        return;
      }
      *xd = LAMBDA_POSE * ex;
      *yd = LAMBDA_POSE * ey;
      *phid = LAMBDA_POSE * eth;
      g_last_cmd_ms = millis();
      break;
    }
    case MODE_AUTO: {
      if (g_traj_n <= 0 || g_traj_i >= g_traj_n) {
        control_command_stop();
        return;
      }
      float ex = g_traj[g_traj_i].x - g_pose.x;
      float ey = g_traj[g_traj_i].y - g_pose.y;
      float nxy = sqrtf(ex * ex + ey * ey);
      if (nxy < POSE_TOL) {
        g_traj_i++;
        if (g_traj_i >= g_traj_n) {
          control_command_stop();
          return;
        }
        ex = g_traj[g_traj_i].x - g_pose.x;
        ey = g_traj[g_traj_i].y - g_pose.y;
      }
      *xd = LAMBDA_POSE * ex;
      *yd = LAMBDA_POSE * ey;
      *phid = 0.0f;
      break;
    }
    case MODE_STOP:
    default:
      break;
  }
}

void control_loop(void) {
  float xd = 0, yd = 0, phid = 0;
  bool use_body = false;
  compute_cmd(&xd, &yd, &phid, &use_body);

  float omega_ref[3] = {0, 0, 0};
  if (g_state == STATE_RUNNING) {
    if (use_body) {
      invkin_body(xd, yd, phid, omega_ref);
    } else {
      invkin(xd, yd, phid, g_pose.theta, omega_ref);
    }
    saturate_omega(omega_ref);
  }

  for (int i = 0; i < 3; i++) {
    g_consigne_tps[i] = omega_ref[i] / (2.0f * PI);
  }

  float vit_tps[3];
  encoders_sample(vit_tps, g_omega_meas, g_consigne_tps);

  if (g_state == STATE_RUNNING) {
    pid_update(omega_ref, vit_tps, g_pwm);
  } else {
    motors_stop_all();
    for (int i = 0; i < 3; i++) g_pwm[i] = 0;
  }

  float uxd, uyd, uphid;
  fwdkin(g_omega_meas, g_pose.theta, &uxd, &uyd, &uphid);
  g_pose.x += CTRL_DT_S * uxd;
  g_pose.y += CTRL_DT_S * uyd;
  g_pose.theta = wrap_pi(g_pose.theta + CTRL_DT_S * uphid);
  g_speed = sqrtf(uxd * uxd + uyd * uyd);
}
