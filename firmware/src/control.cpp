#include "control.h"
#include "kinematics.h"
#include "pid_speed.h"
#include "encoders.h"
#include "motors.h"

#include <freertos/semphr.h>
#include <math.h>
#include <string.h>

static StaticSemaphore_t g_mutex_storage;
static SemaphoreHandle_t g_mutex;

// Protect state and motor writes. Never hold this lock during serial or network I/O.
class ControlLock {
 public:
  ControlLock() { xSemaphoreTake(g_mutex, portMAX_DELAY); }
  ~ControlLock() { xSemaphoreGive(g_mutex); }
  ControlLock(const ControlLock&) = delete;
  ControlLock& operator=(const ControlLock&) = delete;
};

static ControlMode g_mode = MODE_STOP;
static RunState g_state = STATE_IDLE;
static Pose2D g_pose = {0, 0, 0};
static float g_omega_meas[3] = {0, 0, 0};
static float g_speed = 0.0f;
static float g_cmd_vx = 0, g_cmd_vy = 0, g_cmd_w = 0;
static float g_joy_x = 0, g_joy_y = 0;
static uint32_t g_last_joy_ms = 0;
static uint32_t g_last_cmd_ms = 0;
static Pose2D g_pdes = {0, 0, 0};

static TrajPoint g_traj[TRAJ_MAX_POINTS];
static int g_traj_n = 0;
static int g_traj_i = 0;

static void stop_output() {
  pid_reset();
  motors_stop_all();
}

static void set_idle() {
  g_state = STATE_IDLE;
  g_joy_x = g_joy_y = 0;
  stop_output();
}

static bool manual_expired(uint32_t now) {
  return (g_joy_x != 0 || g_joy_y != 0) && now - g_last_joy_ms >= JOYSTICK_TIMEOUT_MS;
}

void control_init(void) {
  g_mutex = xSemaphoreCreateMutexStatic(&g_mutex_storage);
  set_idle();
}

ControlSnapshot control_get_snapshot(void) {
  ControlLock lock;
  return {g_pose, g_speed, g_mode, g_state, g_traj_i, g_traj_n,
          {g_omega_meas[0], g_omega_meas[1], g_omega_meas[2]}};
}

bool control_set_mode(ControlMode mode) {
  if (mode != MODE_MANUAL && mode != MODE_AUTO && mode != MODE_STOP) return false;
  ControlLock lock;
  set_idle();
  g_mode = mode;
  if (mode == MODE_AUTO) g_traj_i = 0;
  return true;
}

void control_command_stop(void) {
  control_set_mode(MODE_STOP);
}

bool control_command_start(void) {
  ControlLock lock;
  if (g_state != STATE_IDLE) return false;
  if (g_mode == MODE_AUTO && g_traj_n > 0) {
    g_traj_i = 0;
  } else if (g_mode != MODE_MANUAL) {
    return false;
  }
  g_state = STATE_RUNNING;
  return true;
}

bool control_command_pause(void) {
  ControlLock lock;
  if (g_state != STATE_RUNNING || (g_mode != MODE_MANUAL && g_mode != MODE_AUTO)) return false;
  g_state = STATE_PAUSED;
  g_joy_x = g_joy_y = 0;
  stop_output();
  return true;
}

bool control_command_resume(void) {
  ControlLock lock;
  if (g_state != STATE_PAUSED) return false;
  g_state = STATE_RUNNING;
  return true;
}

bool control_set_joystick(float x, float y) {
  if (!isfinite(x) || !isfinite(y) || fabsf(x) > 1 || fabsf(y) > 1) return false;
  ControlLock lock;
  if (g_mode != MODE_MANUAL || g_state != STATE_RUNNING) return false;

  uint32_t now = millis();
  if (manual_expired(now)) {
    set_idle();
    return false;
  }
  if (x == 0 && y == 0 && (g_joy_x != 0 || g_joy_y != 0)) stop_output();
  g_joy_x = x;
  g_joy_y = y;
  g_last_joy_ms = now;
  return true;
}

static void set_serial_mode(ControlMode mode) {
  if (g_mode != mode || g_state != STATE_RUNNING) stop_output();
  g_mode = mode;
  g_joy_x = g_joy_y = 0;
  g_state = STATE_RUNNING;
  g_last_cmd_ms = millis();
}

static void set_velocity(ControlMode mode, float vx, float vy, float w) {
  ControlLock lock;
  set_serial_mode(mode);
  g_cmd_vx = vx;
  g_cmd_vy = vy;
  g_cmd_w = w;
}

void control_set_velocity_global(float vx, float vy, float w) {
  set_velocity(MODE_VELOCITY_GLOBAL, vx, vy, w);
}

void control_set_velocity_body(float vx, float vy, float w) {
  set_velocity(MODE_VELOCITY_BODY, vx, vy, w);
}

void control_set_pose_target(float x, float y, float yaw_rad) {
  ControlLock lock;
  set_serial_mode(MODE_POSE);
  g_pdes = {x, y, wrap_pi(yaw_rad)};
}

TrajectoryResult control_set_trajectory(const TrajPoint* pts, int n) {
  if (pts == nullptr || n <= 0 || n > TRAJ_MAX_POINTS) return TrajectoryResult::Invalid;
  for (int i = 0; i < n; i++) {
    if (!isfinite(pts[i].x) || !isfinite(pts[i].y) ||
        fabsf(pts[i].x) > WORLD_COORD_LIMIT_M || fabsf(pts[i].y) > WORLD_COORD_LIMIT_M) {
      return TrajectoryResult::Invalid;
    }
  }
  ControlLock lock;
  if (g_state != STATE_IDLE) return TrajectoryResult::Busy;
  memcpy(g_traj, pts, sizeof(TrajPoint) * n);
  g_traj_n = n;
  g_traj_i = 0;
  return TrajectoryResult::Ok;
}

int control_get_trajectory(TrajPoint* pts, int max_n) {
  if (pts == nullptr || max_n <= 0) return 0;
  ControlLock lock;
  int n = g_traj_n < max_n ? g_traj_n : max_n;
  memcpy(pts, g_traj, sizeof(TrajPoint) * n);
  return n;
}

static void compute_reference(float omega_ref[3]) {
  uint32_t now = millis();
  switch (g_mode) {
    case MODE_MANUAL:
      if (manual_expired(now)) {
        set_idle();
        return;
      }
      invkin_body(g_joy_x * V_BODY_MAX_M_S, g_joy_y * V_BODY_MAX_M_S, 0, omega_ref);
      break;

    case MODE_VELOCITY_GLOBAL:
    case MODE_VELOCITY_BODY:
    case MODE_POSE: {
      if (now - g_last_cmd_ms >= CMD_WATCHDOG_MS) {
        set_idle();
        return;
      }
      if (g_mode == MODE_VELOCITY_BODY) {
        invkin_body(g_cmd_vx, g_cmd_vy, g_cmd_w, omega_ref);
      } else if (g_mode == MODE_VELOCITY_GLOBAL) {
        invkin(g_cmd_vx, g_cmd_vy, g_cmd_w, g_pose.theta, omega_ref);
      } else {
        float ex = g_pdes.x - g_pose.x;
        float ey = g_pdes.y - g_pose.y;
        float eth = wrap_pi(g_pdes.theta - g_pose.theta);
        if (hypotf(ex, ey) <= POSE_XY_TOL_M && fabsf(eth) <= POSE_YAW_TOL_RAD) {
          set_idle();
          return;
        }
        invkin(LAMBDA_POSE * ex, LAMBDA_POSE * ey, LAMBDA_POSE * eth, g_pose.theta, omega_ref);
      }
      break;
    }

    case MODE_AUTO:
      // Skip any consecutive waypoints already within the position tolerance.
      while (g_traj_i < g_traj_n) {
        float ex = g_traj[g_traj_i].x - g_pose.x;
        float ey = g_traj[g_traj_i].y - g_pose.y;
        if (hypotf(ex, ey) > POSE_XY_TOL_M) {
          invkin(LAMBDA_POSE * ex, LAMBDA_POSE * ey, 0, g_pose.theta, omega_ref);
          return;
        }
        g_traj_i++;
      }
      set_idle();
      break;

    case MODE_STOP:
      break;
  }
}

void control_loop(float dt_s) {
  if (dt_s <= 0) return;
  ControlLock lock;

  float vit_tps[3];
  encoders_sample(vit_tps, g_omega_meas, dt_s);
  float uxd, uyd, uphid;
  fwdkin(g_omega_meas, g_pose.theta, &uxd, &uyd, &uphid);
  g_pose.x += dt_s * uxd;
  g_pose.y += dt_s * uyd;
  g_pose.theta = wrap_pi(g_pose.theta + dt_s * uphid);
  g_speed = hypotf(uxd, uyd);

  if (g_state == STATE_RUNNING) {
    float omega_ref[3] = {0, 0, 0};
    compute_reference(omega_ref);
    if (g_state == STATE_RUNNING) {
      saturate_omega(omega_ref);
      pid_update(omega_ref, vit_tps, dt_s);
    }
  }
}
