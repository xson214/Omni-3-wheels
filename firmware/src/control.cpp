#include "control.h"
#include "kinematics.h"
#include "pid_speed.h"
#include "encoders.h"
#include "motors.h"
#include "imu.h"
#include "tracking.h"

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
static float g_vx = 0, g_vy = 0, g_wz = 0, g_distance = 0;
static ImuSample g_imu = {};
static bool g_imu_recalibrate = false, g_imu_busy = false, g_skip_after_calibration = false;
static bool g_encoder_calibrated = ENCODER_CALIBRATED_DEFAULT;
static float g_cpr[3] = {TICKS_PER_WHEEL_REV, TICKS_PER_WHEEL_REV, TICKS_PER_WHEEL_REV};
static const char* g_fault = "";
static TrackingState g_tracking;
static float g_auto_yaw = 0;
static float g_target_error = 0;
static float g_cmd_vx = 0, g_cmd_vy = 0, g_cmd_w = 0;
static float g_joy_x = 0, g_joy_y = 0;
static uint32_t g_last_joy_ms = 0;
static uint32_t g_last_cmd_ms = 0;
static Pose2D g_pdes = {0, 0, 0};
static int g_test_wheel = 0, g_test_pwm = 0;
static uint32_t g_test_started_ms = 0, g_test_duration_ms = 0;

static TrajPoint g_traj[TRAJ_MAX_POINTS];
static int g_traj_n = 0;
static int g_traj_i = 0;

static void stop_output() {
  pid_reset();
  g_tracking.integral_x = g_tracking.integral_y = g_tracking.integral_yaw = 0;
  g_tracking.settled_s = 0;
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

static bool ready_for_motion(bool require_imu) {
  if (g_imu_busy) { g_fault = "IMU calibration in progress: keep stationary"; return false; }
  if (!g_encoder_calibrated) { g_fault = "Confirm measured encoder CPR before motion"; return false; }
  if (require_imu && (!g_imu.healthy || !g_imu.calibrated)) {
    g_fault = "IMU unavailable or uncalibrated: reboot while stationary";
    return false;
  }
  g_fault = "";
  return true;
}

void control_init(void) {
  g_mutex = xSemaphoreCreateMutexStatic(&g_mutex_storage);
  set_idle();
}

ControlSnapshot control_get_snapshot(void) {
  ControlLock lock;
  return {g_pose, g_speed, g_mode, g_state, g_traj_i, g_traj_n,
          {g_omega_meas[0], g_omega_meas[1], g_omega_meas[2]},
          g_distance, g_imu.healthy, g_imu.calibrated, g_encoder_calibrated,
          g_imu.gyro_z_rad_s, g_target_error, {g_cpr[0], g_cpr[1], g_cpr[2]}, g_fault};
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
  if (!ready_for_motion(g_mode == MODE_AUTO)) return false;
  if (g_mode == MODE_AUTO && g_traj_n > 0) {
    g_traj_i = 0;
    g_auto_yaw = g_pose.theta;
    tracking_begin(g_tracking, g_pose.x, g_pose.y, g_traj[0].x, g_traj[0].y, g_auto_yaw);
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
  if (!ready_for_motion(g_mode == MODE_AUTO)) return false;
  g_tracking.reference_speed = 0;
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

static bool set_velocity(ControlMode mode, float vx, float vy, float w) {
  if (!isfinite(vx) || !isfinite(vy) || !isfinite(w) || fabsf(vx) > SERIAL_LINEAR_LIMIT_M_S ||
      fabsf(vy) > SERIAL_LINEAR_LIMIT_M_S || fabsf(w) > SERIAL_ANGULAR_LIMIT_RAD_S) return false;
  ControlLock lock;
  if (!ready_for_motion(false)) return false;
  set_serial_mode(mode);
  g_cmd_vx = vx;
  g_cmd_vy = vy;
  g_cmd_w = w;
  return true;
}

bool control_set_velocity_global(float vx, float vy, float w) {
  return set_velocity(MODE_VELOCITY_GLOBAL, vx, vy, w);
}

bool control_set_velocity_body(float vx, float vy, float w) {
  return set_velocity(MODE_VELOCITY_BODY, vx, vy, w);
}

bool control_set_pose_target(float x, float y, float yaw_rad) {
  if (!isfinite(x) || !isfinite(y) || !isfinite(yaw_rad) || fabsf(x) > WORLD_COORD_LIMIT_M ||
      fabsf(y) > WORLD_COORD_LIMIT_M) return false;
  ControlLock lock;
  if (!ready_for_motion(true)) return false;
  float yaw = wrap_pi(yaw_rad);
  bool changed = g_mode != MODE_POSE || g_state != STATE_RUNNING ||
      x != g_pdes.x || y != g_pdes.y || yaw != g_pdes.theta;
  set_serial_mode(MODE_POSE);
  g_pdes = {x, y, yaw};
  if (changed) tracking_begin(g_tracking, g_pose.x, g_pose.y, x, y, yaw);
  return true;
}

bool control_reset_odometry(void) {
  ControlLock lock;
  if (g_state != STATE_IDLE || g_speed > SETTLE_SPEED_M_S || fabsf(g_wz) > SETTLE_YAW_RAD_S) return false;
  encoders_set_cpr(g_cpr); // Discard counts since last sample, keep raw counters for calibration.
  g_pose = {0, 0, 0};
  g_distance = g_target_error = 0;
  g_fault = "";
  return true;
}

bool control_set_encoder_cpr(float a, float b, float c) {
  float values[3] = {a, b, c};
  for (float value : values) if (!isfinite(value) || value < 1 || value > 1000000) return false;
  ControlLock lock;
  if (g_state != STATE_IDLE || g_speed > SETTLE_SPEED_M_S || fabsf(g_wz) > SETTLE_YAW_RAD_S) return false;
  memcpy(g_cpr, values, sizeof(g_cpr));
  encoders_set_cpr(g_cpr);
  g_encoder_calibrated = true;
  g_pose = {0, 0, 0};
  g_distance = g_target_error = 0;
  g_fault = "";
  return true;
}

bool control_test_motor(int wheel, int signed_pwm, uint32_t duration_ms) {
  if (wheel < 0 || wheel >= 3 || signed_pwm == 0 || signed_pwm < -MOTOR_TEST_PWM_MAX ||
      signed_pwm > MOTOR_TEST_PWM_MAX || duration_ms < 50 || duration_ms > MOTOR_TEST_DURATION_MAX_MS) return false;
  ControlLock lock;
  if (g_imu_busy || g_state != STATE_IDLE || g_mode != MODE_STOP) return false;
  stop_output();
  g_test_wheel = wheel;
  g_test_pwm = signed_pwm;
  g_test_started_ms = g_last_cmd_ms = millis();
  g_test_duration_ms = duration_ms;
  g_fault = "";
  g_mode = MODE_MOTOR_TEST;
  g_state = STATE_RUNNING;
  return true;
}

bool control_test_heartbeat(void) {
  ControlLock lock;
  if (g_mode != MODE_MOTOR_TEST || g_state != STATE_RUNNING) return false;
  if (millis() - g_last_cmd_ms >= CMD_WATCHDOG_MS) {
    g_fault = "Motor test heartbeat timeout";
    set_idle(); g_mode = MODE_STOP; return false;
  }
  g_last_cmd_ms = millis();
  return true;
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

static void compute_reference(float omega_ref[3], float dt) {
  uint32_t now = millis();
  if (g_mode == MODE_MANUAL) {
    if (manual_expired(now)) { g_fault = "Joystick timeout"; set_idle(); return; }
    invkin_body(g_joy_x * V_BODY_MAX_M_S, g_joy_y * V_BODY_MAX_M_S, 0, omega_ref);
    return;
  }
  if (g_mode == MODE_VELOCITY_GLOBAL || g_mode == MODE_VELOCITY_BODY || g_mode == MODE_POSE) {
    if (now - g_last_cmd_ms >= CMD_WATCHDOG_MS) { g_fault = "Serial command timeout"; set_idle(); return; }
    if (g_mode == MODE_VELOCITY_BODY) { invkin_body(g_cmd_vx, g_cmd_vy, g_cmd_w, omega_ref); return; }
    if (g_mode == MODE_VELOCITY_GLOBAL) { invkin(g_cmd_vx, g_cmd_vy, g_cmd_w, g_pose.theta, omega_ref); return; }
  }
  if (g_mode != MODE_AUTO && g_mode != MODE_POSE) return;
  float command[3];
  bool reached = tracking_step(g_tracking, g_pose.x, g_pose.y, g_pose.theta,
                               g_vx, g_vy, g_wz, dt, command);
  g_target_error = hypotf(g_tracking.target_x - g_pose.x, g_tracking.target_y - g_pose.y);
  if (tracking_timed_out(g_tracking)) {
    g_fault = "Tracking timeout: check wheels, signs and calibration";
    set_idle(); return;
  }
  if (reached) {
    if (g_mode == MODE_POSE || ++g_traj_i >= g_traj_n) { set_idle(); return; }
    // Next segment starts at the requested vertex, avoiding accumulated corner drift.
    tracking_begin(g_tracking, g_traj[g_traj_i - 1].x, g_traj[g_traj_i - 1].y,
                   g_traj[g_traj_i].x, g_traj[g_traj_i].y, g_auto_yaw);
    pid_reset();
    return;
  }
  invkin(command[0], command[1], command[2], g_pose.theta, omega_ref);
}

bool control_recalibrate_imu(void) {
  ControlLock lock;
  if (g_imu_busy || g_state != STATE_IDLE || g_speed > SETTLE_SPEED_M_S) return false;
  set_idle(); g_mode = MODE_STOP;
  g_imu_busy = g_imu_recalibrate = true;
  g_imu = {};
  g_fault = "IMU calibration in progress: keep stationary";
  return true;
}

void control_loop(float dt_s) {
  if (!isfinite(dt_s) || dt_s <= 0) return;
  bool recalibrate;
  {
    ControlLock lock;
    recalibrate = g_imu_recalibrate;
    g_imu_recalibrate = false;
  }
  if (recalibrate) {
    bool ok = imu_init(); // Motors stopped; motion requests blocked while busy.
    ControlLock lock;
    g_imu_busy = false;
    g_skip_after_calibration = true;
    g_fault = ok ? "" : "IMU calibration failed: check mounting and keep still";
    return;
  }
  if (g_skip_after_calibration) {
    ControlLock lock;
    g_skip_after_calibration = false;
    encoders_set_cpr(g_cpr);
    return; // Discard the long calibration interval; preserve the pose origin.
  }
  // Only this task accesses the I2C driver. Do not block the command mutex on I2C.
  ImuSample sample = imu_read();
  ControlLock lock;
  g_imu = sample;
  float vit_tps[3];
  encoders_sample(vit_tps, g_omega_meas, dt_s);
  float unused_x, unused_y, encoder_wz;
  fwdkin(g_omega_meas, 0, &unused_x, &unused_y, &encoder_wz);
  bool usable_imu = sample.healthy && sample.calibrated;
  g_wz = usable_imu ? IMU_GYRO_WEIGHT * sample.gyro_z_rad_s + (1 - IMU_GYRO_WEIGHT) * encoder_wz : encoder_wz;
  // Midpoint heading improves integration while translating and turning together.
  float mid_yaw = wrap_pi(g_pose.theta + 0.5f * dt_s * g_wz);
  fwdkin(g_omega_meas, mid_yaw, &g_vx, &g_vy, &unused_x);
  g_pose.x += dt_s * g_vx;
  g_pose.y += dt_s * g_vy;
  g_pose.theta = wrap_pi(g_pose.theta + dt_s * g_wz);
  g_speed = hypotf(g_vx, g_vy);
  g_distance += g_speed * dt_s;
  if (dt_s > CONTROL_MAX_DT_S) { g_fault = "Control loop overrun"; set_idle(); return; }
  if (g_state == STATE_RUNNING) {
    if (g_mode == MODE_MOTOR_TEST) {
      uint32_t now = millis();
      if (now - g_test_started_ms >= g_test_duration_ms || now - g_last_cmd_ms >= CMD_WATCHDOG_MS) {
        if (now - g_last_cmd_ms >= CMD_WATCHDOG_MS) g_fault = "Motor test heartbeat timeout";
        set_idle(); g_mode = MODE_STOP;
      } else {
        set_motor(g_test_wheel, abs(g_test_pwm), g_test_pwm > 0 ? 1 : -1);
      }
      return;
    }
    if ((g_mode == MODE_AUTO || g_mode == MODE_POSE) && !usable_imu) {
      g_fault = "IMU sample lost: stopped trajectory"; set_idle(); return;
    }
    float omega_ref[3] = {0, 0, 0};
    compute_reference(omega_ref, dt_s);
    if (g_state == STATE_RUNNING) {
      saturate_omega(omega_ref);
      pid_update(omega_ref, vit_tps, dt_s);
    }
  }
}
