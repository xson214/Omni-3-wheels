#include "tracking.h"
#include "config.h"
#include "kinematics.h"
#include <math.h>

void tracking_begin(TrackingState& s, float x, float y, float tx, float ty, float yaw) {
  s = TrackingState{};
  s.start_x = x; s.start_y = y;
  s.target_x = tx; s.target_y = ty; s.target_yaw = yaw;
  s.length = hypotf(tx - x, ty - y);
}

bool tracking_timed_out(const TrackingState& s) {
  return s.elapsed_s > 15.0f + 3.0f * s.length / TRAJ_SPEED_M_S;
}

bool tracking_step(TrackingState& s, float x, float y, float yaw,
                   float vx, float vy, float wz, float dt, float output[3]) {
  s.elapsed_s += dt;
  float ux = s.length > 1e-6f ? (s.target_x - s.start_x) / s.length : 0;
  float uy = s.length > 1e-6f ? (s.target_y - s.start_y) / s.length : 0;
  float rx = s.start_x + ux * s.progress;
  float ry = s.start_y + uy * s.progress;
  float remaining = fmaxf(0, s.length - s.progress);
  float desired_speed = fminf(TRAJ_SPEED_M_S, sqrtf(2 * TRAJ_ACCEL_M_S2 * remaining));
  // Hold the reference if the physical robot is lagging behind it.
  if (hypotf(rx - x, ry - y) > TRAJ_LEAD_MAX_M) desired_speed = 0;
  float dv = TRAJ_ACCEL_M_S2 * dt;
  s.reference_speed += constrain(desired_speed - s.reference_speed, -dv, dv);
  float advance = fminf(remaining, s.reference_speed * dt);
  s.progress += advance;
  if (s.progress >= s.length) s.reference_speed = 0;
  rx = s.start_x + ux * s.progress;
  ry = s.start_y + uy * s.progress;
  float fx = ux * s.reference_speed, fy = uy * s.reference_speed;
  float ex = rx - x, ey = ry - y, eth = wrap_pi(s.target_yaw - yaw);
  float ix = constrain(s.integral_x + POSE_KI_XY * ex * dt, -POSE_I_LIMIT_M_S, POSE_I_LIMIT_M_S);
  float iy = constrain(s.integral_y + POSE_KI_XY * ey * dt, -POSE_I_LIMIT_M_S, POSE_I_LIMIT_M_S);
  float it = constrain(s.integral_yaw + POSE_KI_YAW * eth * dt, -POSE_I_LIMIT_RAD_S, POSE_I_LIMIT_RAD_S);
  output[0] = fx + POSE_KP_XY * ex + ix + POSE_KD_XY * (fx - vx);
  output[1] = fy + POSE_KP_XY * ey + iy + POSE_KD_XY * (fy - vy);
  output[2] = POSE_KP_YAW * eth + it - POSE_KD_YAW * wz;
  float scale = fmaxf(1, hypotf(output[0], output[1]) / V_BODY_MAX_M_S);
  scale = fmaxf(scale, fabsf(output[2]) / YAW_RATE_MAX_RAD_S);
  float wheels[3];
  invkin(output[0], output[1], output[2], yaw, wheels);
  for (int i = 0; i < 3; ++i) scale = fmaxf(scale, fabsf(wheels[i]) / OMEGA_MAX_RAD_S);
  if (scale <= 1.0f) {
    s.integral_x = ix; s.integral_y = iy; s.integral_yaw = it;
  }
  for (int i = 0; i < 3; ++i) output[i] /= scale;
  bool in_tolerance = s.progress >= s.length &&
      hypotf(s.target_x - x, s.target_y - y) <= POSE_XY_TOL_M && fabsf(eth) <= POSE_YAW_TOL_RAD;
  if (in_tolerance) {
    // Coast to rest inside tolerance rather than retaining integral creep.
    output[0] = output[1] = output[2] = 0;
    s.integral_x = s.integral_y = s.integral_yaw = 0;
  }
  bool stopped = hypotf(vx, vy) <= SETTLE_SPEED_M_S && fabsf(wz) <= SETTLE_YAW_RAD_S;
  s.settled_s = in_tolerance && stopped ? s.settled_s + dt : 0;
  return s.settled_s >= POSE_SETTLE_S;
}
