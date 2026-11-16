#include "pid_speed.h"
#include "motors.h"
#include "config.h"
#include <math.h>

static float integral[3] = {};
static float previous_measurement[3] = {};
static float derivative[3] = {};
static int previous_direction[3] = {};

void pid_reset(void) {
  for (int i = 0; i < 3; ++i) {
    integral[i] = previous_measurement[i] = derivative[i] = 0;
    previous_direction[i] = 0;
  }
}

void pid_update(const float omega_ref[3], const float measured_tps[3], float dt_s) {
  if (!isfinite(dt_s) || dt_s <= 0 || dt_s > CONTROL_MAX_DT_S) {
    pid_reset();
    motors_stop_all();
    return;
  }
  for (int i = 0; i < 3; ++i) {
    float target = omega_ref[i] / (2.0f * PI);
    if (!isfinite(target) || !isfinite(measured_tps[i]) || fabsf(target) < CONS_EPS_TPS) {
      integral[i] = derivative[i] = 0;
      previous_direction[i] = 0;
      set_motor(i, 0, 0);
      continue;
    }
    int direction = target > 0 ? 1 : -1;
    // Preserve encoder sign: rotation opposite to the target must increase error.
    float measured = direction * measured_tps[i];
    if (direction != previous_direction[i]) {
      integral[i] = derivative[i] = 0;
      previous_measurement[i] = measured;
    }
    previous_direction[i] = direction;
    float error = fabsf(target) - measured;
    float raw_derivative = -(measured - previous_measurement[i]) / dt_s;
    derivative[i] += dt_s / (PID_D_FILTER_S + dt_s) * (raw_derivative - derivative[i]);
    previous_measurement[i] = measured;
    float candidate = constrain(integral[i] + PID_KI * error * dt_s, -float(PWM_MAX), float(PWM_MAX));
    float base = PID_KV * fabsf(target) + PID_KP * error + PID_KD * derivative[i];
    float output = base + candidate;
    // Conditional integration: retain steps that drive a saturated output inward.
    if ((output <= PWM_MAX || error < 0) && (output >= 0 || error > 0)) integral[i] = candidate;
    output = constrain(base + integral[i], 0.0f, float(PWM_MAX));
    set_motor(i, static_cast<int>(lroundf(output)), direction);
  }
}
