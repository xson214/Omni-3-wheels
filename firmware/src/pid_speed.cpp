#include "pid_speed.h"
#include "motors.h"
#include "config.h"

#include <math.h>

static float g_sum_err[3] = {0, 0, 0};
static float g_err_prev[3] = {0, 0, 0};

void pid_reset(void) {
  for (int i = 0; i < 3; i++) {
    g_sum_err[i] = 0.0f;
    g_err_prev[i] = 0.0f;
  }
}

void pid_update(const float omega_ref[3], const float vit_mes_tps[3], float dt_s) {
  // Keep gains referenced to a nominal 10 ms sample.
  float period_ratio = dt_s / (CTRL_PERIOD_MS / 1000.0f);
  for (int i = 0; i < 3; i++) {
    float consigne = omega_ref[i] / (2.0f * PI);
    if (fabsf(consigne) < CONS_EPS_TPS) {
      g_sum_err[i] = 0.0f;
      g_err_prev[i] = 0.0f;
      set_motor(i, 0, 0);
      continue;
    }

    float err = fabsf(consigne) - vit_mes_tps[i];
    float integral_step = err * period_ratio;
    g_sum_err[i] += integral_step;
    float derr = (err - g_err_prev[i]) / period_ratio;
    g_err_prev[i] = err;

    float pwm = PID_KP * err + PID_KI * g_sum_err[i] + PID_KD * derr;
    if (pwm > PWM_MAX || pwm < 0.0f) {
      pwm = constrain(pwm, 0.0f, static_cast<float>(PWM_MAX));
      g_sum_err[i] -= integral_step;
    }

    int dir = (consigne >= 0.0f) ? +1 : -1;
    set_motor(i, static_cast<int>(pwm), dir);
  }
}
