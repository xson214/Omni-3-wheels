#include "encoders.h"
#include "pins.h"
#include "config.h"

#include <Arduino.h>

static volatile long g_ticks[3] = {0, 0, 0};

static void IRAM_ATTR enc_isr_generic(int idx) {
  int b = digitalRead(ENC_B_PINS[idx]);
  if (b) {
    g_ticks[idx]++;
  } else {
    g_ticks[idx]--;
  }
}

static void IRAM_ATTR enc1_isr() { enc_isr_generic(0); }
static void IRAM_ATTR enc2_isr() { enc_isr_generic(1); }
static void IRAM_ATTR enc3_isr() { enc_isr_generic(2); }

void encoders_init(void) {
  for (int i = 0; i < 3; i++) {
    pinMode(ENC_A_PINS[i], INPUT_PULLUP);
    pinMode(ENC_B_PINS[i], INPUT_PULLUP);
  }
  attachInterrupt(digitalPinToInterrupt(ENC_A_PINS[0]), enc1_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PINS[1]), enc2_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PINS[2]), enc3_isr, RISING);
  encoders_reset();
}

void encoders_reset(void) {
  noInterrupts();
  g_ticks[0] = g_ticks[1] = g_ticks[2] = 0;
  interrupts();
}

void encoders_sample(float vit_tps[3], float omega_rad_s[3], const float consigne_tps[3]) {
  long t[3];
  noInterrupts();
  for (int i = 0; i < 3; i++) {
    t[i] = g_ticks[i];
    g_ticks[i] = 0;
  }
  interrupts();

  for (int i = 0; i < 3; i++) {
    float vit = ((float)t[i] / CTRL_DT_S) / (float)TICKS_PER_WHEEL_REV;
    // Signed from encoder B; if zero ticks, fall back to consigne sign for odometry.
    float signed_vit = vit;
    if (fabsf(signed_vit) < 1e-6f && fabsf(consigne_tps[i]) >= CONS_EPS_TPS) {
      signed_vit = (consigne_tps[i] >= 0.0f ? 1.0f : -1.0f) * fabsf(vit);
    }
    vit_tps[i] = fabsf(vit);
    omega_rad_s[i] = signed_vit * 2.0f * PI;
  }
}
