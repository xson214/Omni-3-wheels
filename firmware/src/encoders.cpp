#include "encoders.h"
#include "pins.h"
#include "config.h"

#include <Arduino.h>

static portMUX_TYPE g_ticks_mux = portMUX_INITIALIZER_UNLOCKED;
static int64_t g_ticks[3] = {0, 0, 0};
static int64_t g_previous[3] = {0, 0, 0};
static float g_cpr[3] = {TICKS_PER_WHEEL_REV, TICKS_PER_WHEEL_REV, TICKS_PER_WHEEL_REV};

// Called under the control mutex, shared with encoders_sample.
void encoders_set_cpr(const float cpr[3]) {
  for (int i = 0; i < 3; ++i) g_cpr[i] = cpr[i];
  encoders_get_raw_counts(g_previous);
}

static void IRAM_ATTR enc_isr_generic(int idx) {
  int step = digitalRead(ENC_B_PINS[idx]) ? 1 : -1;
  portENTER_CRITICAL_ISR(&g_ticks_mux);
  g_ticks[idx] += step;
  portEXIT_CRITICAL_ISR(&g_ticks_mux);
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
}

void encoders_get_raw_counts(int64_t counts[3]) {
  portENTER_CRITICAL(&g_ticks_mux);
  for (int i = 0; i < 3; i++) counts[i] = g_ticks[i];
  portEXIT_CRITICAL(&g_ticks_mux);
}

void encoders_sample(float vit_tps[3], float omega_rad_s[3], float dt_s) {
  int64_t counts[3];
  encoders_get_raw_counts(counts);
  for (int i = 0; i < 3; i++) {
    float ticks = static_cast<float>(counts[i] - g_previous[i]);
    g_previous[i] = counts[i];
    float signed_tps = ENC_SIGN[i] * ticks / (dt_s * g_cpr[i]);
    vit_tps[i] = signed_tps;
    omega_rad_s[i] = signed_tps * (2.0f * PI);
  }
}
