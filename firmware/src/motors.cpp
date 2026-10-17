#include "motors.h"
#include "pins.h"
#include "config.h"

#include <Arduino.h>

static const int LEDC_CH[3] = {0, 1, 2};
static const int LEDC_FREQ = 20000;
static const int LEDC_RES = 8;

void motors_init(void) {
  for (int i = 0; i < 3; i++) {
    pinMode(IN1_PINS[i], OUTPUT);
    pinMode(IN2_PINS[i], OUTPUT);
    digitalWrite(IN1_PINS[i], LOW);
    digitalWrite(IN2_PINS[i], LOW);
    ledcSetup(LEDC_CH[i], LEDC_FREQ, LEDC_RES);
    ledcAttachPin(PWM_PINS[i], LEDC_CH[i]);
    ledcWrite(LEDC_CH[i], 0);
  }
}

void set_motor(int idx, int pwm, int dir) {
  if (idx < 0 || idx > 2) return;
  if (pwm < 0) pwm = 0;
  if (pwm > PWM_MAX) pwm = PWM_MAX;

  if (dir > 0) {
    digitalWrite(IN1_PINS[idx], HIGH);
    digitalWrite(IN2_PINS[idx], LOW);
  } else if (dir < 0) {
    digitalWrite(IN1_PINS[idx], LOW);
    digitalWrite(IN2_PINS[idx], HIGH);
  } else {
    digitalWrite(IN1_PINS[idx], LOW);
    digitalWrite(IN2_PINS[idx], LOW);
    pwm = 0;
  }
  ledcWrite(LEDC_CH[idx], pwm);
}

void motors_stop_all(void) {
  for (int i = 0; i < 3; i++) {
    set_motor(i, 0, 0);
  }
}
