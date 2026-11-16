#pragma once

// ESP32-S3 + 3x L298N + 3x GM25 encoders + MPU9250.
// Keep in sync with README_Xe_3_Banh_Omni_ESP32S3.md section 2.

// Wheel 1 (alpha = 0)
#define M1_PWM_PIN   1
#define M1_IN1_PIN   2
#define M1_IN2_PIN   3
#define E1_A_PIN     4
#define E1_B_PIN     5

// Wheel 2 (alpha = +120°)
#define M2_PWM_PIN   10
#define M2_IN1_PIN   11
#define M2_IN2_PIN   12
#define E2_A_PIN     6
#define E2_B_PIN     7

// Wheel 3 (alpha = -120°)
#define M3_PWM_PIN   13
#define M3_IN1_PIN   14
#define M3_IN2_PIN   21
#define E3_A_PIN     17
#define E3_B_PIN     18

// MPU9250 - I2C (pin/address definitions only; no sensor driver yet).
#define I2C_SDA      8
#define I2C_SCL      9
#define MPU9250_ADDR 0x68

static const int PWM_PINS[3] = {M1_PWM_PIN, M2_PWM_PIN, M3_PWM_PIN};
static const int IN1_PINS[3] = {M1_IN1_PIN, M2_IN1_PIN, M3_IN1_PIN};
static const int IN2_PINS[3] = {M1_IN2_PIN, M2_IN2_PIN, M3_IN2_PIN};
static const int ENC_A_PINS[3] = {E1_A_PIN, E2_A_PIN, E3_A_PIN};
static const int ENC_B_PINS[3] = {E1_B_PIN, E2_B_PIN, E3_B_PIN};
