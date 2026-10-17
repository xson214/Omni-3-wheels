#pragma once

// Pin map from README_Xe_3_Banh_Omni_ESP32S3.md §2 only.

// Wheel 1 (alpha = 0)
#define M1_PWM_PIN   4
#define M1_IN1_PIN   5
#define M1_IN2_PIN   6
#define E1_A_PIN     9
#define E1_B_PIN     10

// Wheel 2 (alpha = +120°)
#define M2_PWM_PIN   7
#define M2_IN1_PIN   15
#define M2_IN2_PIN   16
#define E2_A_PIN     11
#define E2_B_PIN     12

// Wheel 3 (alpha = -120°)
#define M3_PWM_PIN   17
#define M3_IN1_PIN   18
#define M3_IN2_PIN   8
#define E3_A_PIN     13
#define E3_B_PIN     14

static const int PWM_PINS[3] = {M1_PWM_PIN, M2_PWM_PIN, M3_PWM_PIN};
static const int IN1_PINS[3] = {M1_IN1_PIN, M2_IN1_PIN, M3_IN1_PIN};
static const int IN2_PINS[3] = {M1_IN2_PIN, M2_IN2_PIN, M3_IN2_PIN};
static const int ENC_A_PINS[3] = {E1_A_PIN, E2_A_PIN, E3_A_PIN};
static const int ENC_B_PINS[3] = {E1_B_PIN, E2_B_PIN, E3_B_PIN};
