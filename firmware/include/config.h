#pragma once

#include <math.h>
#include <Arduino.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define WHEEL_RADIUS_M     0.029f
#define BASE_RADIUS_M      0.12f
#define MOTOR_RPM_MAX      50.0f
#define OMEGA_MAX_RAD_S    5.236f
#define V_WHEEL_MAX_M_S    0.15184f
#define V_BODY_MAX_M_S     0.12f

#define ALPHA1_RAD         0.0f
#define ALPHA2_RAD         (2.0f * PI / 3.0f)
#define ALPHA3_RAD         (-2.0f * PI / 3.0f)

#define PID_KP             150.0f
#define PID_KI             0.001f
#define PID_KD             0.0001f
#define PWM_MAX            255

#define LAMBDA_POSE        0.6f
#define CTRL_DT_S          0.01f
#define POSE_TOL           0.001f
#define CONS_EPS_TPS       0.01f

#define ENCODER_TICKS_PER_MOTOR_REV   32
#define GEAR_RATIO                    131
#define TICKS_PER_WHEEL_REV           (ENCODER_TICKS_PER_MOTOR_REV * GEAR_RATIO)

#define SERIAL_BAUD                   115200
#define CMD_WATCHDOG_MS               200
#define JOYSTICK_TIMEOUT_MS           300
#define TELEMETRY_PERIOD_MS           50

#define TRAJ_MAX_POINTS               64
