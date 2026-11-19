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
#define PID_KI             100.0f
#define PID_KD             0.02f
#define PID_KV             (255.0f / (MOTOR_RPM_MAX / 60.0f))
#define PID_D_FILTER_S     0.04f
#define PWM_MAX            255

#define LAMBDA_POSE        0.6f
#define CTRL_PERIOD_MS     10
#define POSE_YAW_TOL_RAD   0.06f
#define POSE_XY_TOL_M      0.01f
#define CONS_EPS_TPS       0.01f

// Starting gains only: tune on the physical robot. PID uses seconds.
#define POSE_KP_XY         0.8f
#define POSE_KI_XY         0.08f
#define POSE_KD_XY         0.10f
#define POSE_KP_YAW        1.5f
#define POSE_KI_YAW        0.05f
#define POSE_KD_YAW        0.10f
#define POSE_I_LIMIT_M_S   0.025f
#define POSE_I_LIMIT_RAD_S 0.10f
#define YAW_RATE_MAX_RAD_S 0.5f
#define TRAJ_SPEED_M_S     0.06f
#define TRAJ_ACCEL_M_S2    0.08f
#define TRAJ_LEAD_MAX_M    0.05f
#define POSE_SETTLE_S      0.30f
#define SETTLE_SPEED_M_S   0.008f
#define SETTLE_YAW_RAD_S   0.04f
#define CONTROL_MAX_DT_S   0.10f
#define IMU_GYRO_WEIGHT    0.98f

// Seller specification supplied by user: 26 A-channel cycles/motor rev x 78:1.
// A-rising (1x) decoding => 2028 counts/output-wheel rev. Verify on hardware.
#define TICKS_PER_WHEEL_REV           (26 * 78)
// Bench verified: positive motor commands produce negative raw A-rising counts.
static constexpr int ENC_SIGN[3] = {-1, -1, -1};
static constexpr int MOTOR_SIGN[3] = {1, 1, 1};
// Accept the supplied nominal CPR. This does not certify physical calibration.
#ifndef ENCODER_CALIBRATED_DEFAULT
#define ENCODER_CALIBRATED_DEFAULT true
#endif

#define SERIAL_BAUD                   115200
#define CMD_WATCHDOG_MS               200
#define JOYSTICK_TIMEOUT_MS           500
#define IMU_GAP_MAX_S                 0.05f
#define TELEMETRY_PERIOD_MS           50
#define MOTOR_TEST_PWM_MAX            160
#define MOTOR_TEST_DURATION_MAX_MS    1500

#define TRAJ_MAX_POINTS               64
#define WORLD_COORD_LIMIT_M           1000.0f
#define SERIAL_LINEAR_LIMIT_M_S       1.0f
#define SERIAL_ANGULAR_LIMIT_RAD_S    10.0f
#define SERIAL_YAW_LIMIT_DEG          36000.0f
