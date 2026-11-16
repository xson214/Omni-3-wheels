#pragma once

// Mount sensor Z near parallel to robot yaw Z (up OR down).
// 0: infer up/down from stationary acceleration during calibration.
// +1 / -1: explicit gyro sign override. Accelerations stay in SENSOR axes.
#ifndef IMU_GYRO_Z_SIGN
#define IMU_GYRO_Z_SIGN 0
#endif
static_assert(IMU_GYRO_Z_SIGN >= -1 && IMU_GYRO_Z_SIGN <= 1, "IMU_GYRO_Z_SIGN must be 0 (auto), +1 or -1");

struct ImuSample {
  bool healthy;
  bool calibrated;
  float gyro_z_rad_s;
  float accel_x_m_s2;
  float accel_y_m_s2;
  float accel_z_m_s2;
};

// Call with motors stopped, before creating the control task. Keep the robot
// still and level for ~3.3 s. Returns true only after configuration, suitable
// mounting orientation AND bias calibration. A sideways/tilted mount fails.
// A false result permits encoder-only operation; reboot while still to retry.
bool imu_init(void);

// Effective gyro sign: +1/-1 after calibration, 0 if calibration failed.
int imu_yaw_sign(void);
const char* imu_init_status(void);
unsigned int imu_identity(void);

// Single caller (control task). No blocking retry/delay; I2C timeout is 3 ms
// per transaction. A failed/stale/saturated sample is unhealthy with zero data.
// calibrated means startup bias was accepted, not absolute heading calibration.
// Uses gyro + accelerometer only; the magnetometer is not initialized.
ImuSample imu_read(void);
