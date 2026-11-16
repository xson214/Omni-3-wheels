#include "imu.h"
#include "pins.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// Register definitions: TDK InvenSense RM-MPU-9250A-00 and RM-000008.
// Scale factors: PS-MPU-9250A-01 v1.1, sections 3.1 / 3.2.
// https://web.stanford.edu/class/ee267/misc/MPU-9255-Register-Map.pdf
// https://product.tdk.com/en/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ps-mpu-9250a-01-v1.1.pdf
// MPU6500 uses the same accel/gyro registers here: RM-MPU-6500A-00 rev 2.1.
// https://www.ic-components.sk/files/5b/MPU-6500.pdf
namespace {
constexpr uint8_t WHO_AM_I = 0x75;
constexpr uint8_t PWR_MGMT_1 = 0x6B;
constexpr uint8_t PWR_MGMT_2 = 0x6C;
constexpr uint8_t SAMPLE_RATE_DIV = 0x19;
constexpr uint8_t INT_ENABLE = 0x38;
constexpr uint8_t INT_STATUS = 0x3A;
constexpr float ACCEL_LSB_PER_G = 16384.0f;  // +/- 2 g
constexpr float GYRO_LSB_PER_DPS = 131.0f;   // +/- 250 deg/s
constexpr float GRAVITY = 9.80665f;
constexpr float RAD_PER_DEG = 0.0174532925199433f;
constexpr int CALIBRATION_SAMPLES = 300;
// SMPLRT_DIV=4: 1 kHz / (1 + 4) = 200 Hz, above control-loop rate.
// CONFIG=3: gyro DLPF 41 Hz; GYRO_CONFIG=0: DLPF on, +/-250 deg/s.
// ACCEL_CONFIG=0: +/-2 g; ACCEL_CONFIG2=3: accel DLPF 44.8 Hz.
constexpr uint8_t SENSOR_CONFIG[5] = {4, 3, 0, 0, 3};
bool g_configured = false;
bool g_calibrated = false;
float g_gyro_z_bias_dps = 0;
int g_yaw_sign = 0;
const char* g_init_status = "not initialized";
uint8_t g_identity = 0;

bool read_regs(uint8_t reg, uint8_t* out, size_t count) {
  Wire.beginTransmission(static_cast<uint8_t>(MPU9250_ADDR));
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(static_cast<uint8_t>(MPU9250_ADDR), count, true) != count) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (size_t i = 0; i < count; ++i) out[i] = static_cast<uint8_t>(Wire.read());
  return true;
}

bool write_reg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(static_cast<uint8_t>(MPU9250_ADDR));
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

int16_t signed_word(const uint8_t* bytes) {
  return static_cast<int16_t>((static_cast<uint16_t>(bytes[0]) << 8) | bytes[1]);
}

struct RawSample {
  float accel_g[3];
  float gyro_dps[3];
};

bool read_raw(RawSample& sample) {
  // INT_STATUS plus all 14 sensor bytes in one coherent burst. Require a new
  // sample; never integrate cached gyro data again after an I2C/readiness error.
  uint8_t bytes[15];
  if (!read_regs(INT_STATUS, bytes, sizeof(bytes)) || !(bytes[0] & 0x01)) return false;
  for (int i = 0; i < 3; ++i) {
    int16_t a = signed_word(bytes + 1 + 2 * i);
    int16_t w = signed_word(bytes + 9 + 2 * i);
    if (a >= 32760 || a <= -32760 || w >= 32760 || w <= -32760) return false;
    sample.accel_g[i] = a / ACCEL_LSB_PER_G;
    sample.gyro_dps[i] = w / GYRO_LSB_PER_DPS;
  }
  return true;
}

bool calibrate_bias() {
  float gyro_mean[3] = {}, gyro_m2[3] = {};
  float accel_mean[3] = {}, accel_m2[3] = {};
  for (int n = 1; n <= CALIBRATION_SAMPLES; ++n) {
    delay(10);
    RawSample raw;
    if (!read_raw(raw)) return false;
    float accel_norm_sq = 0, gyro_norm_sq = 0;
    for (int i = 0; i < 3; ++i) {
      accel_norm_sq += raw.accel_g[i] * raw.accel_g[i];
      gyro_norm_sq += raw.gyro_dps[i] * raw.gyro_dps[i];
      const float dg = raw.gyro_dps[i] - gyro_mean[i];
      gyro_mean[i] += dg / n;
      gyro_m2[i] += dg * (raw.gyro_dps[i] - gyro_mean[i]);
      const float da = raw.accel_g[i] - accel_mean[i];
      accel_mean[i] += da / n;
      accel_m2[i] += da * (raw.accel_g[i] - accel_mean[i]);
    }
    if (accel_norm_sq < 0.85f * 0.85f || accel_norm_sq > 1.15f * 1.15f ||
        gyro_norm_sq > 10.0f * 10.0f) return false;
  }
  for (int i = 0; i < 3; ++i) {
    if (gyro_m2[i] / (CALIBRATION_SAMPLES - 1) > 0.5f * 0.5f ||
        accel_m2[i] / (CALIBRATION_SAMPLES - 1) > 0.025f * 0.025f) return false;
  }
  // The planar estimator uses gyro Z only, so a sideways sensor cannot be
  // fixed by a sign flip. Require near-level mounting even with an override.
  if (fabsf(accel_mean[2]) <= 0.85f || fabsf(accel_mean[0]) >= 0.35f ||
      fabsf(accel_mean[1]) >= 0.35f) return false;
  // Slow constant rotation cannot be distinguished from sensor bias here.
  // The operator must keep the chassis stationary throughout calibration.
  g_gyro_z_bias_dps = gyro_mean[2];
  // An accelerometer measures specific force: on a stationary level surface
  // +Z pointing UP reads +1 g, despite gravitational acceleration pointing DOWN.
  // Thus sign(accel Z) converts sensor gyro Z to robot +Z-up yaw convention.
  // MPU9250 PS-MPU-9250A-01 section 9.1 gives shared accel/gyro axes/polarity.
  // This resolves inversion only; it does not compensate arbitrary tilt.
  g_yaw_sign = IMU_GYRO_Z_SIGN == 0 ? (accel_mean[2] > 0 ? 1 : -1) : IMU_GYRO_Z_SIGN;
  return true;
}
}  // namespace

bool imu_init(void) {
  g_configured = g_calibrated = false;
  g_gyro_z_bias_dps = 0;
  g_yaw_sign = 0;
  g_init_status = "I2C begin failed";
  g_identity = 0;
  if (!Wire.begin(I2C_SDA, I2C_SCL, 400000)) return false;
  Wire.setTimeOut(3);
  g_init_status = "No WHO_AM_I response at 0x68: check power, GND, SDA8, SCL9 and AD0";
  if (!read_regs(WHO_AM_I, &g_identity, 1)) return false;
  g_init_status = "Unexpected WHO_AM_I (MPU6500=0x70, MPU9250=0x71, MPU9255=0x73)";
  if (g_identity != 0x70 && g_identity != 0x71 && g_identity != 0x73) return false;
  g_init_status = "MPU register configuration failed";
  if (!write_reg(PWR_MGMT_1, 0x80)) return false;  // Reset configuration.
  delay(100);
  if (!write_reg(PWR_MGMT_1, 0x01) || !write_reg(PWR_MGMT_2, 0x00)) return false;
  for (size_t i = 0; i < sizeof(SENSOR_CONFIG); ++i) {
    if (!write_reg(SAMPLE_RATE_DIV + i, SENSOR_CONFIG[i])) return false;
  }
  if (!write_reg(INT_ENABLE, 0x01)) return false;
  uint8_t configured[5];
  if (!read_regs(SAMPLE_RATE_DIV, configured, sizeof(configured))) return false;
  for (size_t i = 0; i < sizeof(configured); ++i) {
    if (configured[i] != SENSOR_CONFIG[i]) return false;
  }
  delay(150);  // Sensor startup and DLPF settling before bias measurement.
  g_configured = true;
  g_init_status = "Calibration failed: keep robot stationary and sensor level, then reboot";
  g_calibrated = calibrate_bias();
  if (g_calibrated) g_init_status = "OK";
  return g_calibrated;
}

const char* imu_init_status(void) { return g_init_status; }
unsigned int imu_identity(void) { return g_identity; }

int imu_yaw_sign(void) {
  return g_yaw_sign;
}

ImuSample imu_read(void) {
  ImuSample result = {};
  result.calibrated = g_calibrated;
  if (!g_configured) return result;
  RawSample raw;
  if (!read_raw(raw)) return result;
  result.healthy = true;
  result.gyro_z_rad_s = g_yaw_sign * (raw.gyro_dps[2] - g_gyro_z_bias_dps) * RAD_PER_DEG;
  result.accel_x_m_s2 = raw.accel_g[0] * GRAVITY;
  result.accel_y_m_s2 = raw.accel_g[1] * GRAVITY;
  result.accel_z_m_s2 = raw.accel_g[2] * GRAVITY;
  return result;
}
