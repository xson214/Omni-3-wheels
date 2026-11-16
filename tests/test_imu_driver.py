"""Compile the real IMU driver against a fake I2C device; no hardware access."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

ARDUINO = r"""
#pragma once
#include <stdint.h>
#include <stddef.h>
inline void delay(unsigned long) {}
"""

WIRE = r"""
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <vector>
struct FakeWire {
  uint8_t registers[256] = {};
  std::vector<uint8_t> tx, rx;
  size_t offset = 0;
  bool fail = false, ready = true, noisy = false, corrupt = false;
  int ax = 0, ay = 0, az = 16384, gx = 0, gy = 0, gz = 131;
  int reads = 0;
  uint16_t timeout = 0;
  bool begin(int sda, int scl, uint32_t hz) {
    return sda == 8 && scl == 9 && hz == 400000;
  }
  void setTimeOut(uint16_t value) { timeout = value; }
  void beginTransmission(uint8_t address) { tx.clear(); }
  void write(uint8_t value) { tx.push_back(value); }
  uint8_t endTransmission(bool stop) {
    if (fail) return 2;
    if (tx.size() == 2) registers[tx[0]] = tx[1];
    return 0;
  }
  void word(int index, int value) {
    rx[index] = static_cast<uint16_t>(value) >> 8;
    rx[index+1] = static_cast<uint16_t>(value) & 255;
  }
  size_t requestFrom(uint8_t address, size_t count, bool stop) {
    offset = 0;
    rx.assign(count, 0);
    if (fail) return 0;
    if (tx[0] == 0x3A) {
      rx[0] = ready ? 1 : 0;
      word(1, ax); word(3, ay); word(5, az);
      word(9, noisy ? ((++reads % 2) ? 262 : -262) : gx);
      word(11, gy); word(13, gz);
    } else {
      for (size_t i=0; i<count; ++i) rx[i] = registers[tx[0]+i];
      if (corrupt && tx[0] == 0x19) rx[0] = 0;
    }
    return count;
  }
  int available() { return static_cast<int>(rx.size()-offset); }
  int read() { return rx[offset++]; }
};
extern FakeWire Wire;
"""

HARNESS = r"""
#include "imu.h"
#include "Wire.h"
#include <cassert>
#include <cmath>
#include <iostream>
FakeWire Wire;
void fresh(uint8_t id = 0x71) {
  Wire = FakeWire{};
  Wire.registers[0x75] = id;
}
int main() {
  const int upright_sign = IMU_GYRO_Z_SIGN == 0 ? 1 : IMU_GYRO_Z_SIGN;
  const int inverted_sign = IMU_GYRO_Z_SIGN == 0 ? -1 : IMU_GYRO_Z_SIGN;
  fresh();
  assert(imu_init());
  assert(imu_yaw_sign() == upright_sign);
  assert(Wire.timeout == 3);
  auto sample = imu_read();
  assert(sample.healthy && sample.calibrated);
  assert(std::fabs(sample.gyro_z_rad_s) < 1e-5f);
  assert(std::fabs(sample.accel_z_m_s2 - 9.80665f) < 1e-5f);
  Wire.gz = 1441;  // 11 dps raw minus 1 dps startup bias.
  sample = imu_read();
  assert(std::fabs(sample.gyro_z_rad_s - upright_sign * 0.174532925f) < 1e-5f);
  Wire.gz = -1179; // -9 dps raw minus 1 dps startup bias.
  assert(std::fabs(imu_read().gyro_z_rad_s + upright_sign * 0.174532925f) < 1e-5f);
  Wire.fail = true;
  sample = imu_read();
  assert(!sample.healthy && sample.gyro_z_rad_s == 0);
  Wire.fail = false;
  assert(imu_read().healthy); // A transient read failure does not latch off.
  Wire.ready = false;
  assert(!imu_read().healthy);
  Wire.ready = true;
  Wire.gz = 32767;
  assert(!imu_read().healthy);
  fresh(0x73);
  assert(imu_init()); // MPU9255 supported.
  fresh(0x70);
  assert(imu_init() && imu_identity() == 0x70); // Physical robot reports MPU6500.
  fresh(0x68);
  assert(!imu_init() && !imu_read().healthy);
  fresh(); Wire.corrupt = true;
  assert(!imu_init()); // Configuration readback mismatch.
  fresh(); Wire.noisy = true;
  assert(!imu_init()); // X-axis motion must reject even if Z stays steady.
  assert(!imu_read().calibrated);
  fresh(); Wire.gy = 1500;
  assert(!imu_init()); // Constant large rate on Y.
  fresh(); Wire.az = 0;
  assert(!imu_init()); // Free fall/no gravity, not a stationary robot.
  fresh(); Wire.az = 0; Wire.ay = 16384;
  assert(!imu_init() && imu_yaw_sign() == 0); // Sideways cannot use gyro Z for yaw.
  fresh(); Wire.az = 14189; Wire.ay = 8192;
  assert(!imu_init() && imu_yaw_sign() == 0); // 30-degree tilt is rejected.
  fresh(); Wire.az = -16384;
  assert(imu_init() && imu_yaw_sign() == inverted_sign);
  Wire.gz = -1179;
  sample = imu_read();
  assert(std::fabs(sample.gyro_z_rad_s + inverted_sign * 0.174532925f) < 1e-5f);
  assert(std::fabs(sample.accel_z_m_s2 + 9.80665f) < 1e-5f); // Still sensor axes.
  fresh();
  assert(imu_init() && imu_yaw_sign() == upright_sign); // No stale inverted sign.
  fresh(); Wire.fail = true;
  assert(!imu_init() && imu_yaw_sign() == 0);
  std::cout << "IMU driver fault/calibration/conversion checks passed\n";
}
"""


class ImuDriverTests(unittest.TestCase):
    def test_driver_with_fake_i2c(self):
        compiler = shutil.which("g++")
        if compiler is None:
            self.skipTest("g++ is required for the host IMU driver test")
        with tempfile.TemporaryDirectory(prefix="omni-imu-test-") as folder:
            path = Path(folder)
            (path / "Arduino.h").write_text(ARDUINO)
            (path / "Wire.h").write_text(WIRE)
            (path / "main.cpp").write_text(HARNESS)
            for sign in (0, 1, -1):
                binary = path / f"test_{sign}.exe"
                subprocess.run([
                    compiler, "-std=c++11", f"-DIMU_GYRO_Z_SIGN={sign}",
                    "-I", str(path), "-I", str(ROOT / "firmware/include"),
                    str(ROOT / "firmware/src/imu.cpp"), str(path / "main.cpp"),
                    "-o", str(binary),
                ], check=True, capture_output=True, text=True)
                result = subprocess.run([str(binary)], check=True, capture_output=True, text=True)
                self.assertIn("checks passed", result.stdout)


if __name__ == "__main__":
    unittest.main()
