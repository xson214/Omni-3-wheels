"""Exercise bounded motor-test state transitions using real firmware control code.

Compiles host-only mocks. This test never opens a serial port or commands hardware.
"""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

ARDUINO = r'''
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <cstdlib>
#include <cmath>
extern uint32_t fake_now_ms;
inline uint32_t millis() { return fake_now_ms; }
template<class T> T constrain(T x, T lo, T hi) { return x < lo ? lo : (x > hi ? hi : x); }
'''
SEMAPHORE = r'''
#pragma once
#include <cassert>
#include <stdint.h>
struct StaticSemaphore_t { bool held = false; };
using SemaphoreHandle_t = StaticSemaphore_t*;
constexpr uint32_t portMAX_DELAY = UINT32_MAX;
inline SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t* s) { s->held = false; return s; }
inline void xSemaphoreTake(SemaphoreHandle_t s, uint32_t) { assert(s && !s->held); s->held = true; }
inline void xSemaphoreGive(SemaphoreHandle_t s) { assert(s && s->held); s->held = false; }
'''
HARNESS = r'''
#include "control.h"
#include "imu.h"
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>

uint32_t fake_now_ms = 100;
static int output[3] = {};
static int pid_updates = 0;
static int writes = 0;
void motors_init() {}
void set_motor(int index, int pwm, int direction) {
  assert(index >= 0 && index < 3);
  assert(pwm >= 0 && pwm <= PWM_MAX);
  assert(direction >= -1 && direction <= 1);
  output[index] = pwm * direction;
  ++writes;
}
void motors_stop_all() { for (int i = 0; i < 3; ++i) set_motor(i, 0, 0); }
void pid_reset() {}
void pid_update(const float[3], const float[3], float) { ++pid_updates; }
void encoders_init() {}
void encoders_set_cpr(const float[3]) {}
void encoders_get_raw_counts(int64_t counts[3]) { for (int i = 0; i < 3; ++i) counts[i] = 0; }
void encoders_sample(float tps[3], float omega[3], float dt) {
  assert(dt > 0);
  for (int i = 0; i < 3; ++i) tps[i] = omega[i] = 0;
}
static bool imu_healthy = true;
ImuSample imu_read() { return {imu_healthy, true, 0, 0, 0, 9.80665f}; }
bool imu_init() { return true; }
static void expect_stopped() {
  assert(output[0] == 0 && output[1] == 0 && output[2] == 0);
  assert(control_get_snapshot().state == STATE_IDLE);
  assert(!control_get_snapshot().encoder_calibrated);
}
static void step(uint32_t elapsed_ms, float dt = 0.01f) {
  fake_now_ms += elapsed_ms;
  control_loop(dt);
}
static void require_only(int wheel, int pwm) {
  for (int i = 0; i < 3; ++i) assert(output[i] == (i == wheel ? pwm : 0));
  assert(control_get_snapshot().state == STATE_RUNNING);
  assert(control_get_snapshot().mode == MODE_MOTOR_TEST);
  assert(!control_get_snapshot().encoder_calibrated);
  assert(pid_updates == 0); // Generic PID must not overwrite diagnostic output.
}
static void reset_stopped() {
  control_command_stop();
  expect_stopped();
  assert(control_get_snapshot().mode == MODE_STOP);
}
int main() {
  static_assert(!ENCODER_CALIBRATED_DEFAULT, "Tests require the uncalibrated factory default");
  control_init();
  expect_stopped();
  assert(!control_test_heartbeat());
  assert(!control_test_motor(-1, 80, 200));
  assert(!control_test_motor(3, 80, 200));
  assert(!control_test_motor(0, 0, 200));
  assert(!control_test_motor(0, MOTOR_TEST_PWM_MAX + 1, 200));
  assert(!control_test_motor(0, -MOTOR_TEST_PWM_MAX - 1, 200));
  assert(!control_test_motor(0, 80, 49));
  assert(!control_test_motor(0, 80, MOTOR_TEST_DURATION_MAX_MS + 1));
  assert(!control_test_motor(0, 80, UINT32_MAX));
  expect_stopped();

  // Merely being IDLE is insufficient; diagnostics require explicit STOP mode.
  assert(control_set_mode(MODE_MANUAL));
  assert(!control_test_motor(0, 80, 200));
  assert(!control_command_start()); // Normal movement still needs CPR.
  assert(control_set_mode(MODE_AUTO));
  assert(!control_test_motor(0, 80, 200));
  reset_stopped();

  assert(control_test_motor(1, 80, 350));
  assert(!control_test_motor(2, 100, 1500)); // Active test cannot change wheel or extend duration.
  assert(!control_set_encoder_cpr(500, 500, 500));
  assert(!control_reset_odometry());
  assert(!control_command_pause());
  step(10);
  require_only(1, 80);
  for (int i = 0; i < 3; ++i) {
    step(100);
    assert(control_test_heartbeat());
    require_only(1, 80);
  }
  step(30); // elapsed 340 ms
  assert(control_test_heartbeat());
  step(10); // Hard deadline 350 ms despite a fresh heartbeat.
  expect_stopped();
  assert(control_get_snapshot().mode == MODE_STOP);
  assert(!control_test_heartbeat());
  const int stopped_writes = writes;
  step(100);
  assert(writes == stopped_writes); // No implicit restart after completion.

  // Without H, a long requested duration cannot exceed the heartbeat budget.
  assert(control_test_motor(2, -MOTOR_TEST_PWM_MAX, MOTOR_TEST_DURATION_MAX_MS));
  step(10);
  require_only(2, -MOTOR_TEST_PWM_MAX);
  step(CMD_WATCHDOG_MS - 11);
  require_only(2, -MOTOR_TEST_PWM_MAX);
  step(1);
  expect_stopped();
  assert(!control_test_heartbeat());

  // A late heartbeat cannot revive a test even before the next loop executes.
  assert(control_test_motor(0, 60, 1000));
  step(10);
  require_only(0, 60);
  fake_now_ms += CMD_WATCHDOG_MS - 10;
  assert(!control_test_heartbeat());
  expect_stopped();

  // STOP is synchronous and cancels subsequent heartbeat/loop continuation.
  assert(control_test_motor(0, -90, 700));
  step(10);
  require_only(0, -90);
  reset_stopped();
  assert(!control_test_heartbeat());
  step(50);
  expect_stopped();

  // Overlong control periods must de-energize the test output.
  assert(control_test_motor(1, MOTOR_TEST_PWM_MAX, 1000));
  step(10);
  require_only(1, MOTOR_TEST_PWM_MAX);
  step(101, CONTROL_MAX_DT_S + 0.001f);
  expect_stopped();
  assert(std::strstr(control_get_snapshot().fault, "overrun"));
  assert(control_get_snapshot().loop_overruns == 1);
  assert(control_get_snapshot().loop_dt_ms > CONTROL_MAX_DT_S * 1000);
  assert(control_get_snapshot().loop_max_dt_ms >= control_get_snapshot().loop_dt_ms);
  assert(!control_test_heartbeat());
  reset_stopped();

  // Unsigned millis subtraction must preserve both deadlines across wraparound.
  fake_now_ms = UINT32_MAX - 50;
  assert(control_test_motor(2, 70, 150));
  step(10);
  require_only(2, 70);
  step(80);
  assert(control_test_heartbeat());
  require_only(2, 70);
  step(60);
  expect_stopped();

  // Testing an encoder never marks its CPR measured or permits normal movement.
  assert(!control_set_velocity_body(.02f, 0, 0));
  assert(!control_set_velocity_global(.02f, 0, 0));
  assert(!control_set_pose_target(.1f, 0, 0));
  assert(control_set_mode(MODE_MANUAL));
  assert(!control_command_start());
  assert(!control_set_joystick(.1f, 0));
  TrajPoint points[1] = {{.1f, 0}};
  assert(control_set_trajectory(points, 1) == TrajectoryResult::Ok);
  assert(control_set_mode(MODE_AUTO));
  assert(!control_command_start());
  reset_stopped();
  assert(pid_updates == 0);
  assert(control_recalibrate_imu());
  assert(!control_test_motor(0, 80, 100));
  assert(!control_set_velocity_body(.02f, 0, 0));
  step(10);
  step(3300);
  step(10);
  expect_stopped();
  assert(control_get_snapshot().imu_calibrated);
  assert(control_set_encoder_cpr(2028,2028,2028));
  assert(control_set_mode(MODE_AUTO));
  assert(control_command_start());
  imu_healthy = false;
  step(10); step(10);
  assert(control_get_snapshot().state == STATE_RUNNING);
  imu_healthy = true; step(10);
  assert(control_get_snapshot().state == STATE_RUNNING);
  imu_healthy = false;
  for(int i=0;i<6;++i) step(10);
  assert(control_get_snapshot().state == STATE_IDLE);
  imu_healthy = true; step(10);
  assert(control_get_snapshot().state == STATE_IDLE); // No automatic restart.
  assert(control_set_mode(MODE_MANUAL));
  assert(control_command_start());
  assert(control_set_joystick(.1f,0));
  step(350);
  assert(control_get_snapshot().state == STATE_RUNNING);
  step(150);
  assert(control_get_snapshot().state == STATE_IDLE);
  std::cout << "PASS bounded motor diagnostics: limits, isolation, deadlines, heartbeat, STOP, overrun, wrap, calibration gates\n";
}
'''


class MotorTestControlTests(unittest.TestCase):
    def test_bounded_diagnostics(self):
        compiler = shutil.which(os.environ.get("CXX", "g++"))
        if compiler is None:
            self.skipTest("g++ is required for the host control test")
        with tempfile.TemporaryDirectory(prefix="omni-control-test-") as folder:
            path = Path(folder)
            (path / "freertos").mkdir()
            (path / "Arduino.h").write_text(ARDUINO, encoding="utf-8")
            (path / "freertos/semphr.h").write_text(SEMAPHORE, encoding="utf-8")
            (path / "main.cpp").write_text(HARNESS, encoding="utf-8")
            binary = path / ("test.exe" if os.name == "nt" else "test")
            subprocess.run([
                compiler, "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
                "-DENCODER_CALIBRATED_DEFAULT=false",
                "-I", str(path), "-I", str(ROOT / "firmware/include"),
                *(str(ROOT / "firmware/src" / filename) for filename in
                  ("control.cpp", "kinematics.cpp", "tracking.cpp")),
                str(path / "main.cpp"), "-o", str(binary),
            ], check=True)
            result = subprocess.run([str(binary)], check=True, capture_output=True, text=True)
            self.assertIn("PASS bounded motor diagnostics", result.stdout)
            print(result.stdout.strip())


if __name__ == "__main__":
    unittest.main()
