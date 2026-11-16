#include "serial_cmd.h"
#include "control.h"
#include "encoders.h"
#include "imu.h"

#include <Arduino.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>

static char g_line[96];
static size_t g_len = 0;
static bool g_discard_line = false;

void serial_cmd_init(void) {
  Serial.begin(SERIAL_BAUD);
}

static bool read_values(const char* args, float& a, float& b, float& c) {
  for (float* value : {&a, &b, &c}) {
    if (!isspace(static_cast<unsigned char>(*args))) return false;
    char* end;
    *value = strtof(args, &end);
    if (end == args || !isfinite(*value)) return false;
    args = end;
  }
  while (isspace(static_cast<unsigned char>(*args))) args++;
  return *args == 0;
}

static void handle_line(const char* line) {
  while (isspace(static_cast<unsigned char>(*line))) line++;
  if (*line == 0) return;

  char cmd = toupper(static_cast<unsigned char>(*line++));
  if (cmd == 'V' || cmd == 'B' || cmd == 'P') {
    float a, b, c;
    const float xy_limit = cmd == 'P' ? WORLD_COORD_LIMIT_M : SERIAL_LINEAR_LIMIT_M_S;
    const float angle_limit = cmd == 'P' ? SERIAL_YAW_LIMIT_DEG : SERIAL_ANGULAR_LIMIT_RAD_S;
    if (!read_values(line, a, b, c) || fabsf(a) > xy_limit || fabsf(b) > xy_limit ||
        fabsf(c) > angle_limit) {
      Serial.println(F("ERR invalid motion command"));
      return;
    }
    bool accepted = false;
    if (cmd == 'V') accepted = control_set_velocity_global(a, b, c);
    if (cmd == 'B') accepted = control_set_velocity_body(a, b, c);
    if (cmd == 'P') accepted = control_set_pose_target(a, b, c * (PI / 180.0f));
    Serial.printf("%s %c\n", accepted ? "OK" : "ERR calibration required", cmd);
    return;
  }

  if (cmd == 'C') {
    float a, b, c;
    bool accepted = read_values(line, a, b, c) && control_set_encoder_cpr(a, b, c);
    Serial.println(accepted ? F("OK CPR (RAM only)") : F("ERR CPR: three measured counts/rev, while stopped"));
    return;
  }

  if (cmd == 'T') {
    float wheel, pwm, duration;
    bool valid = read_values(line, wheel, pwm, duration) &&
        wheel >= 1 && wheel <= 3 && floorf(wheel) == wheel &&
        pwm >= -MOTOR_TEST_PWM_MAX && pwm <= MOTOR_TEST_PWM_MAX && floorf(pwm) == pwm &&
        duration >= 50 && duration <= MOTOR_TEST_DURATION_MAX_MS && floorf(duration) == duration;
    bool accepted = valid && control_test_motor(static_cast<int>(wheel) - 1, static_cast<int>(pwm),
                                                static_cast<uint32_t>(duration));
    Serial.println(accepted ? F("OK T") : F("ERR T: STOP first; wheel 1..3, PWM +/-1..160, time 50..1500ms"));
    return;
  }

  while (isspace(static_cast<unsigned char>(*line))) line++;
  if (*line == 0 && cmd == 'H') {
    Serial.println(control_test_heartbeat() ? F("OK H") : F("ERR H: no active test"));
  } else if (*line == 0 && cmd == 'Z') {
    Serial.println(control_reset_odometry() ? F("OK origin") : F("ERR stop before reset"));
  } else if (*line == 0 && cmd == 'S') {
    control_command_stop();
    Serial.println(F("OK S"));
  } else if (*line == 0 && cmd == '?') {
    ControlSnapshot snapshot = control_get_snapshot();
    const Pose2D& p = snapshot.pose;
    const float* w = snapshot.omega_meas;
    int64_t counts[3];
    encoders_get_raw_counts(counts);
    Serial.printf("P,%.4f,%.4f,%.2f,w,%.3f,%.3f,%.3f,ticks,%lld,%lld,%lld\n",
                  p.x, p.y, p.theta * (180.0f / PI), w[0], w[1], w[2],
                  static_cast<long long>(counts[0]), static_cast<long long>(counts[1]),
                  static_cast<long long>(counts[2]));
    Serial.printf("distance,%.4f,error,%.4f,imu,%d,imu_cal,%d,enc_cal,%d,gyro,%.5f,fault,%s\n",
                  snapshot.distance_m, snapshot.target_error_m, snapshot.imu_healthy,
                  snapshot.imu_calibrated, snapshot.encoder_calibrated,
                  snapshot.gyro_z_rad_s, snapshot.fault);
    Serial.printf("imu_id,0x%02X,yaw_sign,%d,imu_init,%s\n",
                  imu_identity(), imu_yaw_sign(), imu_init_status());
    Serial.printf("state,%d,mode,%d\n", static_cast<int>(snapshot.state), static_cast<int>(snapshot.mode));
  } else {
    Serial.println(F("ERR unknown command"));
  }
}

void serial_cmd_poll(void) {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n') {
      if (g_discard_line) {
        Serial.println(F("ERR invalid line"));
      } else {
        g_line[g_len] = 0;
        handle_line(g_line);
      }
      g_len = 0;
      g_discard_line = false;
    } else if (!g_discard_line) {
      if (c == 0 || g_len + 1 >= sizeof(g_line)) {
        g_discard_line = true;
      } else {
        g_line[g_len++] = c;
      }
    }
  }
}
