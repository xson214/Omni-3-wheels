#include "serial_cmd.h"
#include "control.h"
#include "encoders.h"

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
    if (cmd == 'V') control_set_velocity_global(a, b, c);
    if (cmd == 'B') control_set_velocity_body(a, b, c);
    if (cmd == 'P') control_set_pose_target(a, b, c * (PI / 180.0f));
    Serial.printf("OK %c\n", cmd);
    return;
  }

  while (isspace(static_cast<unsigned char>(*line))) line++;
  if (*line == 0 && cmd == 'S') {
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
