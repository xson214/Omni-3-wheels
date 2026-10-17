#include "serial_cmd.h"
#include "control.h"
#include "config.h"
#include "kinematics.h"

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

static char g_line[96];
static size_t g_len = 0;

void serial_cmd_init(void) {
  Serial.begin(SERIAL_BAUD);
  g_len = 0;
}

static void handle_line(char* line) {
  while (*line == ' ' || *line == '\t') line++;
  if (*line == 0) return;

  char cmd = line[0];
  if (cmd == 'V' || cmd == 'v') {
    float vx = 0, vy = 0, w = 0;
    if (sscanf(line + 1, "%f %f %f", &vx, &vy, &w) >= 1) {
      control_set_velocity_global(vx, vy, w);
      Serial.println(F("OK V"));
    }
  } else if (cmd == 'B' || cmd == 'b') {
    float vx = 0, vy = 0, w = 0;
    if (sscanf(line + 1, "%f %f %f", &vx, &vy, &w) >= 1) {
      control_set_velocity_body(vx, vy, w);
      Serial.println(F("OK B"));
    }
  } else if (cmd == 'P' || cmd == 'p') {
    float x = 0, y = 0, yaw_deg = 0;
    if (sscanf(line + 1, "%f %f %f", &x, &y, &yaw_deg) >= 2) {
      control_set_pose_target(x, y, yaw_deg * PI / 180.0f);
      Serial.println(F("OK P"));
    }
  } else if (cmd == 'S' || cmd == 's') {
    control_command_stop();
    control_set_mode(MODE_STOP);
    Serial.println(F("OK S"));
  } else if (cmd == '?') {
    Pose2D p = control_get_pose();
    float w[3];
    control_get_omega_meas(w);
    Serial.printf("P,%.4f,%.4f,%.2f,w,%.3f,%.3f,%.3f\n",
                  p.x, p.y, p.theta * 180.0f / PI, w[0], w[1], w[2]);
  } else {
    Serial.println(F("ERR unknown"));
  }
}

void serial_cmd_poll(void) {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      g_line[g_len] = 0;
      handle_line(g_line);
      g_len = 0;
    } else if (g_len + 1 < sizeof(g_line)) {
      g_line[g_len++] = c;
    } else {
      g_len = 0;
    }
  }
}
