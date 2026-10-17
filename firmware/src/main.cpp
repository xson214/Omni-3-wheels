#include <Arduino.h>

#include "config.h"
#include "motors.h"
#include "encoders.h"
#include "pid_speed.h"
#include "control.h"
#include "serial_cmd.h"
#include "net_server.h"

static uint32_t g_last_ctrl_ms = 0;
static uint32_t g_last_telem_ms = 0;

void setup() {
  serial_cmd_init();
  delay(200);
  Serial.println(F("Omni 3-wheel firmware starting..."));

  motors_init();
  encoders_init();
  pid_init();
  control_init();
  net_server_init();

  g_last_ctrl_ms = millis();
  g_last_telem_ms = millis();
  Serial.println(F("Ready. Serial: V/B/P/S/?  WiFi SoftAP OmniCar"));
}

void loop() {
  serial_cmd_poll();
  net_server_loop();

  uint32_t now = millis();
  if (now - g_last_ctrl_ms >= (uint32_t)(CTRL_DT_S * 1000.0f)) {
    g_last_ctrl_ms = now;
    control_loop();
  }

  if (now - g_last_telem_ms >= TELEMETRY_PERIOD_MS) {
    g_last_telem_ms = now;
    net_server_send_telemetry();
  }
}
