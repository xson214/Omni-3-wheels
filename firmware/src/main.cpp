#include <Arduino.h>

#include "config.h"
#include "motors.h"
#include "encoders.h"
#include "control.h"
#include "serial_cmd.h"
#include "net_server.h"

static bool g_ready = false;

static void control_task(void*) {
  const TickType_t period = pdMS_TO_TICKS(CTRL_PERIOD_MS);
  TickType_t wake = xTaskGetTickCount();
  uint32_t previous_us = micros();

  for (;;) {
    vTaskDelayUntil(&wake, period);
    uint32_t now_us = micros();
    float dt_s = (now_us - previous_us) * 1e-6f;
    previous_us = now_us;
    control_loop(dt_s);
    // Skip missed periods rather than take several encoder samples back to back.
    if (xTaskGetTickCount() - wake >= period) wake = xTaskGetTickCount();
  }
}

static void http_task(void*) {
  for (;;) {
    net_server_poll_http();
    vTaskDelay(1);
  }
}

static void network_task(void*) {
  net_server_init();
  if (xTaskCreatePinnedToCore(http_task, "omni-http", 8192, nullptr, 1, nullptr, xPortGetCoreID()) != pdPASS) {
    Serial.println(F("ERR HTTP task unavailable"));
  }
  uint32_t last_telemetry_ms = millis();
  for (;;) {
    net_server_poll_ws();
    uint32_t now = millis();
    if (now - last_telemetry_ms >= TELEMETRY_PERIOD_MS) {
      last_telemetry_ms = now;
      net_server_send_telemetry();
    }
    vTaskDelay(1);
  }
}

void setup() {
  motors_init();
  encoders_init();
  control_init();
  serial_cmd_init();

  BaseType_t core = xPortGetCoreID();
  if (xTaskCreatePinnedToCore(control_task, "omni-control", 4096, nullptr, 3, nullptr, core) != pdPASS) {
    Serial.println(F("ERR control task unavailable"));
    return;
  }
  if (xTaskCreatePinnedToCore(network_task, "omni-network", 8192, nullptr, 2, nullptr, core) != pdPASS) {
    Serial.println(F("ERR network task unavailable"));
  }

  g_ready = true;
  Serial.println(F("Ready. Serial: V/B/P/S/?"));
}

void loop() {
  if (g_ready) serial_cmd_poll();
  vTaskDelay(1);
}
