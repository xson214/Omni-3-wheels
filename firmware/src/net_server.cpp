#include "net_server.h"
#include "wifi_config.h"
#include "control.h"
#include "config.h"

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

static WebServer g_http(HTTP_PORT);
static WebSocketsServer g_ws(WS_PORT);

static const char* mode_name(ControlMode m) {
  switch (m) {
    case MODE_MANUAL: return "MANUAL";
    case MODE_AUTO: return "AUTO";
    case MODE_STOP: return "STOP";
    case MODE_VELOCITY_GLOBAL: return "VELOCITY";
    case MODE_VELOCITY_BODY: return "BODY";
    case MODE_POSE: return "POSE";
    default: return "STOP";
  }
}

static const char* state_name(RunState s) {
  switch (s) {
    case STATE_RUNNING: return "RUNNING";
    case STATE_PAUSED: return "PAUSED";
    default: return "IDLE";
  }
}

static void send_status_to(uint8_t num) {
  JsonDocument doc;
  doc["type"] = "status";
  doc["mode"] = mode_name(control_get_mode());
  doc["state"] = state_name(control_get_state());
  String out;
  serializeJson(doc, out);
  g_ws.sendTXT(num, out);
}

static void broadcast_status(void) {
  JsonDocument doc;
  doc["type"] = "status";
  doc["mode"] = mode_name(control_get_mode());
  doc["state"] = state_name(control_get_state());
  String out;
  serializeJson(doc, out);
  g_ws.broadcastTXT(out);
}

static void handle_ws_message(uint8_t num, uint8_t* payload, size_t len) {
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload, len);
  if (err) return;

  const char* type = doc["type"] | "";
  if (strcmp(type, "joystick") == 0) {
    float x = doc["x"] | 0.0f;
    float y = doc["y"] | 0.0f;
    control_set_joystick(x, y);
  } else if (strcmp(type, "mode") == 0) {
    const char* value = doc["value"] | "STOP";
    if (strcasecmp(value, "MANUAL") == 0) {
      control_set_mode(MODE_MANUAL);
    } else if (strcasecmp(value, "AUTO") == 0) {
      control_set_mode(MODE_AUTO);
    } else {
      control_set_mode(MODE_STOP);
    }
    broadcast_status();
  } else if (strcmp(type, "command") == 0) {
    const char* cmd = doc["value"] | doc["command"] | "";
    // support both {"type":"command","value":"STOP"} and {"type":"command","command":"STOP"}
    if (cmd[0] == 0) {
      // idea.json example uses allowed_values at top level sometimes — also accept "cmd"
      cmd = doc["cmd"] | "";
    }
    if (strcasecmp(cmd, "START") == 0) {
      control_command_start();
    } else if (strcasecmp(cmd, "STOP") == 0) {
      control_command_stop();
      control_set_mode(MODE_STOP);
    } else if (strcasecmp(cmd, "PAUSE") == 0) {
      control_command_pause();
    } else if (strcasecmp(cmd, "RESUME") == 0) {
      control_command_resume();
    }
    broadcast_status();
  }
}

static void on_ws_event(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      send_status_to(num);
      break;
    case WStype_TEXT:
      handle_ws_message(num, payload, length);
      break;
    default:
      break;
  }
}

static void handle_get_trajectory(void) {
  JsonDocument doc;
  doc["type"] = "trajectory";
  JsonArray pts = doc["points"].to<JsonArray>();
  TrajPoint buf[TRAJ_MAX_POINTS];
  int n = control_get_trajectory(buf, TRAJ_MAX_POINTS);
  for (int i = 0; i < n; i++) {
    JsonObject p = pts.add<JsonObject>();
    p["x"] = buf[i].x;
    p["y"] = buf[i].y;
  }
  String out;
  serializeJson(doc, out);
  g_http.send(200, "application/json", out);
}

static void handle_post_trajectory(void) {
  if (!g_http.hasArg("plain")) {
    g_http.send(400, "application/json", "{\"ok\":false,\"err\":\"no body\"}");
    return;
  }
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, g_http.arg("plain"));
  if (err) {
    g_http.send(400, "application/json", "{\"ok\":false,\"err\":\"bad json\"}");
    return;
  }
  JsonArray pts = doc["points"].as<JsonArray>();
  if (pts.isNull() || pts.size() == 0) {
    g_http.send(400, "application/json", "{\"ok\":false,\"err\":\"no points\"}");
    return;
  }
  TrajPoint buf[TRAJ_MAX_POINTS];
  int n = 0;
  for (JsonObject p : pts) {
    if (n >= TRAJ_MAX_POINTS) break;
    buf[n].x = p["x"] | 0.0f;
    buf[n].y = p["y"] | 0.0f;
    n++;
  }
  if (!control_set_trajectory(buf, n)) {
    g_http.send(500, "application/json", "{\"ok\":false}");
    return;
  }
  g_http.send(200, "application/json", "{\"ok\":true}");
}

static void handle_root(void) {
  g_http.send(200, "text/plain", "OmniCar ESP32-S3 OK");
}

static void setup_wifi(void) {
#if WIFI_USE_STA
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASSWORD);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(200);
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("STA IP: "));
    Serial.println(WiFi.localIP());
    return;
  }
  Serial.println(F("STA failed, falling back to SoftAP"));
#endif
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, WIFI_AP_CHANNEL, 0, WIFI_AP_MAX_CONN);
  Serial.printf("SoftAP %s (%s) IP=%s\n",
                ok ? "OK" : "FAIL",
                WIFI_AP_SSID,
                WiFi.softAPIP().toString().c_str());
}

void net_server_init(void) {
  setup_wifi();

  g_http.on("/", HTTP_GET, handle_root);
  g_http.on("/trajectory", HTTP_GET, handle_get_trajectory);
  g_http.on("/trajectory", HTTP_POST, handle_post_trajectory);
  g_http.begin();

  g_ws.begin();
  g_ws.onEvent(on_ws_event);

  Serial.printf("HTTP :%d  WS :%d\n", HTTP_PORT, WS_PORT);
}

void net_server_loop(void) {
  g_http.handleClient();
  g_ws.loop();
}

void net_server_send_telemetry(void) {
  Pose2D p = control_get_pose();
  JsonDocument doc;
  doc["type"] = "telemetry";
  doc["x"] = p.x;
  doc["y"] = p.y;
  doc["theta"] = p.theta;
  doc["speed"] = control_get_speed();
  doc["battery"] = -1;
  doc["mode"] = mode_name(control_get_mode());
  doc["state"] = state_name(control_get_state());
  doc["traj_i"] = control_traj_index();
  doc["traj_n"] = control_traj_count();
  float w[3];
  control_get_omega_meas(w);
  doc["w1"] = w[0];
  doc["w2"] = w[1];
  doc["w3"] = w[2];
  String out;
  serializeJson(doc, out);
  g_ws.broadcastTXT(out);
}
