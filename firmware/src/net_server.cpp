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

static constexpr size_t MAX_WS_COMMAND_BYTES = 256;
static constexpr size_t MAX_TRAJECTORY_BODY_BYTES = 8 * 1024;

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

static String status_json(const char* error = nullptr) {
  const ControlSnapshot snapshot = control_get_snapshot();
  JsonDocument doc;
  doc["type"] = "status";
  doc["mode"] = mode_name(snapshot.mode);
  doc["state"] = state_name(snapshot.state);
  if (error != nullptr) doc["error"] = error;
  String out;
  serializeJson(doc, out);
  return out;
}

static void send_status_to(uint8_t num, const char* error = nullptr) {
  String out = status_json(error);
  g_ws.sendTXT(num, out);
}

static bool read_number(JsonVariantConst value, double& result) {
  if (!value.is<double>()) return false;
  result = value.as<double>();
  return isfinite(result);
}

static void report_command_result(uint8_t num, bool accepted) {
  if (!accepted) {
    send_status_to(num, "command not allowed in current mode/state");
    return;
  }
  String out = status_json();
  g_ws.broadcastTXT(out);
}

static void handle_ws_message(uint8_t num, uint8_t* payload, size_t len) {
  if (len > MAX_WS_COMMAND_BYTES) {
    send_status_to(num, "message too large");
    return;
  }
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload, len, DeserializationOption::NestingLimit(4));
  if (err || !doc.is<JsonObject>()) {
    send_status_to(num, "invalid JSON message");
    return;
  }

  const char* type = doc["type"] | "";
  if (strcmp(type, "joystick") == 0) {
    double x, y;
    if (!read_number(doc["x"], x) || !read_number(doc["y"], y) ||
        fabs(x) > 1.0 || fabs(y) > 1.0) {
      send_status_to(num, "joystick x and y must be numbers in [-1, 1]");
      return;
    }
    control_set_joystick(x, y);
  } else if (strcmp(type, "mode") == 0) {
    const char* value = doc["value"] | "";
    ControlMode mode;
    if (strcasecmp(value, "MANUAL") == 0) {
      mode = MODE_MANUAL;
    } else if (strcasecmp(value, "AUTO") == 0) {
      mode = MODE_AUTO;
    } else if (strcasecmp(value, "STOP") == 0) {
      mode = MODE_STOP;
    } else {
      send_status_to(num, "unknown mode");
      return;
    }
    report_command_result(num, control_set_mode(mode));
  } else if (strcmp(type, "command") == 0) {
    // Keep the legacy command keys for clients that use them.
    const char* cmd = doc["value"] | "";
    if (cmd[0] == '\0') cmd = doc["command"] | "";
    if (cmd[0] == '\0') cmd = doc["cmd"] | "";
    bool accepted = true;
    if (strcasecmp(cmd, "START") == 0) {
      accepted = control_command_start();
    } else if (strcasecmp(cmd, "STOP") == 0) {
      control_command_stop();
    } else if (strcasecmp(cmd, "PAUSE") == 0) {
      accepted = control_command_pause();
    } else if (strcasecmp(cmd, "RESUME") == 0) {
      accepted = control_command_resume();
    } else {
      send_status_to(num, "unknown command");
      return;
    }
    report_command_result(num, accepted);
  } else {
    send_status_to(num, "unknown message type");
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

static void send_trajectory_error(int status, const char* error) {
  JsonDocument doc;
  doc["ok"] = false;
  doc["err"] = error;
  String out;
  serializeJson(doc, out);
  g_http.send(status, "application/json", out);
}

static void handle_post_trajectory(void) {
  if (!g_http.hasArg("plain")) {
    send_trajectory_error(400, "no body");
    return;
  }
  const String body = g_http.arg("plain");
  if (body.length() > MAX_TRAJECTORY_BODY_BYTES) {
    send_trajectory_error(413, "body must be at most 8 KiB");
    return;
  }
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, body, DeserializationOption::NestingLimit(4));
  if (err || !doc.is<JsonObject>()) {
    send_trajectory_error(400, "invalid JSON object");
    return;
  }
  if (!doc["type"].isUnbound() && strcmp(doc["type"] | "", "trajectory") != 0) {
    send_trajectory_error(400, "type must be trajectory when provided");
    return;
  }
  JsonArrayConst pts = doc["points"].as<JsonArrayConst>();
  if (pts.isNull() || pts.size() == 0 || pts.size() > TRAJ_MAX_POINTS) {
    send_trajectory_error(400, "points must be an array with 1 to 64 elements");
    return;
  }
  TrajPoint buf[TRAJ_MAX_POINTS];
  int n = 0;
  for (JsonVariantConst p : pts) {
    double x, y;
    if (!p.is<JsonObjectConst>() || !read_number(p["x"], x) || !read_number(p["y"], y) ||
        fabs(x) > WORLD_COORD_LIMIT_M || fabs(y) > WORLD_COORD_LIMIT_M) {
      send_trajectory_error(400, "each point needs numeric x and y in [-1000, 1000] m");
      return;
    }
    buf[n++] = {static_cast<float>(x), static_cast<float>(y)};
  }
  const TrajectoryResult result = control_set_trajectory(buf, n);
  if (result == TrajectoryResult::Busy) {
    send_trajectory_error(409, "stop the current run before uploading");
    return;
  }
  if (result != TrajectoryResult::Ok) {
    send_trajectory_error(400, "invalid trajectory");
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

void net_server_poll_http(void) {
  g_http.handleClient();
}

void net_server_poll_ws(void) {
  g_ws.loop();
}

void net_server_send_telemetry(void) {
  const ControlSnapshot snapshot = control_get_snapshot();
  JsonDocument doc;
  doc["type"] = "telemetry";
  doc["x"] = snapshot.pose.x;
  doc["y"] = snapshot.pose.y;
  doc["theta"] = snapshot.pose.theta;
  doc["speed"] = snapshot.speed;
  doc["battery"] = -1;
  doc["mode"] = mode_name(snapshot.mode);
  doc["state"] = state_name(snapshot.state);
  doc["traj_i"] = snapshot.traj_index;
  doc["traj_n"] = snapshot.traj_count;
  doc["w1"] = snapshot.omega_meas[0];
  doc["w2"] = snapshot.omega_meas[1];
  doc["w3"] = snapshot.omega_meas[2];
  String out;
  serializeJson(doc, out);
  g_ws.broadcastTXT(out);
}
