#pragma once

// SoftAP defaults for lab use (no home WiFi required).
#define WIFI_AP_SSID       "OMNI3_ESP32S3"
#define WIFI_AP_PASSWORD   "OmniRobot123"
#define WIFI_AP_CHANNEL    1
#define WIFI_AP_MAX_CONN   4

// Optional STA mode: set WIFI_USE_STA to 1 and fill credentials.
#define WIFI_USE_STA       0
#define WIFI_STA_SSID      "YOUR_SSID"
#define WIFI_STA_PASSWORD  "YOUR_PASSWORD"

#define HTTP_PORT          80
#define WS_PORT            81
