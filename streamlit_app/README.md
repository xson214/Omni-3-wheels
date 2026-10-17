# OmniCar Streamlit + ESP32-S3

Full-stack control UI for the 3-wheel omni robot.

## Hardware / pin map

Pins are taken only from `README_Xe_3_Banh_Omni_ESP32S3.md`:

| Wheel | PWM | IN1 | IN2 | Enc A | Enc B |
|------:|----:|----:|----:|------:|------:|
| 1 | 4 | 5 | 6 | 9 | 10 |
| 2 | 7 | 15 | 16 | 11 | 12 |
| 3 | 17 | 18 | 8 | 13 | 14 |

Kinematics, PID gains, and control loop follow `README_FIRMWARE.md`.
Networking follows `idea.json`.

## 1. Flash firmware (PlatformIO)

```bash
cd firmware
pio run -t upload
pio device monitor -b 115200
```

Nếu build lỗi do đường dẫn có dấu tiếng Việt trên Windows, dùng junction ASCII:

```bat
mklink /J C:\omni_fw "E:\Xe Tự Hành và Robot di động\source\firmware"
cd /d C:\omni_fw
pio run
pio run -t upload
```

ESP32 creates SoftAP:

- SSID: `OmniCar`
- Password: `omnicar123`
- HTTP: `http://192.168.4.1/`
- WebSocket: `ws://192.168.4.1:81/`

Connect your PC Wi-Fi to `OmniCar` before starting Streamlit.

Optional STA mode: edit `firmware/include/wifi_config.h` (`WIFI_USE_STA 1`).

### Serial debug (USB)

| Cmd | Meaning |
|-----|---------|
| `V vx vy w` | Global velocity |
| `B vx vy w` | Body velocity |
| `P x y yaw_deg` | Pose target |
| `S` | Stop |
| `?` | Print pose + ω |

## 2. Run Streamlit

```bash
cd streamlit_app
python -m venv .venv
# Windows:
.venv\Scripts\activate
pip install -r requirements.txt
streamlit run app.py
```

In the sidebar, keep ESP32 IP as `192.168.4.1` (SoftAP).

## 3. Quick tests

1. HTTP metric shows OK after joining SoftAP.
2. MANUAL + stream joystick: wheels respond; release / disconnect → stop (300 ms firmware timeout).
3. Upload CSV trajectory → mode AUTO → START → robot walks waypoints.
4. STOP always zeros motors.

## WebSocket / HTTP messages

- `{"type":"joystick","x":0.3,"y":0.8}`
- `{"type":"mode","value":"MANUAL"|"AUTO"|"STOP"}`
- `{"type":"command","value":"START"|"STOP"|"PAUSE"|"RESUME"}`
- Telemetry: `{"type":"telemetry","x",..."theta","speed",...}`
- `POST/GET /trajectory` with `{"type":"trajectory","points":[{"x":0,"y":0},...]}`
