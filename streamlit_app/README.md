# OmniCar Streamlit + ESP32-S3

The app sends MANUAL and AUTO commands and displays telemetry from the ESP32-S3. See the [hardware pin map and encoder calibration](../README_Xe_3_Banh_Omni_ESP32S3.md) and [firmware specification](../README_FIRMWARE.md).

## 1. Flash firmware

From the repository root, run

~~~powershell
cd firmware
pio run -t upload
pio device monitor -b 115200
~~~

The ESP32 starts the **OmniCar** Wi-Fi access point with password **omnicar123**. The default address is **192.168.4.1**, with HTTP on port 80 and WebSocket on port 81. For STA mode, set **WIFI_USE_STA** and the Wi-Fi credentials in [wifi_config.h](../firmware/include/wifi_config.h), then use the address printed on Serial.

## 2. Start Streamlit

Join the ESP32 network. Use the same computer for Streamlit and the browser so the manual control pad can reach its local WebSocket relay. From the repository root on Windows, run

~~~powershell
cd streamlit_app
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe -m streamlit run app.py
~~~

Open the **Local URL** printed by Streamlit. Enter the ESP32 IP in the sidebar. Use the HTTP check to verify reachability and confirm the firmware reports mode and state over WebSocket.

## 3. Drive in MANUAL

1. Select **MANUAL** and apply the mode. Check for **MANUAL / IDLE**.
2. Press **START**. Check for **MANUAL / RUNNING**.
3. Hold and drag on the pad to drive. **x** commands body-frame **vx**, **y** commands **vy**, and yaw rate is zero. Releasing the pad sends a zero command and leaves MANUAL ready for another hold.
4. Press **STOP** to end the run. Check for **STOP / IDLE**. Select MANUAL and press START again before driving.

If the ESP32 misses the joystick heartbeat for 300 ms during motion, it stops and requires START before accepting another drive command. After **PAUSE** and **RESUME**, start a fresh hold on the pad.

## 4. Follow a trajectory in AUTO

Stop the current run, then enter one **x,y** pair per line and upload the trajectory. Supply 1–64 points. Each coordinate must be a finite number between −1000 m and +1000 m. Coordinates are in the odometry frame whose origin and heading are set when the ESP32 boots. Upload is allowed while IDLE and leaves the mode unchanged.

~~~csv
0.0,0.0
0.5,0.0
1.0,0.5
~~~

Apply **AUTO**, then press **START**. The robot follows the uploaded points and reports its progress in telemetry. **PAUSE** and **RESUME** preserve the current target. **STOP** ends the run. AUTO can continue following the stored trajectory after a temporary Wi-Fi disconnect.

## 5. Protocol

[idea.json](../idea.json) lists the message shapes. WebSocket accepts the following messages.

~~~json
{"type":"mode","value":"MANUAL"}
{"type":"command","value":"START"}
{"type":"joystick","x":0.3,"y":0.8}
~~~

Modes are **MANUAL**, **AUTO**, and **STOP**. Commands are **START**, **STOP**, **PAUSE**, and **RESUME**. Joystick coordinates are numbers in **[-1, 1]**. Status messages report the firmware's actual mode and state and may include an error. Telemetry reports position in meters, angle in radians, speed in m/s, wheel speeds in rad/s and trajectory progress. The battery field is **−1** because voltage measurement is unavailable.

**GET /trajectory** retrieves the stored points. **POST /trajectory** accepts a JSON body and returns an error for invalid points or if the firmware is RUNNING or PAUSED. The server rejects bodies over 8 KiB.

~~~json
{"type":"trajectory","points":[{"x":0.0,"y":0.0},{"x":0.5,"y":0.0}]}
~~~
