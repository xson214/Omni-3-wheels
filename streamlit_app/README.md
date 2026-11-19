# OmniCar Streamlit + ESP32-S3

The app sends MANUAL and AUTO commands and displays telemetry from the ESP32-S3. See the [hardware pin map and encoder calibration](../README_Xe_3_Banh_Omni_ESP32S3.md) and [firmware specification](../README_FIRMWARE.md).

## 1. Flash firmware

From the repository root, run

~~~powershell
cd firmware
pio run -t upload
pio device monitor -b 115200
~~~

The ESP32 starts the **OMNI3_ESP32S3** Wi-Fi access point with password **OmniRobot123**, matching the reference `servo.ino`. The address is **192.168.4.1**, with HTTP on port 80 and WebSocket on port 81. No router or Internet connection is required. For STA mode, set **WIFI_USE_STA** and the Wi-Fi credentials in [wifi_config.h](../firmware/include/wifi_config.h), then use the address printed on Serial.

### Điều khiển trực tiếp bằng trình duyệt

1. Cấp nguồn ESP32-S3; giữ xe đứng yên trong lúc firmware hiệu chuẩn gyro MPU.
2. Trên máy tính, kết nối Wi-Fi **OMNI3_ESP32S3**, mật khẩu **OmniRobot123**. Giữ kết nối dù Windows báo không có Internet.
3. Mở [http://192.168.4.1/](http://192.168.4.1/) để dùng trang điều khiển có sẵn trên ESP32. Trang này không cần Streamlit, Python hay Internet.
4. Nâng bánh khỏi mặt đất, tắt nguồn động cơ nhưng giữ nguồn ESP32/encoder, bấm **STOP**. Đánh dấu từng bánh. Để **Số vòng quay mỗi bánh = 5**, bấm **Ghi số đếm ban đầu**, quay tay lần lượt mỗi bánh đúng 5 vòng cùng một chiều, đợi bánh đứng yên rồi bấm **Tính CPR**. Trang tính `abs(ticks_cuối − ticks_đầu) / số_vòng` cho từng bánh và không cho xác nhận nếu bất kỳ bánh nào chưa có số đếm thay đổi. Kiểm tra kết quả rồi bấm **Xác nhận CPR đã đo**. Có thể nhập trực tiếp CPR đã đo từ trước. CPR gửi qua mạng chỉ lưu trong RAM; cần xác nhận lại sau mỗi lần khởi động nếu chưa cấu hình cố định trong firmware. Mất kết nối khi đang đo sẽ hủy phép đo; ghi mốc mới để đo lại.
5. Khi xe đứng yên ở vị trí xuất phát, bấm **Đặt gốc odometry**. Kiểm tra encoder đã xác nhận và MPU sẵn sàng trước khi chạy AUTO.
6. Chọn **MANUAL → Chọn chế độ → START**, giữ một nút hướng để di chuyển, nhả nút để dừng. Hoặc nhập quỹ đạo theo mét, **Nạp quỹ đạo → AUTO → Chọn chế độ → START**.

**STOP** kết thúc chuyển động. Khi WebSocket của máy đang điều khiển bị ngắt, firmware dừng cả MANUAL và AUTO; kết nối lại phải chọn chế độ và START lại. MANUAL còn có watchdog joystick 500 ms. Mất mạng không gửi khung đóng được phát hiện bằng ping/pong WebSocket, nên AUTO có thể mất vài giây mới phát hiện.

Trạng thái cũng có tại [http://192.168.4.1/status](http://192.168.4.1/status). Đây là số đo/ước lượng từ encoder và gyro, không phải đo vị trí tuyệt đối.

## 2. Start Streamlit

Streamlit is optional. Install its dependencies while the PC still has Internet, then join the ESP32 network. Use the same computer for Streamlit and the browser so the manual control pad can reach its local WebSocket relay. From the repository root on Windows, run

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

If the ESP32 misses the joystick heartbeat for 500 ms during motion, it stops and requires START before accepting another drive command. After **PAUSE** and **RESUME**, start a fresh hold on the pad.

## 4. Follow a trajectory in AUTO

Stop the current run, then enter one **x,y** pair per line and upload the trajectory. Supply 1–64 points. Each coordinate must be a finite number between −1000 m and +1000 m. Coordinates are in the odometry frame whose origin and heading are set when the ESP32 boots. Upload is allowed while IDLE and leaves the mode unchanged.

~~~csv
0.0,0.0
0.5,0.0
1.0,0.5
~~~

Apply **AUTO**, then press **START**. The robot follows the uploaded points and reports its progress in telemetry. **PAUSE** and **RESUME** preserve the current target. **STOP** ends the run. Losing the controlling WebSocket stops AUTO as well as MANUAL.

## 5. Kinematics and path simulation

These three tabs work without connecting to the ESP32.

- **Động học thuận** takes three signed wheel speeds in rad/s and the current heading in degrees. It returns global-frame ẋ, ẏ (m/s) and φ̇ (rad/s).
- **Động học nghịch** takes a target x, y in meters and the current pose. This educational calculation uses proportional position feedback ẋ = 0.6 × (x_target − x), ẏ = 0.6 × (y_target − y), and φ̇ = 0. It is a simplified illustration, not the full firmware PID trajectory controller. Within 0.01 m of the target, the requested velocity is zero. The tab shows wheel speeds before and after scaling all three to the 5.236 rad/s limit.
- **Mô phỏng đường đi** accepts its own list of points in meters. Edit the list or use **Sao chép điểm từ Autonomous**, then click **Chạy mô phỏng**. The simulation starts from (0 m, 0 m, 0°), runs for up to 60 seconds and plots three paths on the same x-y axes in meters. **Lý thuyết** integrates the ideal wheel motion. **Encoder** estimates position from simulated signed wheel ticks using forward kinematics. **IMU** integrates simulated gyroscope and accelerometer readings, with adjustable slip and sensor biases. A high encoder resolution can make the theory and encoder paths overlap.

See the [kinematics equations](../README_FIRMWARE.md#2-động-học). All three curves are generated in software. IMU drift in the chart comes from the simulation settings and does not represent a particular chip.

## 6. Protocol

[idea.json](../idea.json) lists the message shapes. WebSocket accepts the following messages.

~~~json
{"type":"mode","value":"MANUAL"}
{"type":"command","value":"START"}
{"type":"joystick","x":0.3,"y":0.8}
~~~

Modes are **MANUAL**, **AUTO**, and **STOP**. Commands are **START**, **STOP**, **PAUSE**, **RESUME**, and **RESET_ODOMETRY** (only when IDLE). Joystick coordinates are numbers in **[-1, 1]**. Status messages report the firmware's actual mode and state and may include an error. Telemetry reports position in meters, angle in radians, speed in m/s, wheel speeds in rad/s and trajectory progress. It also includes `distance_m` (estimated accumulated path length), `target_error_m`, `imu_healthy`, `imu_calibrated`, `encoder_calibrated`, `gyro_z_rad_s`, `ticks`, `cpr`, and `fault`. The battery field is **−1** because voltage measurement is unavailable.

While IDLE, confirm measured encoder counts with `{"type":"calibration","cpr":[N1,N2,N3]}`; replace N1–N3 with measured rising edges A per output-wheel revolution. Motion is blocked until CPR is confirmed; AUTO additionally requires a healthy, calibrated MPU. To set the current physical position and heading as the odometry origin, send `{"type":"command","value":"RESET_ODOMETRY"}` while stationary.

**GET /trajectory** retrieves the stored points. **POST /trajectory** accepts a JSON body and returns an error for invalid points or if the firmware is RUNNING or PAUSED. The server rejects bodies over 8 KiB.

~~~json
{"type":"trajectory","points":[{"x":0.0,"y":0.0},{"x":0.5,"y":0.0}]}
~~~

Cập nhật xử lý gián đoạn: joystick watchdog 500 ms; cầu nối trình duyệt chấp nhận gói mới dưới 300 ms và nhả giữ khi không có input 350 ms. MPU thiếu mẫu ngắn tạm dùng tốc độ góc encoder; mất mẫu liên tục khoảng 50 ms hoặc mất hiệu chuẩn thì dừng AUTO/POSE, không tự khởi động lại. Chưa xác định nguyên nhân vật lý của các lần mất mẫu trên xe.
