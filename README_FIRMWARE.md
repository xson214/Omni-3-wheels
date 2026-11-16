# Firmware xe Omni 3 bánh

Firmware PlatformIO cho ESP32-S3, ba L298N/GM25 encoder và MPU9250. [Sơ đồ chân](README_Xe_3_Banh_Omni_ESP32S3.md), [đối chiếu slide](docs/DOI_CHIEU_CONG_THUC.md), [hướng dẫn Wi-Fi](streamlit_app/README.md).

## Kết nối và chạy

ESP32 phát Wi-Fi **OMNI3_ESP32S3**, mật khẩu **OmniRobot123**. Máy tính kết nối mạng này rồi mở **http://192.168.4.1**. Trang điều khiển chạy trực tiếp trên ESP32, không cần Internet hoặc Streamlit. HTTP cổng 80, WebSocket cổng 81. Streamlit vẫn dùng được.

Giữ xe đứng yên và nằm ngang khoảng 3,3 giây khi bật nguồn để hiệu chuẩn gyro. CPR mặc định đã cấu hình danh định là **2028 = 26 × 78 tick/vòng**, dựa trên thông số nhà bán hàng do người dùng cung cấp và cách đếm A-RISING 1x. Chưa kiểm chứng số vòng thực tế. Đo encoder bằng chức năng ghi mốc / tính CPR trên trang web hoặc nhập ba CPR đã đo qua Serial. AUTO/POSE còn yêu cầu MPU khỏe và hiệu chuẩn thành công. Các thông số CPR và quỹ đạo nhập lúc chạy nằm trong RAM, cần nhập lại sau khởi động.

## Thông số và công thức

| Thông số | Giá trị |
|---|---|
| Bán kính bánh r / khoảng cách tâm L | 0,029 m / 0,12 m |
| Góc bánh α | 0°, +120°, −120° |
| Tốc độ bánh tối đa | 50 rpm ≈ 5,236 rad/s |
| Chu kỳ điều khiển | 10 ms |
| PID tốc độ Kp / Ki / Kd | 150 / 100 / 0,02, thời gian tính bằng giây |
| Feedforward Kv | 255 / (50/60) PWM/(vòng/s) |
| PID vị trí x/y | 0,8 / 0,08 / 0,10 |
| PID yaw | 1,5 / 0,05 / 0,10 |
| Tốc độ / gia tốc tham chiếu AUTO | 0,06 m/s / 0,08 m/s² |
| Dung sai vị trí / góc | 0,01 m / 0,06 rad |

Các hệ số PID là giá trị khởi đầu, chưa được hiệu chỉnh trên xe thật. Dung sai 1 cm áp dụng cho **ước lượng odometry**, không phải cam kết sai số vị trí thực. Độ trượt bánh và sai số kích thước vẫn gây sai lệch.

Động học nghịch trong hệ toàn cục:

```text
ω_i = [-sin(φ + α_i) × vx + cos(φ + α_i) × vy + L × wz] / r
```

Trong hệ thân xe dùng φ=0. Biến đổi toàn cục sang thân xe: `vx_b = cosφ vx + sinφ vy`, `vy_b = -sinφ vx + cosφ vy`. Động học thuận dùng tốc độ bánh có dấu:

```text
vx = (2r/3) × Σ[-sin(φ + α_i) × ω_i]
vy = (2r/3) × Σ[ cos(φ + α_i) × ω_i]
wz = r/(3L) × Σω_i
```

Ma trận khớp slide 8 và 10–12; các lỗi dấu tại những trang khai triển được ghi trong báo cáo đối chiếu. Slide không quy định PID. Firmware điều khiển tốc độ/vị trí, không điều khiển trực tiếp lực Newton qua PWM L298N.

## Encoder và PID tốc độ

Đếm cạnh lên A, đọc B xác định chiều. Với N_i là CPR từng bánh, tốc độ có dấu `n_i = ENC_SIGN[i] × Δticks_i/(N_i × dt)` vòng/s, `ω_i = 2πn_i`. CPR mặc định là **2028 tick/vòng**; `ENCODER_CALIBRATED_DEFAULT=true` chấp nhận cấu hình danh định này, không chứng nhận hiệu chuẩn thực nghiệm. Nếu 26 xung của nhà bán hàng đã bao gồm nhiều cạnh, cần xác nhận lại quy ước và sửa CPR. Có thể đặt cờ false để yêu cầu xác nhận trước khi chạy. `MOTOR_SIGN` chỉnh chiều động cơ riêng với `ENC_SIGN`.

Đặt `d=sign(ω_ref)`, `n_ref=|ω_ref|/(2π)`, `n_meas=d × n_i`, `e=n_ref-n_meas`. Bánh quay ngược yêu cầu phải làm sai số tăng, không lấy trị tuyệt đối của tốc độ đo.

```text
I_candidate = clamp(I + Ki × e × dt, -255, 255)
D_raw = -(n_meas - n_meas_previous) / dt
D = lọc thông thấp D_raw với tau=0,04 s
PWM = clamp(Kv × n_ref + Kp × e + I + Kd × D, 0, 255)
```

Chỉ nhận bước tích phân không làm bão hòa nặng thêm. Xóa tích phân khi dừng hoặc đổi chiều. Nếu tốc độ đặt dưới 0,01 vòng/s thì tắt PWM. Feedforward Kv là ước lượng, cần hiệu chỉnh theo motor, nguồn và tải thật.

## Bám quỹ đạo và MPU

AUTO tạo tham chiếu dọc từng đoạn thẳng, tăng tốc và giảm tốc trước đích. Nếu tham chiếu đi trước vị trí ước lượng quá 5 cm thì giảm tốc tham chiếu. PID x/y hiệu chỉnh sai số vị trí và vận tốc; PID yaw giữ góc lúc START. P dùng cùng bộ bám với góc đích riêng. Giới hạn vận tốc thân xe, tốc độ góc và tốc độ bánh bằng một hệ số chung, đồng thời ngăn tích phân tăng khi bão hòa.

Chỉ kết thúc điểm khi sai số vị trí ≤1 cm, góc ≤0,06 rad, tốc độ ≤0,008 m/s và tốc độ quay ≤0,04 rad/s liên tục 0,3 giây. Xe dừng tại từng đỉnh trước khi sang đoạn tiếp theo. Timeout mỗi đoạn là `15 + 3 × chiều_dài / 0,06` giây. Tối đa 64 điểm.

MPU6500/9250/9255 tại 0x68 được kiểm tra WHO_AM_I, cấu hình ±250°/s, ±2g, lọc DLPF và lấy mẫu 200 Hz. Hiệu chuẩn gyro gồm kiểm tra chuyển động, bias và tư thế lắp. Khi cảm biến gần nằm ngang, dấu gia tốc Z tĩnh xác định trục Z hướng lên/xuống. Lắp nghiêng nhiều hoặc hiệu chuẩn thất bại sẽ khóa AUTO/POSE; khởi động lại khi xe đứng yên để thử lại. Telemetry có mã nhận dạng và lý do khởi tạo thất bại.

```text
wz_est = 0,98 × gyro_Z_đã_trừ_bias + 0,02 × wz_encoder
heading_mid = heading + 0,5 × dt × wz_est
x += vx(heading_mid) × dt
y += vy(heading_mid) × dt
heading = wrap(heading + wz_est × dt)
distance_m += hypot(vx,vy) × dt
```

Không tích phân kép gia tốc thành vị trí. Chưa sử dụng magnetometer. Góc yaw tương đối và vị trí vẫn trôi theo thời gian; không phải định vị tuyệt đối. `distance_m` là độ dài đường đi ước lượng, khác khoảng cách thẳng tới đích. MANUAL có thể dùng encoder khi MPU lỗi; AUTO/POSE dừng khi mất mẫu MPU.

## Chế độ, giao thức và watchdog

| Chế độ | Hoạt động / điều kiện dừng |
|---|---|
| MANUAL | START rồi giữ hướng; quá 300 ms không cập nhật joystick đang chuyển động thì dừng |
| AUTO | Nạp điểm, chọn AUTO, START; dừng khi xong, lỗi MPU, timeout hoặc nhận STOP |
| V / B / P | Làm mới lệnh trong dưới 200 ms; quá hạn thì dừng |
| PAUSE / RESUME | Tạm dừng motor; tiếp tục đoạn AUTO đang theo |

Ngắt WebSocket của máy đang điều khiển sẽ dừng xe, kể cả AUTO. Ping/pong phát hiện mất kết nối im lặng sau vài giây; mất joystick MANUAL vẫn có watchdog 300 ms. Máy chỉ theo dõi ngắt kết nối không ảnh hưởng. Chu kỳ điều khiển vượt 100 ms dừng và báo lỗi. STOP luôn đưa về STOP/IDLE.

HTTP: `GET /` trang điều khiển, `GET /status` trạng thái và ticks, `GET/POST /trajectory` đọc/nạp `{points:[{x,y},...]}`. WebSocket giữ các kiểu `mode`, `command`, `joystick`; bổ sung `{type:"calibration",cpr:[n1,n2,n3]}` và lệnh `RESET_ODOMETRY`. Telemetry có `distance_m`, `target_error_m`, `imu_healthy`, `imu_calibrated`, `encoder_calibrated`, `cpr`, `ticks`, `fault`.

Serial 115200, mỗi lệnh kết thúc bằng xuống dòng:

| Lệnh | Ý nghĩa |
|---|---|
| `V vx vy wz` | Vận tốc toàn cục, m/s và rad/s |
| `B vx vy wz` | Vận tốc thân xe |
| `P x y yaw_deg` | Đích toàn cục và góc theo độ |
| `S` | Dừng |
| `?` | Vị trí, tốc độ bánh, ticks, MPU và lỗi |
| `C cpr1 cpr2 cpr3` | Xác nhận ba CPR đã đo, chỉ khi IDLE và đứng yên |
| `Z` | Đặt gốc và quãng đường về 0, chỉ khi IDLE và đứng yên |
| `T wheel pwm ms` | Thử một bánh treo tự do, bánh 1–3, PWM ±1–160, thời gian 50–1500 ms; chỉ từ STOP/IDLE |
| `H` | Heartbeat cho T, gửi dưới 200 ms; không kéo dài thời hạn bài thử |

## Kiểm thử

```powershell
python firmware/tests/run_host_tests.py
python tests/test_imu_driver.py
pio run -d firmware
```

Kiểm thử host biên dịch module C++ thật: ví dụ trong slide, đổi hệ tọa độ, giới hạn tốc độ, PID có dấu, anti-windup, bám đoạn với mô hình trễ/nhiễu. Kiểm thử MPU dùng I2C giả để kiểm tra đơn vị, lỗi đọc và hiệu chuẩn. Chúng không thay thế đo CPR, chiều motor/encoder, quãng đường và đáp ứng PID trên xe thật.

Kiểm thử motor/encoder bằng nguồn thật đã xác nhận dấu `ENC_SIGN={-1,-1,-1}`. Xem [kết quả chạy hai chiều và watchdog](docs/ENCODER_BENCH_2026-11-16.md). Chế độ T không xác nhận CPR và không mở khóa điều khiển quỹ đạo.
