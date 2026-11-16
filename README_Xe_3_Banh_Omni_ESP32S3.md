# Xe 3 bánh Omni – ESP32-S3

## 1. Giới thiệu

Xe dùng ba động cơ DC với encoder, bố trí bánh cách nhau 120°. ESP32-S3 điều khiển tốc độ từng bánh bằng PID, tính động học thuận/nghịch và ước lượng vị trí **(x, y, φ)** từ encoder. Xem [đặc tả firmware](README_FIRMWARE.md) để biết công thức và tham số điều khiển.

## 2. Sơ đồ chân ESP32-S3

Bảng này khớp với [pins.h](firmware/include/pins.h), dùng cho ESP32-S3 + 3× L298N + 3× GM25 encoder + MPU9250. PWM nối vào chân ENA của từng L298N. IN1 và IN2 đặt chiều motor.

| Bánh | Góc bố trí | PWM | IN1 | IN2 | Encoder A | Encoder B |
|---|---:|---:|---:|---:|---:|---:|
| 1 | 0° | GPIO 1 | GPIO 2 | GPIO 3 | GPIO 4 | GPIO 5 |
| 2 | +120° | GPIO 10 | GPIO 11 | GPIO 12 | GPIO 6 | GPIO 7 |
| 3 | −120° | GPIO 13 | GPIO 14 | GPIO 21 | GPIO 17 | GPIO 18 |

| MPU9250 — I2C | Cấu hình |
|---|---|
| SDA | GPIO 8 |
| SCL | GPIO 9 |
| Địa chỉ | `0x68` |

Firmware đọc MPU6500/9250/9255 qua I2C, tự hiệu chuẩn bias gyro khi xe đứng yên khoảng 3,3 giây. Lắp cảm biến gần nằm ngang, Z hướng lên hoặc xuống; firmware tự suy ra dấu Z từ gia tốc tĩnh. Gyro hỗ trợ ước lượng góc, encoder cung cấp tịnh tiến. Chưa dùng magnetometer.

| Chiều đặt | IN1 | IN2 |
|---|---|---|
| Dương | HIGH | LOW |
| Âm | LOW | HIGH |
| Dừng | LOW | LOW |

## 3. Nguồn và thông số xe

- Động cơ dùng nguồn riêng 12 V qua driver. Nối GND của ESP32-S3, driver và nguồn động lực. Không cấp nguồn động cơ từ chân 3,3 V hoặc 5 V của ESP32-S3.
- Bánh Omni đường kính **58 mm**, bán kính **0,029 m**. Khoảng cách từ tâm xe đến tâm bánh là **0,12 m**.
- Giới hạn tốc độ đặt **50 rpm ≈ 5,236 rad/s**.

## 4. Encoder

Firmware đếm cạnh **RISING** của kênh A và dùng kênh B để xác định chiều. Số đếm cần cấu hình là số cạnh lên A trên **một vòng trục ra/bánh**, đã tính qua hộp số. Thông số PPR từ datasheet có thể dùng quy ước cạnh khác.

[config.h](firmware/include/config.h) dùng CPR danh định **2028 = 26 × 78 tick/vòng** theo thông số nhà bán hàng do người dùng cung cấp: 26 chu kỳ/kênh/vòng motor, hộp số 1:78. Firmware đếm A-RISING 1x; không nhân thêm 2 hoặc 4. Cần kiểm chứng số vòng thực tế để xác nhận quy ước xung và độ chính xác.

1. Nâng bánh khỏi mặt đất. Giữ nguồn logic và encoder, ngắt cấp nguồn động cơ nếu có thể xoay bánh bằng tay.
2. Gửi **?** qua Serial 115200 và ghi ba số sau trường **ticks**.
3. Đánh dấu bánh rồi xoay một số vòng hoàn chỉnh theo chiều dương của mô hình. Gửi **?** lần nữa. Lấy trị tuyệt đối của chênh lệch số đếm, chia cho số vòng đã quay. Lặp lại theo chiều ngược để kiểm tra.
4. Nhập ba CPR đã đo trên trang web hoặc gửi **C cpr1 cpr2 cpr3** qua Serial (chỉ khi IDLE và đứng yên). Các giá trị này nằm trong RAM, cần nhập lại sau khi khởi động. **ENCODER_CALIBRATED_DEFAULT=true** hiện chấp nhận CPR danh định 2028; cập nhật lại nếu đo được giá trị khác. Đặt **ENC_SIGN** của bánh là **+1** nếu đếm thô tăng theo chiều dương, hoặc **−1** nếu đếm thô giảm theo chiều dương.
5. Kiểm tra thêm chiều motor so với chiều encoder. Khi motor được điều khiển theo chiều dương, tốc độ bánh đo được cũng phải dương.

Cách đo bằng trình duyệt: kết nối Wi-Fi **OMNI3_ESP32S3** (mật khẩu **OmniRobot123**), mở **http://192.168.4.1**, bấm STOP rồi **Ghi số đếm ban đầu**. Đánh dấu và quay mỗi bánh đúng số vòng đã nhập (mặc định 5), để bánh đứng yên, bấm **Tính CPR** rồi **Xác nhận CPR đã đo**. Phần mềm tính `|ticks_sau − ticks_trước| / số_vòng`. Phải có mốc vòng quay cơ học: không thể suy ra CPR chỉ từ encoder chưa hiệu chuẩn. Không dùng lại giả định cũ 4192 hoặc giá trị nhập thử 1000.

## 5. Thử chuyển động

Kiểm tra lần lượt nguồn, GND, từng motor, hai kênh encoder và chiều đếm trước khi đặt xe xuống. Dùng lệnh Serial **?** để đọc tốc độ bánh và odometry. Sau khi hiệu chuẩn encoder, thử các lệnh ở tốc độ thấp.

| Lệnh | Chuyển động mong đợi |
|---|---|
| **B 0.05 0 0** | Dịch dọc trục **x** của thân xe |
| **B 0 0.05 0** | Dịch dọc trục **y** của thân xe |
| **B 0 0 0.3** | Quay tại chỗ |
| **S** | Tắt PWM |

Làm mới lệnh **B** trong khoảng dưới 200 ms nếu muốn tiếp tục chuyển động. Đọc [quy trình nạp firmware và thử MANUAL/AUTO](streamlit_app/README.md) để kiểm tra đường điều khiển qua Wi-Fi.

Nếu hộp số không cho xoay bánh bằng tay, dùng [chế độ thử có cấp điện T](docs/ENCODER_BENCH_2026-11-16.md) với bánh nâng tự do, đánh dấu và quan sát số vòng thực tế. Không ép quay tay hoặc suy CPR từ tốc độ danh định. Cả ba dấu encoder đã được kiểm chứng bằng motor và cập nhật thành −1; CPR vẫn chưa đo.
