# Xe 3 bánh Omni – ESP32-S3

## 1. Giới thiệu

Xe dùng ba động cơ DC với encoder, bố trí bánh cách nhau 120°. ESP32-S3 điều khiển tốc độ từng bánh bằng PID, tính động học thuận/nghịch và ước lượng vị trí **(x, y, φ)** từ encoder. Xem [đặc tả firmware](README_FIRMWARE.md) để biết công thức và tham số điều khiển.

## 2. Sơ đồ chân ESP32-S3

Bảng này khớp với [pins.h](firmware/include/pins.h). PWM nối vào chân enable của kênh driver tương ứng. IN1 và IN2 đặt chiều motor.

| Bánh | Góc bố trí | PWM | IN1 | IN2 | Encoder A | Encoder B |
|---|---:|---:|---:|---:|---:|---:|
| 1 | 0° | GPIO 4 | GPIO 5 | GPIO 6 | GPIO 9 | GPIO 10 |
| 2 | +120° | GPIO 7 | GPIO 15 | GPIO 16 | GPIO 11 | GPIO 12 |
| 3 | −120° | GPIO 17 | GPIO 18 | GPIO 8 | GPIO 13 | GPIO 14 |

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

[config.h](firmware/include/config.h) hiện dùng **4192 tick/vòng** theo giả định **32 × 131**. Giá trị **11 × 30 = 330** trong bản tài liệu trước chưa được xác nhận cho xe này. Hãy đo trực tiếp trên từng bánh trước khi chạy PID và odometry.

1. Nâng bánh khỏi mặt đất. Giữ nguồn logic và encoder, ngắt cấp nguồn động cơ nếu có thể xoay bánh bằng tay.
2. Gửi **?** qua Serial 115200 và ghi ba số sau trường **ticks**.
3. Đánh dấu bánh rồi xoay một số vòng hoàn chỉnh theo chiều dương của mô hình. Gửi **?** lần nữa. Lấy trị tuyệt đối của chênh lệch số đếm, chia cho số vòng đã quay. Lặp lại theo chiều ngược để kiểm tra.
4. Cập nhật **TICKS_PER_WHEEL_REV** theo số đếm đo được. Đặt **ENC_SIGN** của bánh là **+1** nếu đếm thô tăng theo chiều dương, hoặc **−1** nếu đếm thô giảm theo chiều dương.
5. Kiểm tra thêm chiều motor so với chiều encoder. Khi motor được điều khiển theo chiều dương, tốc độ bánh đo được cũng phải dương.

## 5. Thử chuyển động

Kiểm tra lần lượt nguồn, GND, từng motor, hai kênh encoder và chiều đếm trước khi đặt xe xuống. Dùng lệnh Serial **?** để đọc tốc độ bánh và odometry. Sau khi hiệu chuẩn encoder, thử các lệnh ở tốc độ thấp.

| Lệnh | Chuyển động mong đợi |
|---|---|
| **B 0.05 0 0** | Dịch dọc trục **x** của thân xe |
| **B 0 0.05 0** | Dịch dọc trục **y** của thân xe |
| **B 0 0 0.3** | Quay tại chỗ |
| **S** | Tắt PWM |

Làm mới lệnh **B** trong khoảng dưới 200 ms nếu muốn tiếp tục chuyển động. Đọc [quy trình nạp firmware và thử MANUAL/AUTO](streamlit_app/README.md) để kiểm tra đường điều khiển qua Wi-Fi.
