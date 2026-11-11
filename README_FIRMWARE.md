# Đặc tả firmware xe Omni 3 bánh

Firmware trong [firmware/](firmware/) chạy trên ESP32-S3 với Arduino và PlatformIO. Xem [sơ đồ chân và hiệu chuẩn encoder](README_Xe_3_Banh_Omni_ESP32S3.md), [cách nạp và điều khiển từ Streamlit](streamlit_app/README.md) và [đặc tả Wi-Fi](idea.json).

## 1. Thông số

Các giá trị điều khiển nằm trong [config.h](firmware/include/config.h). [pins.h](firmware/include/pins.h) ánh xạ PWM, IN1, IN2 và hai kênh encoder của mỗi bánh.

| Thông số | Giá trị |
|---|---:|
| Bán kính bánh **r** | 0,029 m |
| Khoảng cách tâm xe đến tâm bánh **L** | 0,12 m |
| Góc bánh **α** | 0°, +120°, −120° |
| Tốc độ đặt tối đa của bánh | 50 rpm ≈ 5,236 rad/s |
| PID tốc độ (**Kp**, **Ki**, **Kd**) | 150 / 0,001 / 0,0001 |
| PWM | 0…255, chiều qua IN1/IN2 |
| Hệ số bám vị trí **λ** | 0,6 |
| Chu kỳ điều khiển danh định | 10 ms (100 Hz) |
| Dung sai vị trí, góc | 0,01 m, 0,06 rad |
| Ngưỡng tốc độ đặt của bánh | 0,01 vòng/s |

Dung sai vị trí và góc là giá trị khởi đầu để bộ điều khiển có thể kết thúc gần đích. Dưới ngưỡng tốc độ đặt, PID tắt motor. Với lệnh tịnh tiến thuần, sai số nhỏ hơn khoảng 3,5 mm có thể làm cả ba tốc độ đặt lọt dưới ngưỡng này. Cần kiểm tra khả năng thắng ma sát và độ chính xác bám điểm trên xe thật trước khi chỉnh dung sai.

Số đếm encoder mặc định là **4192 cạnh lên A/vòng bánh**, lấy từ giả định 32 × 131 và cần đo lại. Xem [quy trình hiệu chuẩn](README_Xe_3_Banh_Omni_ESP32S3.md#4-encoder). Nếu cấu hình 4192 nhưng thực tế chỉ có 330 count/vòng, tốc độ và độ dịch chuyển ước lượng sẽ thấp hơn thực tế khoảng 12,7 lần.

## 2. Động học

Với góc hướng xe **φ** và vận tốc theo hệ tọa độ toàn cục **(ẋ, ẏ, φ̇)**, tốc độ góc mỗi bánh là

~~~text
ω_i = [-sin(φ + α_i) × ẋ + cos(φ + α_i) × ẏ + L × φ̇] / r
~~~

Với lệnh **(vx, vy, wz)** theo thân xe, dùng cùng công thức với **φ = 0**. Chuyển vận tốc toàn cục sang hệ tọa độ thân xe theo phép quay sau.

~~~text
vx =  cos(φ) × ẋ + sin(φ) × ẏ
vy = -sin(φ) × ẋ + cos(φ) × ẏ
wz = φ̇
~~~

Nếu tốc độ đặt lớn nhất vượt giới hạn, nhân cả ba tốc độ bánh cùng một hệ số **ω_max / max(|ω_i|)** để giữ hướng chuyển động.

Động học thuận sử dụng tốc độ góc đo có dấu.

~~~text
ẋ = (2r/3) × Σ[-sin(φ + α_i) × ω_i]
ẏ = (2r/3) × Σ[ cos(φ + α_i) × ω_i]
φ̇ = (r/3L) × Σω_i
~~~

Mỗi vòng điều khiển tích phân các giá trị này theo thời gian thực giữa hai lần lấy mẫu. Odometry bắt đầu từ **(0, 0, 0)** khi khởi động. Sai số có thể tích lũy do trượt bánh, sai số hình học và hiệu chuẩn encoder.

## 3. PID và encoder

Encoder đếm cạnh lên trên A và đọc B để xác định dấu. **N** là số cạnh lên A đo được trên một vòng bánh. **Δn** là số đếm có dấu trong khoảng lấy mẫu **Δt**.

~~~text
tốc độ [vòng/s] = Δn / (N × Δt)
ω [rad/s]       = 2π × tốc độ [vòng/s]
ω [rad/s]       = 2π × rpm / 60
~~~

PID mỗi bánh dùng sai số **|ω_đặt / (2π)| − |tốc_độ_đo|** theo vòng/s. Hệ số PID lấy chu kỳ danh định 10 ms làm chuẩn. Mỗi bước, phần tích phân nhân với **Δt / 10 ms** và phần vi phân chia cho tỷ lệ này. Chiều motor lấy từ dấu của tốc độ đặt. PID giới hạn PWM và bỏ phần tích phân mới ở bước bị bão hòa. Khi tốc độ đặt lọt dưới ngưỡng, firmware tắt PWM và xóa trạng thái PID của bánh đó.

Vòng điều khiển chạy trong task riêng, lấy mẫu danh định mỗi 10 ms. Serial, HTTP và WebSocket được xử lý ngoài task này. Encoder tiếp tục cập nhật odometry trong lúc bánh còn quay sau lệnh dừng.

## 4. Chế độ và watchdog

| Chế độ | Cách chạy | Điều kiện dừng |
|---|---|---|
| MANUAL | Chọn chế độ, gửi START, rồi gửi joystick theo thân xe. | Quá 300 ms mất joystick trong khi đang có lệnh di chuyển thì dừng và cần START lại. |
| AUTO | Nạp quỹ đạo, chọn AUTO, rồi gửi START. Xe lần lượt đến các điểm **x, y** theo odometry. | Hoàn thành quỹ đạo hoặc nhận STOP. Xe tiếp tục bám quỹ đạo khi mất Wi-Fi. |
| V / B | Serial đặt vận tốc toàn cục / theo thân xe. | Quá 200 ms không làm mới lệnh hoặc nhận lệnh dừng. |
| P | Serial đặt **x, y, yaw** và bật vòng bám vị trí, góc. | Đạt dung sai hoặc quá 200 ms không làm mới lệnh. |

Chọn chế độ qua WebSocket đưa xe về IDLE. STOP kết thúc lệnh đang chạy và đưa xe về STOP/IDLE. Gói joystick chỉ điều khiển khi MANUAL đã được START. Trong MANUAL và AUTO, PAUSE ngắt motor. RESUME tiếp tục AUTO từ điểm đang theo hoặc cho phép giữ joystick lại trong MANUAL.

AUTO sử dụng **ẋ = λ(x_đích − x)**, **ẏ = λ(y_đích − y)** và **φ̇ = 0**. POSE dùng cùng công thức và thêm **φ̇ = λ × sai số góc đã chuẩn hóa**. Quỹ đạo chứa tối đa 64 điểm, lưu trong RAM và mất khi ESP32 khởi động lại.

## 5. Lệnh Serial

Baud **115200**, mỗi lệnh kết thúc bằng xuống dòng.

| Lệnh | Ý nghĩa |
|---|---|
| **V &lt;vx&gt; &lt;vy&gt; &lt;wz&gt;** | Vận tốc toàn cục, m/s và rad/s |
| **B &lt;vx&gt; &lt;vy&gt; &lt;wz&gt;** | Vận tốc theo thân xe, m/s và rad/s |
| **P &lt;x&gt; &lt;y&gt; &lt;yaw_deg&gt;** | Đích toàn cục, m và độ |
| **S** | Dừng và xóa lệnh đang chạy |
| **?** | In vị trí, góc, tốc độ bánh và bộ đếm encoder tích lũy |

~~~text
B 0.05 0 0
P 0.5 0.2 30
?
S
~~~

V, B và P yêu cầu đủ ba số hữu hạn. V và B chấp nhận mỗi thành phần tịnh tiến trong **±1 m/s** và tốc độ quay trong **±10 rad/s**. P chấp nhận mỗi tọa độ trong **±1000 m** và góc trong **±36000°**. Firmware giới hạn tốc độ bánh sau khi tính động học nghịch.

Một lệnh V, B hoặc P hợp lệ bắt đầu điều khiển ngay. Để duy trì chuyển động, phần mềm gửi lệnh phải gửi lại lệnh tương ứng với khoảng cách giữa hai lần gửi nhỏ hơn 200 ms. Lệnh **?** chỉ đọc trạng thái. Ba giá trị sau trường **ticks** là số đếm encoder thô, có dấu, tích lũy từ lúc khởi động.

## 6. Module

| Module | Trách nhiệm |
|---|---|
| main.cpp | Khởi tạo, lập lịch task điều khiển, xử lý I/O và phát telemetry |
| control.cpp | Chế độ, watchdog, theo điểm, cập nhật odometry |
| kinematics.cpp | Động học thuận/nghịch và giới hạn tốc độ bánh |
| pid_speed.cpp | PID tốc độ ba bánh |
| motors.cpp / encoders.cpp | Điều khiển driver và lấy mẫu encoder |
| serial_cmd.cpp / net_server.cpp | Lệnh Serial, HTTP, WebSocket |
