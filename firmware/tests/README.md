# Kiểm thử trên máy tính

Từ thư mục gốc dự án:

```powershell
python firmware/tests/run_host_tests.py
```

Cần Python và `g++` (hoặc đặt biến `CXX`). Runner biên dịch trực tiếp các tệp thật `kinematics.cpp`, `tracking.cpp`, `pid_speed.cpp` cùng `config.h` hiện hành. Chỉ API Arduino tối thiểu và đầu ra motor được thay thế; chương trình không kết nối hoặc điều khiển xe.

Các kiểm thử đối chiếu ví dụ số ở slide 14–17, đổi hệ tọa độ, giới hạn bánh, PID với encoder quay ngược và tích phân bão hòa. Mô hình motor có vùng chết PWM, trễ bậc nhất và tốc độ cực đại khác ước lượng feedforward để kiểm tra khả năng giảm sai số tốc độ. Mô phỏng bám đoạn thẳng dùng cả đáp ứng tức thời và đáp ứng trễ bậc nhất, chuyển nhiều đoạn, nhiễu vị trí, quay qua biên ±π, điều kiện dừng ổn định và trường hợp xe bị kẹt.

Đây là kiểm thử thuật toán với mô hình giả lập. Kết quả không xác nhận độ chính xác thực tế của encoder, MPU, PID hoặc quãng đường trên sàn.
