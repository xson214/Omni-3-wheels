# Kiểm thử motor và encoder ngày 16/11/2026

Thực hiện sau khi người dùng xác nhận ba bánh quay tự do và nguồn 12 V đã bật. Firmware được nạp qua COM16. Mỗi lần chỉ điều khiển một bánh, PWM giới hạn 160/255, thời gian tối đa 1.500 ms, heartbeat phải được làm mới trong 200 ms.

## Kết quả đo cuối sau khi sửa dấu

Mỗi nhịp cấp PWM ±140 trong 600 ms; số đếm cuối bao gồm quán tính quay sau khi tắt PWM. Đây là **số xung của nhịp thử**, không phải CPR.

| Bánh | Δticks thô khi PWM +140 | Δticks thô khi PWM −140 | Tốc độ có dấu sau ENC_SIGN | Các encoder khác |
|---|---:|---:|---|---|
| 1 | −149 | +148 | Khớp dấu lệnh cả hai chiều | Không đổi |
| 2 | −171 | +166 | Khớp dấu lệnh cả hai chiều | Không đổi |
| 3 | −137 | +147 | Khớp dấu lệnh cả hai chiều | Không đổi |

Cả ba encoder phản hồi đúng kênh và đổi dấu khi motor đảo chiều. Đã sửa `ENC_SIGN={-1,-1,-1}` trong `firmware/include/config.h` và nạp lại. `MOTOR_SIGN={1,1,1}` giữ nguyên. Kết quả chứng minh sự đồng nhất motor/encoder theo lệnh điện; chưa chứng minh thứ tự bố trí bánh trên khung đúng ma trận cơ học.

Bài thử mất heartbeat: yêu cầu bánh 1 PWM140 trong 1.000 ms nhưng không gửi H. Firmware dừng sớm, ghi −43 tick. Quan sát trạng thái qua Serial cho thấy RUNNING chuyển STOP/IDLE sau khoảng 0,25 s giữa hai mẫu; watchdog thiết kế là 200 ms và việc quan sát có độ trễ lấy mẫu. Lỗi trả về `Motor test heartbeat timeout`. Không tiếp tục cấp PWM đến hết 1.000 ms.

Dữ liệu đầy đủ: [encoder_verified.json](encoder_verified.json). Các phép thử ban đầu ở PWM80/110 không thấy xung; chưa đủ cơ sở kết luận ngưỡng khởi động vì trạng thái nguồn động lực chỉ được người dùng xác nhận sau đó. Không dùng kết quả đó để đặt deadband motor.

## Thay đổi phần mềm và kiểm tra

- Thêm chế độ `MOTOR_TEST` chỉ qua Serial: `T <bánh 1..3> <PWM có dấu> <ms>`.
- `H` làm mới watchdog, không kéo dài thời hạn cứng. Chỉ nhận T khi STOP/IDLE; T không thể tự gia hạn một bài thử đang chạy.
- `S` dừng ngay. PWM tối đa160, thời gian50–1500ms. Bài thử không xác nhận CPR, không mở khóa MANUAL/AUTO.
- Script `tools/encoder_bench.py` chạy một nhịp có thời hạn, ghi raw ticks/trạng thái/tốc độ, kiểm tra nhầm kênh, gửi STOP trong finally.
- `python tests/test_motor_test_control.py`: PASS, kiểm tra giới hạn, chỉ một bánh chạy, hai chiều, heartbeat, thời hạn cứng, STOP, overrun, tràn millis và khóa hiệu chuẩn.
- `python firmware/tests/run_host_tests.py`: PASS.
- Build/upload bản sửa dấu: SUCCESS, esptool xác minh hash.

Ví dụ chạy lại khi bánh đã được nâng tự do:

```powershell
python tools/encoder_bench.py --wheel 1 --pwm 140 --duration 600 --output docs/encoder_repeat.json
```

## Giới hạn phép thử

Chưa có mốc vòng quay nên chưa xác định được CPR, rpm thật hoặc quãng đường. Không suy CPR từ tốc độ danh định 50 rpm hoặc thời gian cấp PWM vì tốc độ thực phụ thuộc điện áp, tải và quá trình tăng/giảm tốc. Giữ `ENCODER_CALIBRATED_DEFAULT=false`.

Với hộp số không thể xoay tay, đo CPR bằng cách đánh dấu bánh và quan sát số vòng khi motor được cấp điện chạy chậm, hoặc thêm cảm biến chỉ thị một xung/vòng. Lấy chênh lệch ticks chia số vòng **đã quan sát**, không chia thời gian. Có thể dùng các nhịp T ngắn, giữ nguyên phiên ESP32 và cộng các vòng/chênh lệch ticks. Độ chính xác quỹ đạo trên sàn và tuning PID có tải vẫn cần phép đo riêng.
