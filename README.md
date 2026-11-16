# Xe Omni 3 bánh — ESP32-S3 + Streamlit

| Thành phần | Nội dung |
|---|---|
| [Firmware PlatformIO](firmware/) | Điều khiển ba bánh, đọc encoder và bám quỹ đạo |
| [Ứng dụng Streamlit](streamlit_app/) | Điều khiển MANUAL, tải quỹ đạo, xem telemetry, tính động học và mô phỏng đường đi |
| [Đặc tả firmware](README_FIRMWARE.md) | Động học, PID, watchdog và lệnh Serial |
| [Sơ đồ chân và hiệu chuẩn encoder](README_Xe_3_Banh_Omni_ESP32S3.md) | Đấu dây ESP32-S3 và đo số tick trên xe |
| [Giao thức Wi-Fi](idea.json) | Cấu trúc thông điệp HTTP và WebSocket |

Xem [hướng dẫn nạp firmware và chạy Streamlit](streamlit_app/README.md). Hệ số encoder mặc định cần được xác nhận bằng phép đo trước khi chạy bám điểm.

Máy tính kết nối Wi-Fi **OMNI3_ESP32S3**, mật khẩu **OmniRobot123**, rồi mở **http://192.168.4.1** để điều khiển trực tiếp (không cần Internet hoặc Streamlit). Trang web có đo CPR, đặt gốc, MANUAL, AUTO và trạng thái MPU. Giữ xe đứng yên, nằm ngang khi bật nguồn để hiệu chuẩn gyro. CPR mặc định danh định 2028 tick/vòng theo thông số 26 xung và hộp số 1:78; vẫn cần kiểm chứng bằng vòng quay thực tế.

[Báo cáo đối chiếu công thức với slide](docs/DOI_CHIEU_CONG_THUC.md) ghi cả các công thức khớp và lỗi dấu trong slide.
