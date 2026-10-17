# XE 3 BÁNH OMNI – ESP32-S3
## Test động học thuận / nghịch và điều khiển tốc độ bánh xe

## 1. Giới thiệu

Code sử dụng ESP32-S3 để điều khiển xe robot 3 bánh Omni:

- Điều khiển 3 động cơ DC.
- Đọc encoder của từng động cơ.
- Điều khiển tốc độ bánh bằng bộ PI.
- Tính động học nghịch: vận tốc xe → vận tốc góc bánh.
- Tính động học thuận: vận tốc góc bánh → vận tốc xe.
- Tích phân vận tốc để ước lượng vị trí `(x, y, phi)`.

---

## 2. Sơ đồ chân ESP32-S3

| Bánh | PWM | IN1 | IN2 | Encoder A | Encoder B |
|---|---:|---:|---:|---:|---:|
| Bánh 1 | GPIO 4 | GPIO 5 | GPIO 6 | GPIO 9 | GPIO 10 |
| Bánh 2 | GPIO 7 | GPIO 15 | GPIO 16 | GPIO 11 | GPIO 12 |
| Bánh 3 | GPIO 17 | GPIO 18 | GPIO 8 | GPIO 13 | GPIO 14 |

### Bánh 1

```text
ESP32-S3             DRIVER 1             MOTOR 1
---------             --------             -------
GPIO 4   ---------->  ENA1
GPIO 5   ---------->  IN1
GPIO 6   ---------->  IN2
GPIO 9   <----------  Encoder A
GPIO 10  <----------  Encoder B
```

### Bánh 2

```text
ESP32-S3             DRIVER 2             MOTOR 2
---------             --------             -------
GPIO 7   ---------->  ENA2
GPIO 15  ---------->  IN1
GPIO 16  ---------->  IN2
GPIO 11  <----------  Encoder A
GPIO 12  <----------  Encoder B
```

### Bánh 3

```text
ESP32-S3             DRIVER 3             MOTOR 3
---------             --------             -------
GPIO 17  ---------->  ENA3
GPIO 18  ---------->  IN1
GPIO 8   ---------->  IN2
GPIO 13  <----------  Encoder A
GPIO 14  <----------  Encoder B
```

---

## 3. Sơ đồ kết nối tổng thể

```text
                         ┌──────────────────┐
                         │     ESP32-S3     │
                         │                  │
                         │ GPIO 4  ────────┼──── PWM Driver 1
                         │ GPIO 5  ────────┼──── IN1  Driver 1
                         │ GPIO 6  ────────┼──── IN2  Driver 1
                         │ GPIO 9  ◄──────────── Encoder 1 A
                         │ GPIO 10 ◄──────────── Encoder 1 B
                         │                  │
                         │ GPIO 7  ────────┼──── PWM Driver 2
                         │ GPIO 15 ────────┼──── IN1  Driver 2
                         │ GPIO 16 ────────┼──── IN2  Driver 2
                         │ GPIO 11 ◄──────────── Encoder 2 A
                         │ GPIO 12 ◄──────────── Encoder 2 B
                         │                  │
                         │ GPIO 17 ────────┼──── PWM Driver 3
                         │ GPIO 18 ────────┼──── IN1  Driver 3
                         │ GPIO 8  ────────┼──── IN2  Driver 3
                         │ GPIO 13 ◄──────────── Encoder 3 A
                         │ GPIO 14 ◄──────────── Encoder 3 B
                         └──────────────────┘
                                  │
                                  │ GND chung
                                  ▼
                         ┌──────────────────┐
                         │  Nguồn động lực  │
                         │      12 V DC     │
                         └──────────────────┘
```

**Lưu ý:** GND của ESP32-S3, các driver và nguồn điều khiển phải được nối chung.

---

## 4. Thông số xe

| Thông số | Giá trị |
|---|---:|
| Bán kính bánh `R_WHEEL` | 0.029 m |
| Khoảng cách tâm xe đến bánh `L_BODY` | 0.12 m |
| Tốc độ bánh tối đa `W_MAX` | 5.236 rad/s |
| Đường kính bánh | 58 mm |
| Tốc độ động cơ | 50 rpm |

Chuyển đổi:

```text
50 rpm = 50 × 2π / 60 ≈ 5.236 rad/s
```

---

## 5. Góc bố trí 3 bánh

```cpp
const float ALPHA[3] = {
    0.0f,
    2.0f * PI / 3.0f,
   -2.0f * PI / 3.0f
};
```

| Bánh | ALPHA | Góc |
|---|---:|---:|
| Bánh 1 | 0 | 0° |
| Bánh 2 | +2π/3 | +120° |
| Bánh 3 | -2π/3 | -120° |

Ba bánh được bố trí cách nhau 120° quanh tâm xe.

---

## 6. Encoder

Thông số encoder:

```text
ENC_PPR = 11 xung/vòng
GEAR = 30
```

Do code chỉ đếm cạnh RISING của kênh A:

```text
COUNTS_PER_REV = 11 × 30 = 330 count/vòng
```

Tốc độ góc được tính:

```text
ω = (Δcount / COUNTS_PER_REV) × 2π / Δt
```

---

## 7. Quy ước chiều encoder

```cpp
const int SIGN_W[3] = {1, 1, 1};
```

- `+1`: count tăng tương ứng với omega dương.
- `-1`: đảo dấu omega của bánh tương ứng.

Nếu một bánh quay ngược chiều so với quy ước, có thể thay đổi ví dụ:

```cpp
const int SIGN_W[3] = {1, 1, -1};
```

---

## 8. Động học nghịch

Hàm:

```cpp
inverseKin(xd, yd, phid, phi, w);
```

Công thức:

```text
w_i =
[-xd sin(φ + α_i)
 +yd cos(φ + α_i)
 +L phid] / R
```

Ý nghĩa:

```text
[ẋ, ẏ, φ̇]
      ↓
Động học nghịch
      ↓
[ω1, ω2, ω3]
```

---

## 9. Động học thuận

Hàm:

```cpp
forwardKin(w, phi, xd, yd, phid);
```

Công thức trong code:

```text
ẋ = 2R/3 × Σ[-sin(φ + α_i)ω_i]

ẏ = 2R/3 × Σ[ cos(φ + α_i)ω_i]

φ̇ = R/(3L) × Σω_i
```

Ý nghĩa:

```text
[ω1, ω2, ω3]
      ↓
Động học thuận
      ↓
[ẋ, ẏ, φ̇]
```

---

## 10. Điều khiển PI tốc độ

Thông số:

```cpp
KP = 20
KI = 60
TS_MS = 50
```

Chu kỳ điều khiển:

```text
Ts = 50 ms = 0.05 s = 20 Hz
```

Sai số:

```text
e = ω_ref - ω_meas
```

Tín hiệu điều khiển:

```text
u = Feedforward + KP × e + KI × Integral(e)
```

PWM được giới hạn:

```text
-255 ≤ PWM ≤ 255
```

---

## 11. Giới hạn tốc độ bánh

```cpp
W_MAX = 5.236 rad/s
```

Nếu một bánh vượt giới hạn, cả 3 omega được nhân cùng một hệ số:

```text
k = W_MAX / max(|ω1|, |ω2|, |ω3|)
```

Mục đích là giữ nguyên hướng chuyển động và chỉ giảm độ lớn vận tốc.

---

## 12. Thời gian chạy test

```cpp
T_TEST = 5.0 s
T_SETTLE = 1.5 s
```

Mỗi bài test chạy 5 giây.

- `0 → 1.5 s`: giai đoạn quá độ, không tính trung bình.
- `1.5 → 5 s`: dùng để tính tốc độ bánh trung bình.

---

## 13. Các bài test động học nghịch

### Test nghịch 1

```cpp
testInverse(1, 0.10f, 0.05f, 0.5f, 0.0f);
```

```text
xd   = 0.10 m/s
yd   = 0.05 m/s
phid = 0.5 rad/s
phi  = 0°
```

### Test nghịch 2

```cpp
testInverse(2, -0.08f, 0.08f, -0.2f, 90.0f);
```

```text
xd   = -0.08 m/s
yd   =  0.08 m/s
phid = -0.2 rad/s
phi  = 90°
```

### Test nghịch 3

```cpp
testInverse(3, -0.10f, 0.05f, 0.3f, 180.0f);
```

```text
xd   = -0.10 m/s
yd   =  0.05 m/s
phid =  0.3 rad/s
phi  = 180°
```

---

## 14. Các bài test động học thuận

### Test thuận 1

```cpp
testForward(1, 4.0f, 2.0f, 3.0f, 0.0f);
```

```text
ω1 = 4.0 rad/s
ω2 = 2.0 rad/s
ω3 = 3.0 rad/s
phi = 0°
```

### Test thuận 2

```cpp
testForward(2, -2.0f, 5.0f, 1.0f, 90.0f);
```

```text
ω1 = -2.0 rad/s
ω2 =  5.0 rad/s
ω3 =  1.0 rad/s
phi = 90°
```

### Test thuận 3

```cpp
testForward(3, 1.5f, -3.0f, 4.5f, 180.0f);
```

```text
ω1 = 1.5 rad/s
ω2 = -3.0 rad/s
ω3 = 4.5 rad/s
phi = 180°
```

---

## 15. Serial Monitor

Thông số:

```text
Baud rate = 115200
```

Serial Monitor hiển thị:

```text
w dat
w do TB
Thuan(w do)
Thuan(w dat)
Vi tri sau 5 s
```

Trong đó:

- `w dat`: tốc độ góc đặt.
- `w do TB`: tốc độ góc đo trung bình từ encoder.
- `Thuan(w do)`: vận tốc robot tính từ omega đo được.
- `Thuan(w dat)`: vận tốc robot lý thuyết từ omega đặt.
- `Vi tri sau 5 s`: vị trí ước lượng sau bài test.

---

## 16. Nguồn điện

Không cấp trực tiếp nguồn động cơ từ chân 3.3V hoặc 5V của ESP32-S3.

Sử dụng nguồn riêng cho động cơ:

```text
Nguồn 12V
   │
   ├── Driver 1 ── Motor 1
   ├── Driver 2 ── Motor 2
   └── Driver 3 ── Motor 3
```

ESP32-S3 cung cấp tín hiệu:

```text
PWM
IN1
IN2
```

Các GND phải nối chung:

```text
ESP32-S3 GND
      │
      ├── Driver 1 GND
      ├── Driver 2 GND
      ├── Driver 3 GND
      └── GND nguồn điều khiển
```

---

## 17. Thứ tự kiểm tra

Khuyến nghị:

```text
1. Kiểm tra nguồn
       ↓
2. Kiểm tra GND chung
       ↓
3. Kiểm tra chiều quay từng motor
       ↓
4. Kiểm tra encoder A/B
       ↓
5. Kiểm tra số count encoder
       ↓
6. Kiểm tra tốc độ omega đo được
       ↓
7. Chạy PI từng bánh
       ↓
8. Kiểm tra động học nghịch
       ↓
9. Kiểm tra động học thuận
       ↓
10. Chạy đồng thời cả 3 bánh
```

---

## 18. Tóm tắt nhanh

```text
             XE 3 BÁNH OMNI
                  │
        ┌─────────┼─────────┐
        │         │         │
      Motor 1   Motor 2   Motor 3
        │         │         │
     GPIO 4    GPIO 7    GPIO 17     ← PWM
     GPIO 5    GPIO 15   GPIO 18      ← IN1
     GPIO 6    GPIO 16   GPIO 8       ← IN2
        │         │         │
     Encoder   Encoder   Encoder
     A=9       A=11      A=13
     B=10      B=12      B=14

R = 0.029 m
L = 0.12 m
Wmax = 5.236 rad/s
Encoder = 11 PPR
Gear = 30:1
Counts/rev = 330
PI: KP=20, KI=60
Ts = 50 ms
Baud = 115200
```
