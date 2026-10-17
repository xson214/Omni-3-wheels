# README — Đặc tả firmware xe Omni 3 bánh (cho Codex / AI code)

> **Mục đích file này:** Cho AI (Codex, Cursor Agent, …) đọc và **viết code thật** chạy trên MCU khi đã biết cấu hình chân pin.  
> **Không đoán chân pin** — chỉ dùng bảng ở §3 sau khi người dùng điền.  
> **Tham chiếu MATLAB:** thư mục `mophong/` đã mô phỏng đúng công thức và PID.

---

## 0. Nhiệm vụ cho AI khi được yêu cầu “code thật”

Khi user cung cấp **bảng chân pin đã điền** (§3):

1. Tạo project firmware (Arduino / PlatformIO) cho MCU đã chỉ định.
2. Implement đúng:
   - Động học nghịch / thuận (§4)
   - PID tốc độ 3 bánh (§5)
   - Driver motor PWM + DIR (§6)
   - Encoder → tốc độ (§7)
   - Vòng điều khiển (§8)
   - Giao tiếp Serial (§9)
3. Dùng đúng thông số số học §2 — **không đổi** trừ khi user bảo.
4. Không thêm ROS/SLAM/camera trừ khi được yêu cầu.
5. Output: file `.ino` / `.cpp` + `pins.h` (map từ bảng §3) + hướng dẫn nạp/chạy ngắn.

---

## 1. Tổng quan hệ thống

| Hạng mục | Giá trị |
|----------|---------|
| Robot | Omni 3 bánh, bố trí 120° |
| MCU mục tiêu (slide) | **ESP32-S3** (có thể ESP8266 / Arduino Mega nếu user chỉ định) |
| Động cơ | 3× GM25-370, 12 V, **50 rpm** (trục ra) |
| Driver | L298N / cầu H tương đương (PWM + DIR mỗi motor) |
| Bánh | Omni Ø **58 mm** → `r = 0.029 m` |
| Hình học | `L = 0.12 m` (tâm khung → tâm bánh) |
| Nguồn | 4S 18650 (~14.8–16.8 V) + LM2596 → 5 V logic |
| Cảm biến (tùy chọn sau) | HC-SR04 — **không bắt buộc** cho vòng động học/PID |

**Kiến trúc điều khiển (khớp MATLAB `sim_quy_dao_co_PID.m`):**

```text
[Lệnh] vx,vy,w  HOẶC  Pdes(x,y,θ)
        │
        ▼
 Vòng ngoài (tùy chọn): u = λ * E , E = Pdes - P
        │
        ▼
 Động học nghịch:  ω_ref[3] = (1/r) * A(φ) * [ẋ; ẏ; φ̇]
        │
        ▼
 Giới hạn |ω| ≤ ω_max (50 rpm)
        │
        ▼
 Consigne tour/s:  c_i = ω_ref_i / (2π)
        │
        ▼
 PID 3 kênh → PWM[0..255] + DIR
        │
        ▼
 Driver → Motor + Encoder
        │
        ▼
 Động học thuận (odometry): [ẋ;ẏ;φ̇] = r * A^{-1}(φ) * ω_meas
        │
        ▼
 Cập nhật pose P += dt * u_hat
```

---

## 2. Thông số số học (BẮT BUỘC dùng đúng)

```c
#define WHEEL_RADIUS_M     0.029f      // r
#define BASE_RADIUS_M      0.12f       // L
#define MOTOR_RPM_MAX      50.0f
#define OMEGA_MAX_RAD_S    5.236f      // 2*pi*50/60
#define V_WHEEL_MAX_M_S    0.15184f    // r * omega_max

#define ALPHA1_RAD         0.0f
#define ALPHA2_RAD         (2.0f * PI / 3.0f)   // +120°
#define ALPHA3_RAD         (-2.0f * PI / 3.0f)  // -120°

#define PID_KP             150.0f
#define PID_KI             0.001f
#define PID_KD             0.0001f
#define PWM_MAX            255

#define LAMBDA_POSE        0.6f        // vòng ngoài bám pose (nếu bật)
#define CTRL_DT_S          0.01f       // 100 Hz khuyến nghị
#define POSE_TOL           0.001f

// Encoder (điều chỉnh nếu encoder thật khác)
#define ENCODER_TICKS_PER_MOTOR_REV   32
#define GEAR_RATIO                    131   // nếu dùng công thức Arduino cũ
// HOẶC đo thực tế: ticks một vòng trục RA bánh (sau hộp số)
```

**Chuyển đơn vị:**

```text
ω [rad/s]  =  rpm * 2π / 60
c [tour/s] =  ω / (2π)  = rpm / 60
rpm        =  ω * 60 / (2π)
```

---

## 3. Cấu hình chân pin — ĐIỀN TRƯỚC KHI CODE

> AI: nếu ô còn `TODO` / trống → **hỏi user**, không bịa chân.

### 3.1 MCU

| Trường | Giá trị |
|--------|---------|
| Board | `TODO` (vd: ESP32-S3 / ESP8266 / Mega) |
| Framework | Arduino |
| Logic I/O | `TODO` (3.3V / 5V) |

### 3.2 Motor driver (mỗi bánh: PWM + DIR)

| Kênh | Motor | Chân PWM | Chân DIR (hoặc IN1/IN2) | Ghi chú |
|------|-------|----------|-------------------------|---------|
| M1 | Bánh 1 (α=0) | `TODO` | `TODO` | |
| M2 | Bánh 2 (+120°) | `TODO` | `TODO` | |
| M3 | Bánh 3 (−120°) | `TODO` | `TODO` | |

Quy ước DIR:

```text
DIR = HIGH  →  ω > 0  (chiều dương theo mô hình)
DIR = LOW   →  ω < 0
```

(Nếu chiều thực tế ngược: đảo logic DIR trong `set_motor()`, **không** đảo công thức IK.)

### 3.3 Encoder

| Kênh | Chân A (interrupt) | Chân B (tùy chọn quadrature) | Kéo điện trở |
|------|--------------------|------------------------------|--------------|
| E1 | `TODO` | `TODO` / không dùng | INPUT_PULLUP nếu hở |
| E2 | `TODO` | `TODO` / không dùng | |
| E3 | `TODO` | `TODO` / không dùng | |

Nếu chỉ 1 kênh (CHANGE): tốc độ = `|tick|` / dt → luôn ≥ 0; dấu lấy từ `consigne` (như firmware Arduino tham khảo).

### 3.4 Serial / debug

| Chức năng | Chân / Port | Baud |
|-----------|-------------|------|
| Debug / lệnh | USB Serial hoặc UART `TODO` | 115200 (khuyến nghị) |

### 3.5 Nguồn & an toàn

| Mục | Ghi chú |
|-----|---------|
| VMOT | 12 V (hoặc 4S qua driver) — **không** cấp 16.8 V thẳng vào chân MCU |
| VCC logic | 5 V / 3.3 V theo board |
| GND chung | MCU ↔ driver ↔ nguồn |

---

## 4. Động học (implement đúng)

### 4.1 Động học nghịch — `invkin(xd, yd, phid, phi) → ω[3]`

\[
A(\phi)=\begin{bmatrix}
-\sin\phi & \cos\phi & L\\
-\sin(\phi+\frac{2\pi}{3}) & \cos(\phi+\frac{2\pi}{3}) & L\\
-\sin(\phi-\frac{2\pi}{3}) & \cos(\phi-\frac{2\pi}{3}) & L
\end{bmatrix},\quad
\boldsymbol{\omega}=\frac{1}{r}\,A(\phi)\begin{bmatrix}\dot x\\\dot y\\\dot\phi\end{bmatrix}
\]

Pseudo-code:

```c
void invkin(float xd, float yd, float phid, float phi, float w[3]) {
  float a[3] = { phi + ALPHA1_RAD, phi + ALPHA2_RAD, phi + ALPHA3_RAD };
  for (int i = 0; i < 3; i++) {
    float vi = -sinf(a[i])*xd + cosf(a[i])*yd + BASE_RADIUS_M*phid;
    w[i] = vi / WHEEL_RADIUS_M;   // rad/s
  }
}
```

**Saturate giữ hướng** (scale đều 3 bánh):

```c
void saturate_omega(float w[3]) {
  float m = fmaxf(fmaxf(fabsf(w[0]), fabsf(w[1])), fabsf(w[2]));
  if (m > OMEGA_MAX_RAD_S) {
    float s = OMEGA_MAX_RAD_S / m;
    w[0]*=s; w[1]*=s; w[2]*=s;
  }
}
```

### 4.2 Động học thuận — `fwdkin(ω[3], phi) → (xd, yd, phid)`

\[
\begin{aligned}
\dot x&=\frac{2r}{3}\sum_{i=1}^{3}\big(-\sin(\phi+\alpha_i)\,\omega_i\big)\\
\dot y&=\frac{2r}{3}\sum_{i=1}^{3}\big(\cos(\phi+\alpha_i)\,\omega_i\big)\\
\dot\phi&=\frac{r}{3L}(\omega_1+\omega_2+\omega_3)
\end{aligned}
\]

### 4.3 Dạng body (lệnh cục bộ `vx, vy, ω`)

```c
// ω = (1/r) * Jb * [vx; vy; wz]
// Jb = [[ 0, 1, L], [-√3/2, -1/2, L], [√3/2, -1/2, L]]
```

Chuyển global → body trước khi dùng Jb:

```text
vx_b =  cosφ * ẋ + sinφ * ẏ
vy_b = -sinφ * ẋ + cosφ * ẏ
wz_b = φ̇
```

---

## 5. PID tốc độ bánh (khớp MATLAB / Arduino tham khảo)

Đơn vị consigne & đo: **tour/s**.

```c
// Mỗi bánh i, mỗi chu kỳ dt:
float consigne = omega_ref[i] / (2.0f * PI);   // tour/s, có dấu
float vit_mes  = measure_speed_abs_tps(i);     // >= 0

float err = fabsf(consigne) - vit_mes;
sum_err[i] += err;
float derr = err - err_prev[i];
err_prev[i] = err;

float pwm = PID_KP*err + PID_KI*sum_err[i] + PID_KD*derr;
if (pwm > PWM_MAX) { pwm = PWM_MAX; sum_err[i] -= err; }  // anti-windup
if (pwm < 0)       { pwm = 0;       sum_err[i] -= err; }

set_motor(i, (int)pwm, /*dir*/ consigne >= 0 ? +1 : -1);
```

Khi `|consigne| < epsilon` (vd 0.01 tour/s): `set_motor(i, 0, 0)` và có thể reset `sum_err[i]`.

---

## 6. API driver motor (bắt buộc)

```c
// pins.h — generate từ bảng §3
void motors_init(void);
void set_motor(int idx /*0..2*/, int pwm /*0..255*/, int dir /*+1/0/-1*/);
void motors_stop_all(void);
```

`set_motor`:

1. Ghi DIR theo `dir`.
2. `analogWrite(PWM_PIN[idx], pwm)` (ESP32: dùng LEDC / `ledcWrite`).

---

## 7. Encoder → tốc độ

Mỗi chu kỳ `CTRL_DT_S`:

```c
// ticks[i] tăng trong ISR
noInterrupts();
long t = ticks[i];
ticks[i] = 0;
interrupts();

// Cách A — khớp Arduino tham khảo (1 kênh):
float vit_tps = (t / CTRL_DT_S) / (ENCODER_TICKS_PER_MOTOR_REV * GEAR_RATIO);

// Cách B — nếu biết ticks/vòng TRỤC RA bánh (sau hộp số): TICKS_OUT
// float vit_tps = (t / CTRL_DT_S) / TICKS_OUT;

vit_mes[i] = fabsf(vit_tps);
// omega_meas[i] = sign(consigne[i]) * vit_mes[i] * 2π;  // cho odometry
```

**Hiệu chỉnh bắt buộc trên xe thật:** đo thời gian 1 vòng bánh ở PWM cố định → chỉnh hệ số tick cho đúng `tour/s`.

---

## 8. Vòng lặp realtime (100 Hz)

```c
void control_loop() {
  // 1) Đọc lệnh mới (Serial) nếu có → mode VELOCITY hoặc POSE
  // 2) Nếu mode POSE:
  //      E = Pdes - P; E.theta = wrap_pi(E.theta);
  //      if (norm(E) < POSE_TOL) { stop; return; }
  //      u = LAMBDA_POSE * E;   // xd,yd,phid
  //    Nếu mode VELOCITY: u = [vx, vy, w] (global hoặc body — ghi rõ trong Serial)
  // 3) invkin(u, phi) → omega_ref; saturate_omega(omega_ref);
  // 4) PID 3 bánh → set_motor
  // 5) Đo vit_mes; suy omega_meas
  // 6) fwdkin → u_hat; P += CTRL_DT_S * u_hat; phi = wrap_pi(P.theta)
}
```

Timer / `millis()`: gọi `control_loop` mỗi 10 ms. **Không** `delay()` dài trong loop điều khiển.

---

## 9. Giao thức Serial (đề xuất)

Baud **115200**, dòng kết thúc `\n`.

| Lệnh | Ý nghĩa |
|------|---------|
| `V <vx> <vy> <w>` | Vận tốc global (m/s, m/s, rad/s) |
| `B <vx> <vy> <w>` | Vận tốc body-frame |
| `P <x> <y> <yaw_deg>` | Đi tới pose (bật vòng ngoài λE) |
| `S` | Stop, reset PID integrator |
| `?` | In pose + ω_meas |

Ví dụ:

```text
V 0.1 0.0 0.0
B 0.0 0.1 0.0
P 1.0 0.8 45
S
```

Telemetry (mỗi 50–100 ms, tùy chọn):

```text
T,x,y,yaw_deg,w1,w2,w3,pwm1,pwm2,pwm3
```

---

## 10. Ánh xạ file MATLAB → module firmware

| MATLAB | Firmware |
|--------|----------|
| `omni3_params.m` | `config.h` constants §2 |
| `omni3_invkin.m` | `kinematics.cpp` → `invkin` |
| `omni3_fwdkin.m` | `kinematics.cpp` → `fwdkin` |
| `omni3_saturate.m` | `saturate_omega` |
| `omni3_pid_*.m` | `pid_speed.cpp` |
| `omni3_motor_plant.m` | **Không port** — thay bằng motor + encoder thật |
| `sim_quy_dao_co_PID.m` | `control_loop` §8 |

---

## 11. Checklist mang xe chạy

1. [ ] Điền đủ bảng chân §3  
2. [ ] AI sinh `pins.h` + firmware  
3. [ ] Test từng motor: quay đúng chiều khi `DIR+` / `DIR−`  
4. [ ] Test encoder: tick tăng khi bánh quay  
5. [ ] Hiệu chỉnh hệ số tour/s  
6. [ ] Lệnh `V 0.05 0 0` — xe đi thẳng chậm  
7. [ ] `B 0 0.05 0` — dịch ngang (đặc trưng Omni)  
8. [ ] `B 0 0 0.3` — quay tại chỗ  
9. [ ] `P x y yaw` — bám điểm (nếu implement vòng ngoài)  
10. [ ] So sánh hướng/hành vi với `sim_quy_dao_co_PID` / `sim_quy_dao_khong_PID`

---

## 12. Ràng buộc an toàn (firmware phải có)

- Watchdog lệnh: nếu >200 ms không nhận `V`/`B`/`P` → `motors_stop_all()`.
- Clamp PWM 0…255; clamp `|ω_ref|` ≤ `OMEGA_MAX_RAD_S`.
- Nút emergency / lệnh `S` dừng ngay.
- Không block UART trong ISR encoder.

---

## 13. Cấu trúc project đề xuất

```text
firmware/
  platformio.ini   (hoặc sketch Arduino)
  include/config.h
  include/pins.h          ← sinh từ §3
  src/main.cpp
  src/kinematics.cpp/.h
  src/pid_speed.cpp/.h
  src/motors.cpp/.h
  src/encoders.cpp/.h
  src/control.cpp/.h
  src/serial_cmd.cpp/.h
```

---

## 14. Prompt mẫu gửi Codex sau khi có chân pin

```text
Đọc README_FIRMWARE.md trong repo mophong.
Cấu hình chân đã điền ở §3 như sau:
  MCU: ESP32-S3
  M1 PWM=... DIR=...
  M2 PWM=... DIR=...
  M3 PWM=... DIR=...
  E1=... E2=... E3=...
Viết firmware Arduino/PlatformIO đủ để:
  - nhận lệnh Serial V/B/P/S
  - invkin + saturate + PID 3 bánh + odometry fwdkin
  - control loop 100 Hz
Không đổi r, L, alpha, PID gains, omega_max trong §2.
Tạo pins.h từ bảng chân. Sau khi gen, liệt kê cách nạp và lệnh test.
```

---

## 15. Tham chiếu trong repo

| File | Nội dung |
|------|----------|
| `tinh_toan_mo_hinh_thuc_te.md` | Công thức A / A⁻¹ |
| `sim_quy_dao_co_PID.m` | Mô phỏng có PID |
| `sim_quy_dao_khong_PID.m` | Mô phỏng chỉ động học |
| `omni3_*.m` | Tham số + IK/FK/PID |
| `PHAN_TICH_REPO_THAM_KHAO.md` | Phân tích code Arduino tham khảo |
| `Slide_Xe_tu_hanh_3_banh_omni new.pptx` | Slide công thức |

---

**Tóm lại cho AI:**  
Thông số + công thức + PID đã cố định. Việc còn lại là **điền chân pin §3** rồi sinh firmware map đúng pin, chạy vòng §8, giao tiếp §9.
