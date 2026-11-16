# Đối chiếu công thức với slide và servo.ino

Nguồn đối chiếu: [bộ slide gồm 40 trang](../Slide_Xe_tu_hanh_3_banh_omni31_10.pptx), `C:/Users/Admin/Downloads/servo/servo.ino`, [kinematics.cpp](../firmware/src/kinematics.cpp) và [config.h](../firmware/include/config.h). Số slide dưới đây là thứ tự trang thực tế trong tệp, không phải số trang chân trang vì một số chân trang bị lặp. Công thức được đọc từ đối tượng toán học OOXML, bao gồm căn bậc hai, phân số và dấu đạo hàm. Tệp slide và sketch tham khảo không được sửa.

## 1. Cấu hình chân

[pins.h](../firmware/include/pins.h) trùng với các mảng `ENA`, `IN1`, `IN2`, `EA`, `EB` và cấu hình MPU trong `servo.ino`:

| Bánh | PWM/ENA | IN1 | IN2 | Encoder A | Encoder B |
|---|---:|---:|---:|---:|---:|
| 1 | 1 | 2 | 3 | 4 | 5 |
| 2 | 10 | 11 | 12 | 6 | 7 |
| 3 | 13 | 14 | 21 | 17 | 18 |

MPU9250: SDA = GPIO 8, SCL = GPIO 9, địa chỉ I2C `0x68`.

## 2. Quy ước và tham số

Slide 3, 13, 29 và 31 dùng:

- Góc bố trí bánh `α = [0, +2π/3, −2π/3]`.
- Bán kính bánh `r = 0.029 m`, khoảng cách tâm xe tới tâm bánh `L = 0.12 m`.
- Tốc độ bánh `ω` tính bằng rad/s, góc hướng thân xe `φ` tính bằng radian.
- Tốc độ danh định 50 rpm tương ứng `ωmax = 2π × 50 / 60 ≈ 5.235988 rad/s`.
- `x, y` thuộc hệ toàn cục. `vx0, vy0` là vận tốc trong hệ gắn với thân xe.

Các giá trị hình học và thứ tự bánh này khớp firmware và sketch tham khảo. Chiều motor/encoder vật lý phải được hiệu chuẩn để tuân theo quy ước dấu này; công thức không tự sửa được việc đấu ngược chiều.

## 3. Động học nghịch

Slide 10–12:

```text
          [ 0       1      L ]
Jb =      [ −√3/2  −1/2    L ]
          [ +√3/2  −1/2    L ]

          [ cosφ   sinφ    0 ]
R(−φ) =   [−sinφ   cosφ    0 ]
          [ 0      0       1 ]

[ω1, ω2, ω3]ᵀ = (1/r) Jb R(−φ) [ẋ, ẏ, φ̇]ᵀ
```

Tương đương với từng hàng trong `invkin()`:

```text
ωi = [−sin(φ + αi) ẋ + cos(φ + αi) ẏ + L φ̇] / r
```

`invkin_body()` dùng trực tiếp `Jb` với đầu vào `vx0, vy0, φ̇`, giống `inverseKinematics()` trong sketch. Không nhân thêm ma trận quay khi đầu vào đã thuộc hệ thân xe.

## 4. Động học thuận

Slide 7–8 và `fwdkin()`:

```text
ẋ = (2r/3) Σ[−sin(φ + αi) ωi]
ẏ = (2r/3) Σ[ cos(φ + αi) ωi]
φ̇ = r/(3L) Σωi
```

Tại hệ thân xe, rút gọn thành:

```text
vx0 = r/√3 (ω3 − ω2)
vy0 = r/3 (2ω1 − ω2 − ω3)
φ̇  = r/(3L) (ω1 + ω2 + ω3)
```

Đây chính là `forwardKinematics()` trong `servo.ino`. `fwdkin()` của firmware tính trực tiếp trong hệ toàn cục; sketch tính trong hệ thân rồi gọi `bodyToWorld()`. Hai cách tương đương.

Encoder phải cung cấp vận tốc **có dấu** cho động học thuận. Nếu lấy trị tuyệt đối trước bước này, xe không thể ước lượng đúng chuyển động ngược chiều.

## 5. Ví dụ kiểm tra từ slide

| Trường hợp | Đầu vào | Kết quả từ công thức |
|---|---|---|
| Slide 14, động học thuận | `ω=[4,2,3] rad/s`, `φ=0` | `vx0=0.0167431578 m/s`, `vy0=0.029 m/s`, `φ̇=0.725 rad/s` |
| Slide 15, tích phân 5 s | Bắt đầu `(0,0,0)`, giữ các tốc độ bánh trên | `x=−0.0861507216 m`, `y=0.0249498736 m` |
| Slide 17, động học nghịch | `ẋ=0.1 m/s`, `ẏ=0.05 m/s`, `φ̇=0.5 rad/s`, `φ=0` | `ω=[3.7931034483, −1.7793979441, 4.1931910475] rad/s` |

Các kết quả phù hợp số làm tròn trong slide. Ví dụ slide 15 cần tích phân với hướng xe thay đổi, không dùng `x=vx0×5`, `y=vy0×5`.

## 6. Động lực học và giới hạn của điều khiển PWM

Slide 22–23, 29 và 31 cho ma trận lực trong hệ thân:

```text
Fx0 = −√3/2 f2 + √3/2 f3
Fy0 = f1 − 1/2 f2 − 1/2 f3
Mz  = L (f1 + f2 + f3)
τi  = r fi
```

Với giả thiết bỏ ma sát `Fc=0` được nêu rõ ở slide 29–32:

```text
Fx0 = m ax0,  Fy0 = m ay0,  Mz = Jz αφ

f1 =  2/3 Fy0 + Mz/(3L)
f2 = −1/√3 Fx0 − 1/3 Fy0 + Mz/(3L)
f3 = +1/√3 Fx0 − 1/3 Fy0 + Mz/(3L)
```

`forwardDynamics()` và `inverseDynamics()` của sketch tham khảo khớp mô hình không ma sát này. `ax0, ay0` phải được hiểu là các thành phần của gia tốc quán tính biểu diễn trong hệ thân, không đơn thuần là đạo hàm của vận tốc trong hệ đang quay.

Firmware điều khiển xe sử dụng động học và phản hồi tốc độ/vị trí. L298N nhận PWM và chiều quay; không có đo dòng hoặc bộ điều khiển mô-men để bảo đảm lực `fi` hay mô-men `τi` thực tế bằng giá trị mô hình. Vì vậy, ví dụ lực hằng trên slide là bài toán mô hình, không phải lệnh PWM bảo đảm xe đi đúng quãng đường đó.

## 7. Những chỗ slide tự mâu thuẫn

Các công thức tổng quát ở slide 8 và 10–12 là cơ sở đối chiếu động học. Không thể làm mã nguồn trùng đồng thời với tất cả các dòng sau vì chúng mâu thuẫn nhau:

1. **Slide 17:** dòng khai triển riêng `ω1` viết `+ẋ sinφ`. Theo ma trận ngay trên cùng trang và slide 12, phải là `−ẋ sinφ`. Với ví dụ `φ=0` lỗi dấu không hiện ra trong kết quả số.
2. **Slide 21:** viết `−f3 sin(−30°) = −f3/2`, nhưng vế trái bằng `+f3/2`. Thành phần đúng theo ma trận slide 22–23 là `f3y=−f3/2`; dòng trung gian bị sai dấu.
3. **Slide 23–24 so với 25–26:** hai trang đầu dùng `m a + Fc = F`, còn các trang sau dùng `m a − Fc = F` cho hai trục tịnh tiến. Khi cùng dùng `Fc` là lực cản được trừ khỏi lực kéo như lời giải thích ở slide 23–24, dấu phải thống nhất với `m a + Fc = F`. Các ví dụ `Fc=0` không bộc lộ lỗi này.
4. **Slide 27:** phép thế số gia tốc ghi `2.986/1 = 0.597`, trong khi với `τ2=−0.01, τ3=0.01 N·m`, lực đúng xấp xỉ `0.597259 N`. Slide 29 dùng một bộ đầu vào khác (`±0.05 N·m`), nên lực `2.986294 N` ở đó là đúng.
5. **Slide 16:** góc tham số của đường tròn lúc đầu là `−63.43°`, còn hướng thân xe lúc đầu là `0°`. Hai đại lượng cùng được ký hiệu gần giống `φ` nhưng cần phân biệt khi triển khai quỹ đạo.

Không thay đổi bản slide trong lần cập nhật firmware này.

## 8. PID, MPU và độ chính xác

Bộ slide không quy định công thức PID, hệ số PID, số tick encoder mỗi vòng hay thuật toán hợp nhất MPU. Các phần này là thiết kế điều khiển bổ sung, cần kiểm tra trên phần cứng.

Trong sketch tham khảo gốc, `CPR=0`, `PI_ENABLED=false`; MPU chỉ được đọc để báo trạng thái. Các tốc độ đích dựa trên quỹ đạo mô hình, và khi hết thời gian, `modelPose` được gán vào đích mô hình. Cách đó không chứng minh xe thật đã tới đích.

Để đánh giá bản điều khiển có phản hồi, cần đo và ghi lại:

- Số cạnh lên A trên một vòng **trục ra/bánh**, đúng chế độ đếm đang dùng; hiệu chuẩn riêng từng bánh khi cần.
- Chiều motor và dấu encoder theo quy ước các ma trận ở trên.
- Bán kính lăn thực tế và `L`, sai số đi thẳng trên quãng đường đã đo bằng thước.
- Bias gyro khi xe đứng yên, hướng trục Z và dấu góc quay của MPU so với hệ xe.
- Sai số cuối đường, sai số ngang đường, độ lặp lại qua nhiều lần thử; so sánh với vị trí đo bên ngoài.

PID có thể giảm sai số bám theo ước lượng vị trí. Encoder và gyro vẫn tích lũy sai số khi trượt bánh hoặc lệch hiệu chuẩn. Tích phân hai lần gia tốc MPU không cung cấp vị trí tuyệt đối chính xác. Muốn xác nhận độ chính xác tuyệt đối hoặc hiệu chỉnh trôi dài hạn cần phép đo tham chiếu bên ngoài.
