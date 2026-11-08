# Xe tự hành dò line (mini AGV) điều khiển qua WiFi dùng ESP32

Xe chạy bám vạch đen khép kín qua 3 trạm, dừng chờ tại từng trạm, tự dừng khi có vật cản và được điều khiển, giám sát qua trang web do chính ESP32 phát ra (không cần Internet). Firmware viết bằng MicroPython.

## Tính năng

- Dò line bằng 5 cảm biến hồng ngoại + bộ điều khiển PID; tự tìm lại vạch khi mất vạch.
- Nhận diện vạch ngang là trạm, phanh dừng, đếm trạm 1 → 2 → 3 → 1.
- Chờ tại trạm theo thời gian (mặc định 5 s) hoặc chờ nhấn nút / lệnh "Đi tiếp".
- Dừng khi vật cản ≤ 20 cm, chạy tiếp khi > 25 cm (HC-SR04).
- Web điều khiển tay (8 hướng, 4 mức tốc độ), giám sát cảm biến, chỉnh và lưu thông số vào flash.
- An toàn: mất WiFi hoặc mất lệnh điều khiển tay thì xe tự dừng.

## Phần cứng

| Linh kiện | Ghi chú |
|---|---|
| ESP32 DevKit (ESP-WROOM-32, USB-C) | Vi điều khiển trung tâm |
| 5 module dò line hồng ngoại LM393 | Gắn hàng ngang phía trước xe |
| HC-SR04 | Đo khoảng cách phía trước |
| Mạch cầu H L298N | Điều khiển 2 động cơ bằng PWM |
| 2 động cơ giảm tốc TT + 2 bánh | Gắn trên khung acrylic |
| Pin 2 × 18650 (7,4–8,4 V) | Cấp cho L298N và mạch hạ áp |
| Mạch hạ áp AMS1117 (5 V) | Cấp nguồn cho ESP32 |
| Nút nhấn, điện trở 1 kΩ + 2 kΩ | Nút "đi tiếp"; phân áp cho chân ECHO |

## Sơ đồ nối chân

| GPIO ESP32 | Nối với |
|---|---|
| 32, 33, 25, 26, 27 | D1 … D5 (module dò line, trái → phải) |
| 22 | ENA (L298N) |
| 16, 17 | IN1, IN2 |
| 21 | ENB (L298N) |
| 18, 19 | IN3, IN4 |
| 13 | TRIG (HC-SR04) |
| 23 | ECHO (HC-SR04) qua cầu phân áp 1 kΩ + 2 kΩ |
| 14 | Nút nhấn (nối GND, dùng pull-up nội) |
| 2 | LED trên board (sáng khi chạy dò line) |

Tất cả GND (pin, L298N, ESP32, cảm biến) phải nối chung.

## Cách nạp code

1. Cài MicroPython cho ESP32 và dùng [Thonny](https://thonny.org) (Interpreter: *MicroPython (ESP32)*).
2. Mở file code, chọn **File → Save as → MicroPython device**, lưu với tên `main.py`.
3. Cấp nguồn lại cho xe: xe ở chế độ **Dừng**, tự bật WiFi và web server.

## Cách sử dụng

1. Trên điện thoại/laptop, kết nối WiFi **`XeDoLine-WiFi`** (mật khẩu đặt trong code, biến cấu hình WiFi).
2. Mở trình duyệt, vào **http://192.168.4.1**.
3. Bấm **Chạy dò line** để xe chạy tự động, **DỪNG XE** để dừng; dùng bàn phím hướng để điều khiển tay (giữ để chạy, thả để dừng).
4. Ở khung **Thông số**, chỉnh PID, tốc độ, khoảng cách vật cản, thời gian chờ trạm... rồi bấm **Áp dụng**; bấm **Lưu vào ESP32** để giữ lại sau khi tắt nguồn (lưu trong `xe_config.json`). Chỉ chỉnh/lưu được khi xe đang dừng.

## Điểm truy cập API

| Phương thức | Đường dẫn | Chức năng |
|---|---|---|
| GET | `/` | Trang web điều khiển |
| GET | `/api/status` | Trạng thái xe và thông số |
| POST | `/api/cmd` | Gửi lệnh một ký tự, ví dụ `{"cmd":"F"}` |
| POST | `/api/params` | Chỉnh thông số, ví dụ `{"KP":6,"AD":20}` |
| POST | `/api/save` | Lưu thông số vào flash |

## Lưu ý khi chỉnh xe

- Nếu module dò line có logic ngược, đổi `LINE_LEVEL` trong code.
- Nếu động cơ quay ngược chiều, đổi `INVERT_LEFT` / `INVERT_RIGHT`.
- Dùng biến trở xanh trên module LM393 để chỉnh ngưỡng phát hiện vạch đen.

## Video demo

[Dán link video tại đây]

## Thông tin nhóm

- Đề tài: Thiết kế và chế tạo xe tự hành dò line vận chuyển hàng qua các trạm, tránh vật cản và điều khiển qua WiFi dùng ESP32
- Thành viên: [Điền họ tên – MSSV]
