# DuyBot Live2D

Board tùy biến dùng một ESP32-S3 cho Xiaozhi, màn hình, âm thanh, cảm biến và chassis hai bánh.

## Baseline phần cứng

| Chức năng | GPIO |
| --- | ---: |
| L298N ENA / motor trái | 1 |
| L298N ENB / motor phải | 2 |
| L298N IN1, IN2 | 11, 12 |
| L298N IN3, IN4 | 13, 14 |
| HY-SRF05 TRIG | 10 |
| HY-SRF05 ECHO | 18 |
| HY-SRF05 OUT / MODE | Để hở |
| Cảm biến line / mép bàn | 3 |
| Cảm biến chạm | 8 |

Nguồn motor phải tách khỏi nguồn logic ESP32 và nối chung GND. Chân ECHO 5 V của HY-SRF05 phải qua cầu chia áp hoặc level shifter trước khi vào ESP32-S3 3,3 V. Firmware dùng chế độ TRIG/ECHO riêng, vì vậy chân thứ năm OUT/MODE phải để hở; nối nó xuống GND sẽ chuyển cảm biến sang chế độ một dây và GPIO18 không nhận ECHO.

## Kiến trúc chuyển động

`MotionController` là nơi duy nhất được phép ghi GPIO/PWM motor. Mọi nguồn lệnh gửi qua queue theo thứ tự ưu tiên:

1. Safety / chống rơi và dừng khẩn cấp.
2. Manual web.
3. MCP / lệnh AI.
4. Explore tự động.
5. Idle và chuyển động cảm xúc.

Mọi lệnh chuyển động đều có deadline. Nếu mất Wi-Fi, task bị treo hoặc trình duyệt ngừng gửi keepalive, watchdog sẽ đưa PWM về 0 và thả bốn chân chiều motor về LOW.

## Self-check và supervisor agent

Robot chạy self-check không chuyển động sau mỗi lần boot. Kết quả có tại `GET /api/status` hoặc `GET /api/selftest?run=1`:

- `self_check=passed`: motion controller và HY-SRF05 sẵn sàng.
- `self_check=degraded`: robot vẫn dùng được màn hình, âm thanh, chạm và manual; tự hành bị khóa.
- `self_check_reason=hy_srf05_echo_stuck_low_check_out_floating`: ECHO không đổi mức sau TRIG; kiểm tra OUT/MODE đang để hở, nguồn 5 V, GND chung, dây TRIG/ECHO, cầu chia áp và cảm biến.
- `dist=451` / giao diện `CLEAR`: HY-SRF05 đã phản hồi bằng xung timeout khoảng 30 ms nhưng không thấy vật trong tầm; đây là trạng thái khỏe, khác với `dist=-1` là không nhận được ECHO.
- `autonomy_armed=false`: trạng thái bắt buộc sau boot. Agent không được tự quay bánh.
- `reset_reason=brownout` hoặc `power_glitch`: nguồn 3,3 V đã sụt. Agent bị khóa ít nhất 60 giây chạy liên tục; không vô hiệu hóa brownout detector để lách lỗi nguồn.

Supervisor agent chạy cục bộ trên ESP32, theo dõi sensor, hội thoại, chạm, mood, hazard và motion deadline. Xiaozhi/LLM là lớp suy luận cấp cao qua các MCP tool `self.body.self_check`, `self.agent.get_state` và `self.agent.set_autonomy`; supervisor có quyền từ chối lệnh AI nếu phần cứng chưa an toàn.

## Đồng bộ body và mind

Mọi emotion từ Xiaozhi, touch reaction hoặc behavior API đi qua cùng một coordinator. Coordinator ánh xạ ý định cấp cao thành biểu cảm và body pulse có deadline: `greet`, `celebrate`, `curious`, `startled`, `comfort`, `confident`, `rest`. Nếu agent chưa arm, nét mặt vẫn phản ứng nhưng bánh bị khóa và trạng thái báo `face_only_autonomy_locked`. Khi đã arm, biểu cảm và chuyển động chạy cùng một sequence; STOP, mất sensor hoặc hazard sẽ hủy sequence và safety controller giành quyền motor.

LLM gọi MCP `self.body.perform_behavior`. Web/manual có `GET /api/behavior?name=greet`; `/api/status` trả thêm `behavior`, `behavior_state`, `behavior_sequence` và `behavior_body_executed` để xác nhận body có thực sự tham gia hay chỉ chạy nét mặt.

Robot có hai control mode tách biệt. `manual` là chế độ chơi RC bằng các nút giữ-thả, vẫn có motion watchdog và guard. `smart` dành cho body–mind/AI và mới cho phép arm tự hành. Mỗi lần đổi mode đều emergency STOP, hủy behavior/explore và disarm để hai nguồn lệnh không tranh nhau. Chuyển bằng nút trên web, `GET /api/mode?value=manual|smart`, hoặc MCP `self.agent.set_control_mode`.

Touch GPIO8 có debounce 80 ms và gesture engine: một chạm gọi `greet`, chạm đôi gọi `celebrate`, giữ từ 0,9 giây gọi `comfort`. Cả ba đi qua body–mind coordinator và phát âm thanh tương ứng; chạm đôi được khóa release để không phát dư một tap. Telemetry gồm `touch_gesture`, `touch_event_count` và `touch_age_ms`.

Các API điều khiển an toàn:

- `GET /api/agent?enabled=1&arm=1`: arm tự hành, chỉ thành công sau self-check PASS.
- `GET /api/agent?arm=0`: disarm tự hành.
- `GET /api/explore?s=15`: đi dạo có giới hạn khi đã arm.
- `GET /api/motor-test?confirm=LIFTED`: test bốn hướng; chỉ gọi sau khi nhấc hai bánh khỏi mặt bàn. Lệnh STOP sẽ hủy test.
- `GET /api/behavior?name=curious`: chạy một hành vi body–mind; phần bánh chỉ hoạt động khi agent đã arm.

## Checklist thử nghiệm an toàn

1. Tháo pin motor hoặc nhấc cả hai bánh khỏi mặt bàn trước khi flash.
2. Boot và kiểm tra log có `Motion controller ready`.
3. Khi không có lệnh, đo ENA/ENB ở mức 0 và bốn chân IN ở LOW.
4. Mở trang manual, giữ từng nút dưới một giây và xác nhận đúng chiều từng bánh.
5. Ngắt Wi-Fi trong lúc giữ nút; bánh phải dừng trong tối đa 700 ms.
6. Nhấn/thả nút; lệnh `stop` phải dừng ngay và xóa các lệnh đang chờ.
7. Chỉ đặt robot xuống sàn sau khi line sensor đã được kiểm tra. Không thử chống rơi trực tiếp trên bàn cao.
8. Kiểm tra HY-SRF05 bằng vật phẳng ở 10, 20 và 50 cm; giá trị `-1` nghĩa là timeout, OUT/MODE sai chế độ hoặc wiring/level chưa đúng.
9. Nếu API báo brownout, cấp ESP32 bằng USB/nguồn 5 V ổn định riêng, cấp motor bằng pin riêng, nối chung GND và đặt tụ 470–1000 µF gần L298N; không lấy nguồn motor qua chân 5 V/3V3 của ESP32.

## Roadmap tới trải nghiệm kiểu Vector

- Giai đoạn 0 — baseline: pin map, build tái lập và checklist phần cứng. Hoàn thành.
- Giai đoạn 1 — motion foundation: queue ưu tiên, watchdog, dừng khẩn cấp và một chủ sở hữu motor. Hoàn thành code, chờ test trên chassis.
- Giai đoạn 2 — perception tối thiểu: sửa HY-SRF05, debounce line/touch, sensor snapshot có timestamp và lỗi rõ ràng.
- Giai đoạn 3 — chuyển động chính xác: encoder hai bánh, IMU, PID tốc độ, quay theo góc và odometry.
- Giai đoạn 4 — behavior engine: state machine nhu cầu/tâm trạng, animation + âm thanh + chuyển động đồng bộ, bộ nhớ ngắn hạn.
- Giai đoạn 5 — nhận thức môi trường: camera, nhận diện người/khuôn mặt, tracking và tránh vật cản nhiều hướng.
- Giai đoạn 6 — tự chủ: docking marker, đo pin, tự tìm dock và sạc an toàn.

Mỗi giai đoạn chỉ được coi là xong khi có test bench và test trên sàn; không dùng chuyển động theo thời gian cố định thay cho encoder ở giai đoạn 3 trở đi.
