# PROGRESS — Xe dò line 16 mắt 2026

> Cập nhật: 2026-11-11 · Commit rà soát: `1c6324c` (main) · Người rà soát: Claude Code
>
> **Phạm vi rà soát:** đọc tĩnh toàn bộ `src/`, `include/`, `index.html`, `platformio.ini`.
> **Chưa làm được:** build và chạy thử (máy rà soát không có PlatformIO, không có xe). Mọi bug dưới đây là kết luận từ đọc code; các mục ghi "cần đo" là suy luận chưa có số liệu thực.

---

## 1. Tiến trình dự án

### 1.1 Lịch sử (36 commit, 03/11 → 11/11/2026)

| Ngày | Mốc chính | Tag |
|---|---|---|
| 03/11 | Khung Dual-Core FreeRTOS + Web BLE + quản lý 3 map; tách Mode 1 (Scan) / Mode 2 (Race); lưới 16 mắt realtime; chốt cua 90° | — |
| 04/11 | Sửa giao thức telemetry nhiều lần (binary → CSV 1 gói `T:`); lọc IIR 85/15 + lọc mắt cô độc; soft-start; căn tâm sau calib; lap timer; nhận diện vạch đích | `stable-1`, `stable-2`, `complete-BLEv1`, `zero-base-align-v1`, `finish-line-v1` |
| 05/11 | Vạch đích ≥14 mắt + debounce 10 chu kỳ; PID mặc định 4/0/20/120; bộ chuyển mode | `align-pwm-boost-v1`, `mode2-pid-normal-v1`, `final_v1_051106` |
| 07–08/11 | Thử nghiệm PID nâng cao (EMA D-filter, guarantee action, ADC nhanh 8x, cua vi sai) → **revert 2 lần về `final_v1_051106`**; thêm chỉnh tốc độ rẽ 90° trên web; waypoint đổi sang `start_ticks`, lưu localStorage | `final_v2_081106`, `final_v3_081106` |
| 11/11 | Tắt auto_boost khi ôm cua | — |

### 1.2 Trạng thái từng module

| Module | File | Trạng thái |
|---|---|---|
| Động cơ (PWM 20 kHz, phanh ngắn mạch, coast) | `motor.cpp` | Xong |
| Cảm biến 16 mắt qua MUX, calib quét 4 s, IIR | `sensor.cpp` | Chạy được; calib không kiểm tra hợp lệ, không lưu flash |
| Encoder PCNT X4 | `encoder.cpp` | Chạy được; lỗi tiềm ẩn khi tràn ±32767 xung |
| PID + máy trạng thái vạch (normal/lost/cross/dashed/corner) | `pid_controller.cpp` | Chạy được; đang tinh chỉnh (đã revert 2 lần) |
| Quản lý map/waypoint + NVS | `map_manager.cpp` | Xong phần lưu; thiếu kiểm tra dữ liệu |
| BLE NUS + lệnh `CMD:*` | `ble_handler.cpp` | Xong; gửi notify không có khóa |
| Vòng điều khiển, căn tâm, vạch đích | `main.cpp` | Chạy được; lỗi chuyển mode sau calib |
| Web dashboard | `index.html` | Xong phần chính; không đồng bộ ngược từ xe, thiếu nút chọn làn |

### 1.3 Việc còn dở / chưa có

- Nút chọn **làn Trái/Phải** và **đảo cực cảm biến**: firmware có lệnh (`CMD:LANE`, `CMD:POLARITY`), web không có nút nào gọi. README mô tả có.
- Web không đọc `MAP_INFO:` / `WP:` xe gửi về (chỉ in vào log) → bảng mốc trên web chỉ là bản localStorage.
- PID, tốc độ nền, tốc độ rẽ, dữ liệu calib **không lưu flash** → thay pin là về mặc định.
- README còn ghi scan ở tốc độ 70 và "phanh dừng tại rương kho báu" — code hiện tại đã khác.
- Không có test nào (`test/` chỉ có README).

---

## 2. Rà soát bug

### 2.1 Nghiêm trọng

**B1. Bấm RACE sau khi calib lại giữa chừng → không reset xung, không reset đồng hồ** — `main.cpp:62-128`
Khối căn tâm (ALIGN) luôn `continue` trước khối chuyển mode, nên `s_prev_mode` không được cập nhật trong lúc căn tâm. Nếu xe đang ở mode SCAN/RACE mà bấm CALIB (ví dụ xe mất line tự dừng — xem B6 — rồi người dùng calib lại, hoặc bấm RACE trước khi calib lần đầu), thì sau calib bấm RACE: `mode == s_prev_mode` → bỏ qua `encoder_reset()`, `s_race_start_us`, `LOG:AUTO_START`.
Hậu quả: xung còn nguyên của lượt trước → mốc waypoint lệch hết, `dist_ok` đúng ngay từ vạch xuất phát (có thể báo về đích tức thì), thời gian vòng sai, đồng hồ web không chạy.
Sửa: gán `s_prev_mode = CAR_MODE_ALIGN;` ở đầu khối ALIGN.

**B2. Bấm STOP là xung về 0, nút "CHỤP MỐC" ghi số giả 1000** — `main.cpp:112-114`, `index.html:923`
Khối chuyển mode gọi `encoder_reset()` cho *mọi* lần đổi mode, kể cả → STOP. Trái với ghi chú trong `ble_handler.cpp:35` và tiêu đề thẻ trên web "KHÔNG BỊ RESET KHI DỪNG". Sau khi STOP, `lastLiveTicks = 0` → `captureCurrentTicks()` âm thầm chèn mốc 1000 xung.
Sửa: chỉ reset encoder khi vào SCAN/RACE; trên web bỏ giá trị thay thế 1000 (báo lỗi nếu xung = 0).

**B3. Nút GPIO35 dùng `INPUT_PULLUP` nhưng chân 34–39 không có điện trở kéo nội** — `main.cpp:275`
Nếu mạch không có điện trở kéo lên ngoài, chân thả nổi → tự kích calib / tự xuất phát / tự dừng ngẫu nhiên. Cần kiểm tra mạch; nếu chưa có thì hàn 10 kΩ lên 3V3 hoặc chuyển nút sang chân khác.

**B4. Vòng điều khiển gần như chắc chắn không đạt 500 Hz** *(cần đo)* — `main.cpp:52`, `pid_controller.cpp:120`, `sensor.cpp:51-59`
Mỗi chu kỳ gọi `sensor_read_all()` **2 lần** (một lần ở `controlTask`, một lần trong `pid_line_follow_step`), mỗi lần 16 kênh × 4 mẫu = 64 `analogRead`. `analogRead` trên ESP32 Arduino cỡ vài chục µs mỗi lần → ước lượng 5–10 ms/chu kỳ, vượt xa 2 ms. Nếu đúng:
- "Debounce 10 chu kỳ = 20 ms" thực tế là 50–100 ms → xe chạy nhanh qua vạch đích rộng 2–3 cm sẽ **không bao giờ đủ 10 chu kỳ liên tiếp** → không nhận vạch đích.
- `vTaskDelayUntil` không bao giờ ngủ → task ưu tiên 10 chiếm trọn Core 1 → `loop()` (nút bấm, LED) bị bỏ đói.
- Kd, soft-start (+1 PWM/chu kỳ), `encoder_update(0.002f)` đều giả định 2 ms → sai tỉ lệ.
Cách đo: bọc `micros()` quanh thân vòng lặp, in giá trị lớn nhất mỗi giây. Sửa tối thiểu: bỏ lần đọc thứ hai (truyền `position`/`blackCount` vào `pid_line_follow_step`), giảm `ADC_SAMPLES`.

**B5. Calib luôn báo thành công, mắt không quét qua vạch sẽ nhiễu thành "đen"** — `sensor.cpp:99-109`, `145-168`
`calibrationReady = true` bất kể kết quả. Mắt có `max − min` nhỏ (chỉ thấy nền trắng) vẫn bị chuẩn hóa 0–1000 → nhiễu vài chục LSB thành nháy đen/trắng. Mắt rìa (0, 1, 14, 15) hay bị nhất, và chính chúng kích chốt cua 90° (`pid_controller.cpp:215, 222`). `MIN_CALIBRATION_DELTA` có khai báo nhưng chỉ dùng cho mảng `blackIsHigher[]` không ai đọc.
Sửa: trong `sensor_get_calibrated()` thêm `if (mx - mn < MIN_CALIBRATION_DELTA) return 0;` và báo về web số mắt calib hỏng.

### 2.2 Trung bình

**B6. Mất line quá 1,5 s chỉ phanh, không thoát mode** — `pid_controller.cpp:409-412`
Mode vẫn là RACE, đồng hồ web chạy mãi; đặt xe lại lên vạch là xe tự phóng đi. Nên gọi `car_set_run_mode(CAR_MODE_STOP)` và gửi thông báo.

**B7. Gửi BLE notify từ 3 task không có khóa** — `ble_handler.cpp:223-230`, `246-248`
`setValue()` + `notify()` được gọi từ `bleTask` (telemetry), task callback BLE (ACK) và `controlTask` (`LOG:RACE_FINISH`, `ACK:CALIB_DONE`). Hai lời gọi không nguyên tử → gói này có thể đè gói kia, mất `LOG:RACE_FINISH` (mất thời gian vòng). Thêm nữa `ble_send_string` có `delay(5)` chạy ngay trong task điều khiển thời gian thực.
Sửa: một mutex quanh cặp setValue/notify; hoặc `controlTask` chỉ đặt cờ, `bleTask` gửi.

**B8. Waypoint không được sắp xếp** — `map_manager.cpp:118-142`, `index.html:938`
Thuật toán giả định `start_ticks` tăng dần, nhưng web cho sửa tự do và "CHỤP MỐC" luôn chèn cuối. Ví dụ mốc [1000, 3000, 2000]: tại 3500 xung xe chọn mốc #2 (2000) thay vì #1 (3000).
Sửa: `list.sort((a, b) => a.ticks - b.ticks)` trước khi lưu xuống xe.

**B9. Map đang chọn trên web và trên xe có thể lệch nhau** — `index.html:528`, `map_manager.cpp:49`
Web luôn mở ở MAP 1; xe nạp `sel_map` từ flash. `CMD:MAP` chỉ đổi trong RAM (không lưu nếu không bấm "LƯU VÀO XE"). Sau khi thay pin / tải lại trang, tab trên web và map xe chạy có thể khác nhau mà không có cảnh báo.
Sửa: gửi `CMD:MAP:${activeMap}` ngay sau khi kết nối.

**B10. Lưu map dùng ghi-không-phản-hồi, không kiểm tra kết quả** — `index.html:957-969`, `map_manager.cpp:74-91`
`writeValueWithoutResponse` có thể rớt gói. Rớt 1 gói `WP_SET` ở giữa → ô đó giữ dữ liệu cũ (vì `WP_CLEAR` chỉ đặt `count = 0`, không xóa mảng) mà `count` vẫn tăng. Web vẫn in "✅ Đã lưu vĩnh viễn" kể cả khi chưa kết nối BLE. Xe cũng trả `ACK:WP_SET_OK` kể cả khi `map_set_waypoint()` trả `false` (mốc thứ 17 trở đi) — `ble_handler.cpp:155`.
Sửa: đổi sang `writeValueWithResponse` (characteristic đã hỗ trợ), chỉ báo thành công khi nhận `ACK:SAVED_FLASH`.

**B11. Nút 🗑️ xóa mốc không lưu flash** — `index.html:949-955`
Gửi `WP_CLEAR` nhưng không gửi `CMD:SAVE` → khởi động lại xe là mốc cũ quay lại, trong khi web đã trống.

**B12. Encoder sai khi vượt ±32767 xung (~32 m mỗi bánh trong một lượt)** — `encoder.cpp:20-21`, `32-38`, `100-103`
Bật `accum_count` nhưng không thêm watch point ở hai giới hạn (IDF yêu cầu mới bù tràn) → bộ đếm phần cứng về 0 khi chạm giới hạn, tổng xung tụt ~32767. Ngược lại nếu bù tràn hoạt động thì `s_last_raw_pcnt_*` kiểu `int16_t` lại cắt cụt. Đường nào cũng sai; hiện chưa lộ vì mỗi lượt đều reset và sa bàn ngắn hơn 32 m.
Sửa: thêm `pcnt_unit_add_watch_point()` cho hai giới hạn và đổi hai biến sang `int`.

**B13. Điều kiện chốt cua 90° không yêu cầu mắt giữa thấy vạch** — `pid_controller.cpp:215, 222`
Chỉ cần ≥2 mắt trong cụm ngoài (có mắt 0/1 hoặc 14/15) và phía đối diện trắng. Trên đoạn lượn sóng / cua gắt, vạch trượt ra mép là kích xoay tại chỗ thay vì để PID xử lý. Có thể là chủ ý ("early detection") nhưng đây là ứng viên số một cho hiện tượng văng/giật ở cua — nên xem lại khi tinh chỉnh.

**B14. `platformio.ini` không build lại được trên máy sạch** *(cần build để xác nhận)* — `platformio.ini:2`
Code dùng API Arduino-ESP32 3.x (`ledcAttach(pin, freq, res)`, `driver/pulse_cnt.h`, `getValue()` trả `String`), nhưng `platform = espressif32` không ghim phiên bản; bản chính thức của PlatformIO đi kèm core 2.x và sẽ lỗi biên dịch. Cần ghim đúng platform đang dùng trên máy phát triển (ví dụ gói pioarduino) vào file.

### 2.3 Thấp

| # | Vị trí | Vấn đề |
|---|---|---|
| B15 | `pid_controller.cpp:230` | Dùng `blackCount == 0` (ngưỡng 500) để kết luận mất line, bỏ qua giá trị trả về của `pid_calculate_position()` (ngưỡng tổng 300). Vạch mờ 150–499 vẫn tính được vị trí nhưng bị coi là mất line. |
| B16 | `pid_controller.cpp:176-248` | `isLineCurrentlyLost` chỉ xóa ở nhánh NORMAL. Chuỗi mất line → ngã tư/chốt cua → mất line dùng lại mốc thời gian cũ → bỏ qua nhánh nét đứt, có thể phanh ngay. |
| B17 | `pid_controller.cpp:279-281, 343` | Sau chốt cua `lastError = 0`, sau LOST/CROSS/DASHED `lastError` cũ → chu kỳ PID đầu tiên có cú giật D (tối đa 75 × Kd). |
| B18 | `pid_controller.cpp:372-373`, `ble_handler.cpp:111-116` | Kp/Ki/Kd từ BLE không giới hạn; ép `float → int16_t` ngoài dải là hành vi không xác định. |
| B19 | `motor.cpp:13, 19` | Lệnh ±1..37 bị nâng thành ±38 → bước nhảy 76 PWM quanh 0; ở chế độ test `baseSpeed = 0` bánh rung quanh tâm. |
| B20 | `main.cpp:188-189` | Khi chỉ có 1 waypoint, `wp_idx >= total_wps - 1` đúng ngay từ đầu (trước mốc 0 cũng trả `wp_idx = 0`) → cổng "đã qua mốc cuối" vô hiệu. |
| B21 | `main.cpp:131, 227` | Bấm RACE khi chưa calib: xe đứng yên nhưng mode = RACE, đồng hồ web chạy, không có thông báo lỗi. |
| B22 | `pid_controller.cpp:434, 442` | CROSS/DASHED dùng thẳng `baseSpeed`, bỏ qua soft-start (xuất phát trên vạch ngang là bung ga ngay). |
| B23 | `map_manager.cpp:44-50` | Nạp flash không kẹp `count ≤ MAX_WAYPOINTS`; dữ liệu hỏng → đọc tràn mảng. |
| B24 | `encoder.cpp:80-103` | `s_last_raw_pcnt_*` cập nhật ngoài vùng khóa; `CMD:RESET_ENC` từ Core 0 đúng lúc có thể làm xung nhảy sai trong 1 chu kỳ. |
| B25 | `main.cpp:135` | `labs()` nhận `long` 32-bit, truyền `int64_t` bị cắt (chỉ sai khi > 2 tỉ xung). |
| B26 | `index.html:775-780` | `ACK:FINISH_STOPPED` gọi `stopTimer()` đè dòng trạng thái "🏁 VỀ ĐÍCH" thành "⏹ ĐÃ DỪNG". Best lap gộp chung cả lượt Scan lẫn Race và cả 3 map. |
| B27 | `index.html:616, 939` | `JSON.parse` không kiểm tra cấu trúc; ô trống → `NaN` gửi xuống xe thành 0. |
| B28 | `sensor.h:10-16` vs `config.h` | Khai báo chân trùng lặp ở 2 file (`PIN_MUX_A` vs `PIN_MUX_S0`…); sửa `config.h` không có tác dụng với cảm biến. |
| B29 | `sensor.cpp`, `main.cpp` | `isCalibrating`, `calibrationReady` dùng chung 2 core nhưng không `volatile`. |

### 2.4 Code chết (xóa được)

- `sensor.cpp`: `sensor_get_line_position_analog/_binary/()`, `sensor_is_crossline`, `sensor_is_out_of_line`, `sensor_is_90_left/right`, `sensor_print_states`, `sensor_is_button_pressed`, `sensor_handle_calibration_step`, mảng `thresholds[]`, `blackIsHigher[]`, `s_last_line_pos`, `PIN_CALIB`.
- `main.cpp`: `s_run_requested`, `s_active_speed`. `ble_handler.cpp`: `oldDeviceConnected`, dòng `setMinPreferred` lặp. `config.h`: `PIN_BUTTON_34`, `PIN_MUX_S0..S3`.
- `encoder.cpp`: vận tốc/RPM được tính mỗi chu kỳ nhưng không nơi nào dùng.

---

## 3. Thứ tự đề xuất xử lý

1. **Đo thời gian chu kỳ thực (B4)** — kết quả quyết định cách hiểu mọi hằng số thời gian khác.
2. Kiểm tra điện trở kéo lên của nút GPIO35 (B3).
3. Sửa B1, B2 (mỗi cái 1–3 dòng trong `main.cpp`).
4. Sửa B5 (1 dòng) và B6.
5. Web: B8, B9, B10, B11 (mỗi cái vài dòng trong `index.html`).
6. Ghim platform trong `platformio.ini` (B14), thêm mutex BLE (B7), sửa encoder (B12).
7. Dọn code chết (mục 2.4), cập nhật README.

## 4. Nhật ký

| Ngày | Việc |
|---|---|
| 2026-11-11 | Clone repo, đọc toàn bộ mã nguồn, lập file này. |
| 2026-11-11 | Chế độ ép trái/phải chỉ bỏ 6 mắt phía ngược lại (ép trái đọc mắt 0–9, ép phải đọc mắt 6–15) — `pid_controller.cpp`. Chưa build/chạy thử. |
| 2026-11-11 | 3 bộ PID + tốc độ cua riêng: PID chuẩn (thẳng) / Ép trái / Ép phải. Lệnh mới `CMD:PID:<kp>:<ki>:<kd>:<spd>:<turn>:<profile>`; web có ô chọn phần, bấm "NẠP PID" gửi cả 3 bộ, lưu localStorage. Xe chưa lưu flash (thay pin phải nạp lại). Chưa build/chạy thử. |
