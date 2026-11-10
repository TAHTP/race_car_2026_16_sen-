#ifndef RUN_LOG_H
#define RUN_LOG_H

#include <stdint.h>
#include <stdbool.h>

// ================================================================
// GHI LOG LƯỢT CHẠY 200 DÒNG/GIÂY + TRUYỀN QUA BLUETOOTH CLASSIC (SPP)
// - Trong lúc chạy: Core 1 chỉ ghi vào RAM (không gửi gì, không ảnh hưởng vòng PID).
// - Xe dừng: Core 0 tự gửi toàn bộ log dạng CSV qua SPP cho máy đang kết nối.
// - Bộ đệm vòng: nếu lượt chạy dài hơn sức chứa thì giữ phần CUỐI lượt chạy.
// ================================================================

#define RUNLOG_PERIOD_US   5000   // 5 ms = 200 dòng/giây
#define RUNLOG_MAX_RECORDS 6000   // Trần sức chứa (30 giây); thực tế tự co theo RAM còn trống

// Gọi 1 lần trong setup(), SAU ble_init(). Bật SPP và cấp phát bộ đệm.
void runlog_init(const char *bt_name);

// Core 1: gọi khi bắt đầu một lượt chạy (xóa log cũ)
void runlog_start(void);

// Core 1: gọi mỗi chu kỳ điều khiển khi xe đang chạy; hàm tự giới hạn 200 dòng/giây
void runlog_record(float pos, float corr, int16_t pwm_l, int16_t pwm_r,
                   float v_l_mms, float v_r_mms, uint16_t mask, uint32_t ticks,
                   uint8_t line_case, uint8_t action, uint8_t wp_idx, int16_t base_speed);

// Core 0: gọi liên tục trong bleTask. Trả về true khi đang gửi log (nên gọi lại ngay).
bool runlog_service(bool car_running);

#endif // RUN_LOG_H
