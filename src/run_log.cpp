#include "run_log.h"

#include <Arduino.h>
#include <esp_heap_caps.h>
#include "ble_handler.h"

#pragma pack(push, 1)
typedef struct {
    uint16_t t_ms;      // thời gian từ lúc xuất phát (tràn sau 65 s, được nối lại khi gửi)
    int16_t  pos10;     // vị trí vạch x10
    int16_t  corr;      // giá trị bù lái PID
    int16_t  pwm_l, pwm_r;
    int16_t  v_l, v_r;  // tốc độ thật từng bánh (mm/s) theo encoder
    uint16_t mask;      // 16 mắt
    uint16_t ticks;     // xung trung bình (bão hòa 65535)
    uint8_t  line_case;
    uint8_t  act_wp;    // 4 bit cao: action, 4 bit thấp: chỉ số mốc
    uint8_t  base;      // tốc độ nền đang áp dụng
} LogRec_t;
#pragma pack(pop)

#define HEAP_RESERVE_BYTES  40000  // RAM chừa lại cho BLE và các tác vụ khác
#define CHUNK_MAX_BYTES     190    // Mỗi gói BLE notify (dưới MTU 247 - 3)
#define CHUNK_PACE_MS       20     // Nghỉ giữa các gói để BLE không bị tràn hàng đợi

static LogRec_t *s_buf = NULL;
static uint32_t s_cap = 0;
static volatile uint32_t s_head = 0;     // vị trí ghi kế tiếp
static volatile uint32_t s_count = 0;    // số bản ghi hợp lệ
static volatile uint32_t s_dropped = 0;  // số bản ghi đầu lượt bị ghi đè
static volatile uint32_t s_gen = 0;      // tăng mỗi lượt chạy, dùng để hủy lượt gửi dở
static int64_t s_start_us = 0;
static int64_t s_next_us = 0;

// Trạng thái gửi (chỉ dùng ở Core 0)
static bool s_was_running = false;
static bool s_pending = false;
static bool s_dumping = false;
static uint32_t s_dump_gen = 0;
static uint32_t s_dump_idx = 0;
static uint32_t s_dump_total = 0;
static uint32_t s_dump_first = 0;
static uint32_t s_dump_t = 0;
static uint16_t s_dump_prev_t = 0;
static uint32_t s_last_tx = 0;
static bool s_end_pending = false;

static inline int16_t sat16(float v) {
    if (v > 32767.0f) return 32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)v;
}

void runlog_init(void) {
    // Cấp phát bộ đệm theo RAM còn trống sau khi BLE đã khởi động
    size_t free_heap = ESP.getFreeHeap();
    size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    size_t usable = (free_heap > HEAP_RESERVE_BYTES) ? (free_heap - HEAP_RESERVE_BYTES) : 0;
    if (usable > largest) usable = largest;

    uint32_t cap = (uint32_t)(usable / sizeof(LogRec_t));
    if (cap > RUNLOG_MAX_RECORDS) cap = RUNLOG_MAX_RECORDS;
    if (cap >= 200) {
        s_buf = (LogRec_t *)malloc(cap * sizeof(LogRec_t));
        if (s_buf) s_cap = cap;
    }

    Serial.printf(">> 📝 [LOG] RAM trống %u B | Sức chứa %u dòng (%.1f giây ở 200 dòng/giây)\n",
                  (unsigned)free_heap, (unsigned)s_cap,
                  (float)s_cap * (RUNLOG_PERIOD_US / 1000000.0f));
}

void runlog_start(void) {
    s_dumping = false;
    s_end_pending = false;
    s_gen++;
    s_head = 0;
    s_count = 0;
    s_dropped = 0;
    s_start_us = esp_timer_get_time();
    s_next_us = s_start_us;
}

void runlog_record(float pos, float corr, int16_t pwm_l, int16_t pwm_r,
                   float v_l_mms, float v_r_mms, uint16_t mask, uint32_t ticks,
                   uint8_t line_case, uint8_t action, uint8_t wp_idx, int16_t base_speed) {
    if (!s_buf) return;

    int64_t now = esp_timer_get_time();
    if (now < s_next_us) return;
    s_next_us += RUNLOG_PERIOD_US;
    if (s_next_us < now) s_next_us = now + RUNLOG_PERIOD_US; // bị trễ nhiều thì bắt nhịp lại

    LogRec_t *r = &s_buf[s_head];
    r->t_ms = (uint16_t)((now - s_start_us) / 1000);
    r->pos10 = sat16(pos * 10.0f);
    r->corr = sat16(corr);
    r->pwm_l = pwm_l;
    r->pwm_r = pwm_r;
    r->v_l = sat16(v_l_mms);
    r->v_r = sat16(v_r_mms);
    r->mask = mask;
    r->ticks = (ticks > 65535u) ? 65535u : (uint16_t)ticks;
    r->line_case = line_case;
    r->act_wp = (uint8_t)(((action & 0x0F) << 4) | (wp_idx & 0x0F));
    r->base = (base_speed < 0) ? 0 : (base_speed > 255 ? 255 : (uint8_t)base_speed);

    uint32_t h = s_head + 1;
    if (h >= s_cap) h = 0;
    s_head = h;
    if (s_count < s_cap) s_count++;
    else s_dropped++;
}

void runlog_request_dump(void) {
    s_pending = true; // Web xin gửi lại log của lượt chạy gần nhất
}

static void send_raw(const char *buf, size_t len) {
    ble_send_raw((const uint8_t *)buf, len);
}

bool runlog_service(bool car_running) {
    if (car_running) {
        s_was_running = true;
        s_dumping = false; // xe chạy lượt mới thì hủy lượt gửi dở
        return false;
    }
    if (s_was_running) { // xe vừa dừng -> hẹn gửi log
        s_was_running = false;
        s_pending = true;
    }

    if (!ble_is_connected()) {
        s_dumping = false; // mất kết nối giữa chừng: giữ s_pending để gửi lại khi kết nối
        return false;
    }

    // Nghỉ giữa các gói để BLE không tràn hàng đợi
    if ((s_dumping || s_pending || s_end_pending) && millis() - s_last_tx < CHUNK_PACE_MS) return true;

    if (s_end_pending) { // gói kết thúc, báo tổng số dòng để web kiểm tra thiếu
        s_end_pending = false;
        char tail[24];
        int n = snprintf(tail, sizeof(tail), "RE:%u", (unsigned)s_dump_total);
        send_raw(tail, n);
        s_last_tx = millis();
        Serial.printf(">> 📝 [LOG] Đã gửi %u dòng qua BLE\n", (unsigned)s_dump_total);
        return false;
    }

    if (!s_dumping) {
        if (!s_pending) return false;
        s_pending = false;
        s_last_tx = millis();
        if (s_count == 0) {
            send_raw("RX", 2); // chưa có log nào
            return false;
        }
        s_dump_gen = s_gen;
        s_dump_total = s_count;
        s_dump_first = (s_head + s_cap - s_count) % s_cap;
        s_dump_idx = 0;
        s_dump_t = s_buf[s_dump_first].t_ms;
        s_dump_prev_t = s_buf[s_dump_first].t_ms;
        s_dumping = true;

        char hdr[48];
        int n = snprintf(hdr, sizeof(hdr), "RB:%u,%u,%u",
                         (unsigned)s_dump_total, (unsigned)s_dropped, (unsigned)(RUNLOG_PERIOD_US / 1000));
        send_raw(hdr, n);
        return true;
    }

    if (s_dump_gen != s_gen) { // log đã bị lượt chạy mới ghi đè
        s_dumping = false;
        return false;
    }

    // Mỗi gói BLE: "R:" + nhiều dòng CSV nguyên vẹn, ngăn bằng '\n'
    char out[CHUNK_MAX_BYTES + 8];
    int len = 0;
    out[len++] = 'R';
    out[len++] = ':';
    while (s_dump_idx < s_dump_total && len + 72 <= CHUNK_MAX_BYTES) {
        const LogRec_t *r = &s_buf[(s_dump_first + s_dump_idx) % s_cap];
        s_dump_t += (uint16_t)(r->t_ms - s_dump_prev_t); // nối lại thời gian khi t_ms tràn 16 bit
        s_dump_prev_t = r->t_ms;
        int ap = abs((int)r->pos10);
        len += snprintf(out + len, sizeof(out) - len, "%u,%s%d.%d,%d,%d,%d,%d,%d,%u,%u,%u,%u,%u,%u\n",
                        (unsigned)s_dump_t,
                        (r->pos10 < 0) ? "-" : "", ap / 10, ap % 10,
                        r->corr, r->pwm_l, r->pwm_r, r->v_l, r->v_r,
                        (unsigned)r->mask, (unsigned)r->ticks, (unsigned)r->line_case,
                        (unsigned)(r->act_wp >> 4), (unsigned)(r->act_wp & 0x0F), (unsigned)r->base);
        s_dump_idx++;
    }
    send_raw(out, len);
    s_last_tx = millis();

    if (s_dump_idx >= s_dump_total) {
        s_dumping = false;
        s_end_pending = true;
    }
    return true;
}
