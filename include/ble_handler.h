#ifndef BLE_HANDLER_H
#define BLE_HANDLER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Khởi tạo BLE Server trên Core 0
void ble_init(const char *device_name);

// Kiểm tra trạng thái kết nối
bool ble_is_connected(void);

// Bắn dữ liệu Telemetry về Web BLE
void ble_send_telemetry(int32_t ticks_l, int32_t ticks_r, float line_pos, 
                        int16_t pwm_l, int16_t pwm_r, 
                        uint8_t map_idx, uint8_t wp_idx, uint8_t car_state);

// Bắn chuỗi thông báo chung hoặc danh sách Map về Web
void ble_send_string(const char *str);
void ble_send_map_data(uint8_t map_idx);

#ifdef __cplusplus
}
#endif

#endif // BLE_HANDLER_H
