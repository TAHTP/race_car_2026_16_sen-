#include "map_manager.h"
#include "ble_handler.h"
#include <Arduino.h>
#include <Preferences.h>

static Preferences prefs;
static MapProfile_t s_maps[NUM_MAPS];
static uint8_t s_selected_map = 0;
static bool s_is_right_lane = false;

void map_load_defaults(void) {
    // -------------------------------------------------------------
    // KHỞI TẠO BẢNG MỐC TRỐNG BAN ĐẦU (0 MỐC ẢO)
    // Xe sẽ chạy 100% theo "Tốc độ nền" người dùng cài đặt trên Web!
    // Chỉ khi nào người dùng bấm "Thêm mốc" và "Lưu vào xe" trên Web
    // thì xe mới áp dụng các mốc sa bàn tùy biến.
    // -------------------------------------------------------------
    for (uint8_t m = 0; m < NUM_MAPS; m++) {
        s_maps[m].count = 0;
    }
}

void map_manager_init(void) {
    if (!map_load_from_flash()) {
        map_load_defaults();
        map_save_to_flash();
    }
}

void map_save_to_flash(void) {
    prefs.begin("race_maps_v2", false);
    prefs.putBytes("map_data", (const void*)s_maps, sizeof(s_maps));
    prefs.putUChar("sel_map", s_selected_map);
    prefs.putBool("is_right", s_is_right_lane);
    prefs.end();
}

bool map_load_from_flash(void) {
    prefs.begin("race_maps_v2", true);
    if (!prefs.isKey("map_data")) {
        prefs.end();
        return false;
    }
    size_t len = prefs.getBytes("map_data", (void*)s_maps, sizeof(s_maps));
    if (len != sizeof(s_maps)) {
        prefs.end();
        return false;
    }
    s_selected_map = prefs.getUChar("sel_map", 0);
    if (s_selected_map >= NUM_MAPS) s_selected_map = 0;
    s_is_right_lane = prefs.getBool("is_right", false);
    prefs.end();
    return true;
}

void map_select(uint8_t map_idx) {
    if (map_idx < NUM_MAPS) {
        s_selected_map = map_idx;
    }
}

uint8_t map_get_selected(void) {
    return s_selected_map;
}

void map_set_right_lane(bool is_right) {
    s_is_right_lane = is_right;
}

bool map_is_right_lane(void) {
    return s_is_right_lane;
}

bool map_set_waypoint(uint8_t map_idx, uint8_t wp_idx, uint32_t ticks, int16_t speed, uint8_t action) {
    if (map_idx >= NUM_MAPS || wp_idx >= MAX_WAYPOINTS) return false;

    s_maps[map_idx].wps[wp_idx].end_ticks = ticks;
    s_maps[map_idx].wps[wp_idx].target_speed = speed;
    s_maps[map_idx].wps[wp_idx].action = action;

    if (wp_idx >= s_maps[map_idx].count) {
        s_maps[map_idx].count = wp_idx + 1;
    }
    return true;
}

void map_clear_waypoints(uint8_t map_idx) {
    if (map_idx < NUM_MAPS) {
        s_maps[map_idx].count = 0;
    }
}

uint8_t map_get_waypoint_count(uint8_t map_idx) {
    if (map_idx < NUM_MAPS) return s_maps[map_idx].count;
    return 0;
}

const Waypoint_t* map_get_waypoint(uint8_t map_idx, uint8_t wp_idx) {
    if (map_idx < NUM_MAPS && wp_idx < s_maps[map_idx].count) {
        return &s_maps[map_idx].wps[wp_idx];
    }
    return NULL;
}

void map_get_current_action(uint32_t current_ticks, int16_t *out_speed, uint8_t *out_action, uint8_t *out_wp_idx) {
    MapProfile_t *prof = &s_maps[s_selected_map];

    if (prof->count == 0) {
        *out_speed = car_get_scan_speed();
        *out_action = ACTION_NORMAL;
        *out_wp_idx = 0;
        return;
    }

    // Tìm phân đoạn mà current_ticks đang nằm trong
    for (uint8_t i = 0; i < prof->count; i++) {
        if (current_ticks <= prof->wps[i].end_ticks) {
            *out_speed = prof->wps[i].target_speed;
            uint8_t act = prof->wps[i].action;

            // Nếu xe đặt ở Làn Phải (Mirror Track): Đảo ngược hướng ép Trái <-> Phải
            if (s_is_right_lane) {
                if (act == ACTION_FORCE_LEFT) act = ACTION_FORCE_RIGHT;
                else if (act == ACTION_FORCE_RIGHT) act = ACTION_FORCE_LEFT;
            }

            *out_action = act;
            *out_wp_idx = i;
            return;
        }
    }

    // Nếu đã chạy vượt qua mốc cuối cùng -> Tiếp tục chạy PID thuần với tốc độ của mốc cuối, KHÔNG TỰ DỪNG!
    // Xe sẽ tiếp tục bám line và tự bẻ góc vuông y hệt Mode 1 cho đến khi chạm vạch đích thật sự!
    *out_speed = prof->wps[prof->count - 1].target_speed;
    *out_action = ACTION_NORMAL;
    *out_wp_idx = prof->count;
}
