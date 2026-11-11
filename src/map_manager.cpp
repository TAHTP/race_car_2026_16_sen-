#include "map_manager.h"
#include "ble_handler.h"
#include "pid_controller.h"
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
    prefs.begin("race_maps_v3", false);
    prefs.putBytes("map_data", (const void*)s_maps, sizeof(s_maps));
    prefs.putUChar("sel_map", s_selected_map);
    prefs.putBool("is_right", s_is_right_lane);
    prefs.putFloat("def_kp", pid_get_default_kp());
    prefs.putFloat("def_kd", pid_get_default_kd());
    prefs.putShort("turn_spd", pid_get_turn_speed());
    prefs.putShort("scan_spd", car_get_scan_speed());
    prefs.end();
}

bool map_load_from_flash(void) {
    prefs.begin("race_maps_v3", true);
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

    if (prefs.isKey("def_kp")) {
        float kp = prefs.getFloat("def_kp", pid_get_default_kp());
        float kd = prefs.getFloat("def_kd", pid_get_default_kd());
        pid_set_default_tunings(kp, 0.0f, kd);
    }
    if (prefs.isKey("turn_spd")) {
        pid_set_turn_speed(prefs.getShort("turn_spd", 130));
    }
    if (prefs.isKey("scan_spd")) {
        car_set_scan_speed(prefs.getShort("scan_spd", 120));
    }

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

bool map_set_waypoint(uint8_t map_idx, uint8_t wp_idx, uint32_t ticks, int16_t speed, uint8_t action, float kp, float kd) {
    if (map_idx >= NUM_MAPS || wp_idx >= MAX_WAYPOINTS) return false;

    s_maps[map_idx].wps[wp_idx].start_ticks = ticks;
    s_maps[map_idx].wps[wp_idx].target_speed = speed;
    s_maps[map_idx].wps[wp_idx].action = action;
    s_maps[map_idx].wps[wp_idx].kp = kp;
    s_maps[map_idx].wps[wp_idx].kd = kd;

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

void map_get_current_action(uint32_t current_ticks, int16_t *out_speed, uint8_t *out_action, float *out_kp, float *out_kd, uint8_t *out_wp_idx) {
    MapProfile_t *prof = &s_maps[s_selected_map];
    float def_kp = pid_get_default_kp();
    float def_kd = pid_get_default_kd();

    // 1. Nếu không có mốc nào: Chạy PID thuần với tốc độ nền và PID mặc định
    if (prof->count == 0) {
        *out_speed = car_get_scan_speed();
        *out_action = ACTION_NORMAL;
        *out_kp = def_kp;
        *out_kd = def_kd;
        *out_wp_idx = 0;
        return;
    }

    // 2. Nếu xe chưa chạy tới Mốc Bắt Đầu đầu tiên (current_ticks < mốc 0):
    // Xe xuất phát chạy PID bình thường theo tốc độ nền
    if (current_ticks < prof->wps[0].start_ticks) {
        *out_speed = car_get_scan_speed();
        *out_action = ACTION_NORMAL;
        *out_kp = def_kp;
        *out_kd = def_kd;
        *out_wp_idx = 0;
        return;
    }

    // 3. Tìm mốc BẮT ĐẦU đang có hiệu lực (quét ngược từ mốc lớn nhất về mốc 0):
    // Mốc nào có start_ticks <= current_ticks đầu tiên chính là mốc đang thực thi!
    for (int8_t i = (int8_t)(prof->count - 1); i >= 0; i--) {
        if (current_ticks >= prof->wps[i].start_ticks) {
            *out_speed = prof->wps[i].target_speed;
            uint8_t act = prof->wps[i].action;

            // Nếu xe đặt ở Làn Phải (Mirror Track): Đảo ngược hướng ép Trái <-> Phải
            if (s_is_right_lane) {
                if (act == ACTION_FORCE_LEFT) act = ACTION_FORCE_RIGHT;
                else if (act == ACTION_FORCE_RIGHT) act = ACTION_FORCE_LEFT;
            }

            *out_action = act;
            *out_kp = (prof->wps[i].kp > 0.01f) ? prof->wps[i].kp : def_kp;
            *out_kd = (prof->wps[i].kd > 0.01f) ? prof->wps[i].kd : def_kd;
            *out_wp_idx = (uint8_t)i;
            return;
        }
    }

    *out_speed = car_get_scan_speed();
    *out_action = ACTION_NORMAL;
    *out_kp = def_kp;
    *out_kd = def_kd;
    *out_wp_idx = 0;
}
