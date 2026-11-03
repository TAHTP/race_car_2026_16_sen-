#include "pid_controller.h"
#include "sensor.h"
#include "motor.h"
#include <Arduino.h>

// =================== CẤU HÌNH TRỌNG SỐ CHO 16 MẮT ===================
// Tâm vạch lý tưởng nằm chính giữa mắt 7 và 8 (Error = 0.0)
// Lệch sang trái: Error < 0 | Lệch sang phải: Error > 0
static const float SENSOR_WEIGHTS[N_SENSORS] = {
    -75.0f, -65.0f, -55.0f, -45.0f, -35.0f, -25.0f, -15.0f, -5.0f, // 8 mắt trái (0..7)
      5.0f,  15.0f,  25.0f,  35.0f,  45.0f,  55.0f,  65.0f,  75.0f  // 8 mắt phải (8..15)
};

// Cấu hình PID chuẩn: Kp=2.2, Ki=0.0, Kd=10.0, baseSpeed=100
static PID_Config_t pidConfig = {
    .Kp = 2.2f,
    .Ki = 0.0f,
    .Kd = 10.0f,
    .baseSpeed = 100,
    .maxSpeed = 220,
    .minSpeed = -115,
    .turnSpeed = 130
};

static bool autoMode = false;
static LineCase_t currentCase = LINE_CASE_STOPPED;
static float lastError = 0.0f;
static float integralError = 0.0f;
static float lastCorrection = 0.0f;
static unsigned long lastComputeTime = 0;

#define DASHED_LINE_TIMEOUT_MS  60    // Thời gian cho phép lướt qua nét đứt (ms)
static unsigned long lostLineStartTime = 0;
static bool isLineCurrentlyLost = false;
static int8_t lastKnownDirection = 0; // -1: Trái, +1: Phải, 0: Giữa tâm
static float lastKnownPosition = 0.0f;// Vị trí vạch cuối cùng còn nhìn thấy

// Cơ chế chốt cua góc vuông 90° (Active 90° Corner Latch)
static uint8_t s_corner_latch = 0; // 0: None, 1: Turn Left, 2: Turn Right
static unsigned long s_corner_latch_time = 0;

static uint8_t s_sector_action = 0; // 0: Normal, 1: Force Left, 2: Force Right, 3: Lock Straight, 4: Brake

static inline int16_t clamp_int16(int16_t v, int16_t min_v, int16_t max_v) {
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

void pid_init(float kp, float ki, float kd, int16_t baseSpeed) {
    pidConfig.Kp = kp;
    pidConfig.Ki = ki;
    pidConfig.Kd = kd;
    pidConfig.baseSpeed = baseSpeed;
    lastError = 0.0f;
    integralError = 0.0f;
    lastComputeTime = millis();
    autoMode = false;
    currentCase = LINE_CASE_STOPPED;
    isLineCurrentlyLost = false;
    lastKnownDirection = 0;
    lastKnownPosition = 0.0f;
    s_corner_latch = 0;
    s_corner_latch_time = 0;
}

void pid_set_tunings(float kp, float ki, float kd) {
    pidConfig.Kp = kp;
    pidConfig.Ki = ki;
    pidConfig.Kd = kd;
}

void pid_set_base_speed(int16_t speed) {
    pidConfig.baseSpeed = clamp_int16(speed, 0, 255);
}

void pid_set_turn_speed(int16_t speed) {
    pidConfig.turnSpeed = clamp_int16(speed, 60, 220);
}

int16_t pid_get_turn_speed(void) {
    return pidConfig.turnSpeed;
}

void pid_set_sector_action(uint8_t action) {
    s_sector_action = action;
}

uint8_t pid_get_sector_action(void) {
    return s_sector_action;
}

void pid_set_auto_mode(bool enable) {
    autoMode = enable;
    isLineCurrentlyLost = false;
    lastKnownDirection = 0;
    lastKnownPosition = 0.0f;
    s_corner_latch = 0;
    s_corner_latch_time = 0;
    if (!autoMode) {
        currentCase = LINE_CASE_STOPPED;
        motor_stop();
    } else {
        integralError = 0.0f;
        lastError = 0.0f;
        lastComputeTime = millis();
    }
}

bool pid_is_auto_mode(void) {
    return autoMode;
}

bool pid_calculate_position(float *outPosition, uint8_t *outBlackCount) {
    sensor_read_all(); // Cập nhật đọc giá trị mới nhất từ 16 mắt ADC

    float sumWeights = 0.0f;
    uint32_t sumCalibrated = 0;
    uint8_t count = 0;

    int start_i = 0;
    int end_i = N_SENSORS - 1;
    if (s_sector_action == 1) { // ACTION_FORCE_LEFT: Ép chỉ đọc nhánh trái
        end_i = 8;
    } else if (s_sector_action == 2) { // ACTION_FORCE_RIGHT: Ép chỉ đọc nhánh phải
        start_i = 7;
    }

    // Quét các mắt theo kịch bản với giá trị chuẩn hóa Analog (0 - 1000)
    for (int i = start_i; i <= end_i; i++) {
        uint16_t cal = sensor_get_calibrated(i);

        // Lọc nhiễu nền: chỉ lấy tín hiệu thực sự có vạch đen (>= 150)
        if (cal >= 150) {
            sumCalibrated += cal;
            sumWeights += (float)cal * SENSOR_WEIGHTS[i];
            if (cal >= 500) {
                count++;
            }
        }
    }

    *outBlackCount = count;

    // Nếu tổng độ đen trên thanh cảm biến >= 300 -> Nhìn thấy vạch rõ ràng
    if (sumCalibrated >= 300) {
        float pos = sumWeights / (float)sumCalibrated; // Trọng tâm liên tục Smooth Centroid
        *outPosition = pos;

        // Cập nhật bộ nhớ hướng vạch
        lastKnownPosition = pos;
        if (pos < -15.0f) {
            lastKnownDirection = -1; // Vạch ở bên trái
        } else if (pos > 15.0f) {
            lastKnownDirection = 1;  // Vạch ở bên phải
        } else {
            lastKnownDirection = 0;  // Vạch ở chính giữa tâm
        }
        return true;
    }

    // Mất vạch hoàn toàn -> Giữ vị trí cuối cùng
    *outPosition = lastKnownPosition;
    return false;
}

LineCase_t pid_detect_case(float position, uint8_t blackCount) {
    if (!calibrationReady) {
        return LINE_CASE_STOPPED;
    }

    // Nếu đang trong trạng thái chốt cua góc vuông -> Giữ nguyên trạng thái này
    if (s_corner_latch == 1) return LINE_CASE_CORNER_LEFT;
    if (s_corner_latch == 2) return LINE_CASE_CORNER_RIGHT;

    // Kiểm tra vạch ngang / ngã tư / vạch đích / Diode (từ 10 mắt đen trở lên)
    uint16_t mask = sensor_get_line_bits();
    uint8_t totalBlack = __builtin_popcount(mask);
    if (totalBlack >= 10) {
        // NẾU MỐC SA BÀN ĐANG LÀ ÉP TRÁI (BẪY DIODE RẼ TRÁI / NGÃ 3 CHỮ T):
        // CẢM BIẾN CHẠM VÀO VẠCH NGANG -> KÍCH HOẠT BẺ VUÔNG TRÁI NGAY!
        if (s_sector_action == 1) {
            s_corner_latch = 1;
            s_corner_latch_time = millis();
            return LINE_CASE_CORNER_LEFT;
        }
        // NẾU MỐC SA BÀN ĐANG LÀ ÉP PHẢI (BẪY DIODE RẼ PHẢI / NGÃ 3 CHỮ T):
        // CẢM BIẾN CHẠM VÀO VẠCH NGANG -> KÍCH HOẠT BẺ VUÔNG PHẢI NGAY!
        if (s_sector_action == 2) {
            s_corner_latch = 2;
            s_corner_latch_time = millis();
            return LINE_CASE_CORNER_RIGHT;
        }

        // Mặc định: Ngã tư chữ thập -> Chạy thẳng vượt qua
        return LINE_CASE_CROSS_LINE;
    }

    // NHẬN DIỆN GÓC VUÔNG 90 ĐỘ CHỦ ĐỘNG (ACTIVE 90° CORNER DETECTION)
    // Áp dụng khi không khóa chạy thẳng (ACTION_LOCK_STRAIGHT) và không phanh đích (ACTION_BRAKE_STOP)
    if (s_sector_action != 3 && s_sector_action != 4) {
        uint8_t leftOuter = __builtin_popcount(mask & 0x000F);   // Cụm mắt 0,1,2,3 cực trái
        uint8_t rightOuter = __builtin_popcount(mask & 0xF000);  // Cụm mắt 12,13,14,15 cực phải

        // 1. Góc vuông Trái: Cụm mắt ngoài cùng bên trái nhìn thấy vạch ngang, mắt phải hoàn toàn trắng
        if (leftOuter >= 2 && rightOuter == 0 && (mask & 0x0003) != 0 && totalBlack <= 9) {
            s_corner_latch = 1;
            s_corner_latch_time = millis();
            return LINE_CASE_CORNER_LEFT;
        }

        // 2. Góc vuông Phải: Cụm mắt ngoài cùng bên phải nhìn thấy vạch ngang, mắt trái hoàn toàn trắng
        if (rightOuter >= 2 && leftOuter == 0 && (mask & 0xC000) != 0 && totalBlack <= 9) {
            s_corner_latch = 2;
            s_corner_latch_time = millis();
            return LINE_CASE_CORNER_RIGHT;
        }
    }

    // Kiểm tra mất vạch hoàn toàn
    if (blackCount == 0) {
        if (!isLineCurrentlyLost) {
            isLineCurrentlyLost = true;
            lostLineStartTime = millis();
        }

        unsigned long lostElapsed = millis() - lostLineStartTime;

        // Nếu vừa đi qua tâm thẳng mà mất vạch trong tích tắc (<= 60ms) -> Đoạn nét đứt!
        if (lastKnownDirection == 0 && lostElapsed <= DASHED_LINE_TIMEOUT_MS) {
            return LINE_CASE_DASHED_LINE;
        }

        return LINE_CASE_LOST_LINE;
    }

    isLineCurrentlyLost = false;
    return LINE_CASE_NORMAL_PID;
}

void pid_line_follow_step(void) {
    if (!autoMode) {
        return;
    }

    // Nếu sa bàn yêu cầu khóa thẳng phóng qua vạch đứt
    if (s_sector_action == 3) {
        motor_set_speeds(pidConfig.baseSpeed, pidConfig.baseSpeed);
        return;
    }
    // Nếu sa bàn yêu cầu phanh chốt đích
    if (s_sector_action == 4) {
        motor_stop();
        return;
    }

    // =========================================================================
    // XỬ LÝ CHỐT CUA GÓC VUÔNG 90 ĐỘ (PIVOT SPIN CỰC MẠNH)
    // =========================================================================
    if (s_corner_latch != 0) {
        unsigned long latchElapsed = millis() - s_corner_latch_time;
        uint16_t mask = sensor_get_line_bits();
        // Mắt tâm (6, 7, 8, 9) khóa lại line
        bool center_locked = ((mask & 0x03C0) != 0);

        if ((latchElapsed >= 35 && center_locked) || latchElapsed > 380) {
            s_corner_latch = 0;
            integralError = 0.0f;
            lastError = 0.0f;
        } else {
            int16_t t_fwd = pidConfig.turnSpeed;
            int16_t t_rev = - (int16_t)(pidConfig.turnSpeed * 80 / 100);
            if (s_corner_latch == 1) {
                // Xoay góc vuông Trái: Bánh trái giật lùi, bánh phải phóng tới
                motor_set_speeds(t_rev, t_fwd);
            } else {
                // Xoay góc vuông Phải: Bánh trái phóng tới, bánh phải giật lùi
                motor_set_speeds(t_fwd, t_rev);
            }
            return;
        }
    }

    float position = 0.0f;
    uint8_t blackCount = 0;
    pid_calculate_position(&position, &blackCount);

    // Xác định trạng thái
    currentCase = pid_detect_case(position, blackCount);

    int16_t t_fwd = pidConfig.turnSpeed;
    int16_t t_rev = - (int16_t)(pidConfig.turnSpeed * 80 / 100);

    // Nếu vừa kích hoạt chốt cua góc vuông trong pid_detect_case -> Thực thi ngay
    if (s_corner_latch != 0) {
        if (s_corner_latch == 1) {
            motor_set_speeds(t_rev, t_fwd);
        } else {
            motor_set_speeds(t_fwd, t_rev);
        }
        return;
    }

    unsigned long now = millis();
    float dt = (float)(now - lastComputeTime) / 1000.0f;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;
    lastComputeTime = now;

    switch (currentCase) {

        // =====================================================================
        // CASE 1: BÁM LINE PID CHÍNH (Đường thẳng, cua mềm, cua gắt, cua chữ S)
        // =====================================================================
        case LINE_CASE_NORMAL_PID: {
            float error = position; // -75.0 đến +75.0 (0.0 là chuẩn giữa tâm)
            float absErr = fabsf(error);

            // 1. Khâu P phi tuyến (Non-linear P):
            // Lệch ít (|error| <= 8): lái êm, xe không vẫy đầu
            // Lệch nhiều (|error| > 8): tự động tăng phản xạ bẻ lái kéo đầu xe về gấp
            float pGain = pidConfig.Kp * (1.0f + 0.012f * absErr);
            float pTerm = pGain * error;

            // 2. Khâu I có Anti-windup (kẹp trong khoảng [-150, 150])
            integralError += error * dt;
            if (integralError > 150.0f) integralError = 150.0f;
            if (integralError < -150.0f) integralError = -150.0f;
            float iTerm = pidConfig.Ki * integralError;

            // 3. Khâu D giảm chấn mạnh (Kd = 10.0 triệt tiêu hoàn toàn lắc lư đuôi xe)
            float deltaErr = error - lastError;
            lastError = error;
            float dTerm = pidConfig.Kd * deltaErr;

            // Tổng giá trị bù lái (Correction)
            lastCorrection = pTerm + iTerm + dTerm;

            // 4. Phân phối tốc độ ra 2 bánh:
            int16_t leftSpeed = 0;
            int16_t rightSpeed = 0;

            if (pidConfig.baseSpeed > 0) {
                // Khi xe chạy trên sàn: Giảm tốc ôm cua chủ động (Dynamic Corner Drag)
                int16_t currentBase = pidConfig.baseSpeed - (int16_t)(absErr * 0.55f);
                if (currentBase < 40) currentBase = 40;

                leftSpeed  = currentBase + (int16_t)lastCorrection;
                rightSpeed = currentBase - (int16_t)lastCorrection;

                // Giới hạn trong dải an toàn
                leftSpeed  = clamp_int16(leftSpeed, pidConfig.minSpeed, pidConfig.maxSpeed);
                rightSpeed = clamp_int16(rightSpeed, pidConfig.minSpeed, pidConfig.maxSpeed);
            } else {
                // CHẾ ĐỘ TEST CĂN TÂM TĨNH TRÊN BÀN (baseSpeed = 0):
                // Lệch trái -> bánh quay đưa về phải; Lệch phải -> bánh quay đưa về trái; Chuẩn tâm -> đứng yên!
                leftSpeed  = clamp_int16((int16_t)lastCorrection, -120, 120);
                rightSpeed = clamp_int16(-(int16_t)lastCorrection, -120, 120);
            }

            motor_set_speeds(leftSpeed, rightSpeed);
            break;
        }

        // =====================================================================
        // CASE: GÓC VUÔNG 90 ĐỘ TRÁI / PHẢI
        // =====================================================================
        case LINE_CASE_CORNER_LEFT: {
            motor_set_speeds(t_rev, t_fwd);
            break;
        }

        case LINE_CASE_CORNER_RIGHT: {
            motor_set_speeds(t_fwd, t_rev);
            break;
        }

        // =====================================================================
        // CASE 2: MẤT LINE (Dùng Line Memory xoay mạnh mẽ tìm lại)
        // =====================================================================
        case LINE_CASE_LOST_LINE: {
            unsigned long lostElapsed = millis() - lostLineStartTime;

            // Mất vạch quá 1.5 giây -> Phanh dừng xe an toàn
            if (lostElapsed > 1500) {
                motor_stop();
                break;
            }

            // Xoay tại chỗ theo hướng nhìn thấy vạch lần cuối với lực mạnh
            if (lastKnownDirection < 0 || lastKnownPosition < -5.0f || lastError < -5.0f) {
                motor_set_speeds(t_rev, t_fwd); // Giật xoay mạnh tìm vạch bên trái
            } else if (lastKnownDirection > 0 || lastKnownPosition > 5.0f || lastError > 5.0f) {
                motor_set_speeds(t_fwd, t_rev); // Giật xoay mạnh tìm vạch bên phải
            } else {
                // Nếu hướng = 0 (trước đó đi thẳng), dùng lastKnownPosition để quyết định xoay
                if (lastKnownPosition <= 0.0f) {
                    motor_set_speeds(t_rev, t_fwd);
                } else {
                    motor_set_speeds(t_fwd, t_rev);
                }
            }
            break;
        }

        // =====================================================================
        // CASE 3: VẠCH NGANG / NGÃ TƯ
        // =====================================================================
        case LINE_CASE_CROSS_LINE: {
            motor_set_speeds(pidConfig.baseSpeed, pidConfig.baseSpeed);
            break;
        }

        // =====================================================================
        // CASE 4: ĐOẠN NÉT ĐỨT (DASHED LINE)
        // =====================================================================
        case LINE_CASE_DASHED_LINE: {
            motor_set_speeds(pidConfig.baseSpeed, pidConfig.baseSpeed);
            break;
        }

        // =====================================================================
        // CASE 5: DỪNG XE
        // =====================================================================
        case LINE_CASE_STOPPED:
        default: {
            motor_stop();
            break;
        }
    }
}

LineCase_t pid_get_current_case(void) {
    return currentCase;
}

const char* pid_get_case_name(LineCase_t c) {
    switch (c) {
        case LINE_CASE_NORMAL_PID:   return "NORMAL_PID";
        case LINE_CASE_LOST_LINE:    return "LOST_LINE";
        case LINE_CASE_CROSS_LINE:   return "CROSS_LINE";
        case LINE_CASE_DASHED_LINE:  return "DASHED_LINE";
        case LINE_CASE_CORNER_LEFT:  return "CORNER_LEFT";
        case LINE_CASE_CORNER_RIGHT: return "CORNER_RIGHT";
        case LINE_CASE_STOPPED:      return "STOPPED";
        default: return "UNKNOWN";
    }
}

float pid_get_last_error(void) {
    return lastError;
}

float pid_get_correction(void) {
    return lastCorrection;
}

float pid_get_kp(void) {
    return pidConfig.Kp;
}

float pid_get_ki(void) {
    return pidConfig.Ki;
}

float pid_get_kd(void) {
    return pidConfig.Kd;
}

int16_t pid_get_base_speed(void) {
    return pidConfig.baseSpeed;
}
