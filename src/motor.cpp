#include "motor.h"
#include "config.h"
#include <Arduino.h>

static int8_t s_dir_motor_L = DEFAULT_DIR_MOTOR_L;
static int8_t s_dir_motor_R = DEFAULT_DIR_MOTOR_R;

static int16_t s_speed_L = 0;
static int16_t s_speed_R = 0;

static inline void drive_hbridge_channel(int pin1, int pin2, int speed) {
    if (speed > 0) {
        if (speed < 38) speed = 38; // Khử vùng chết ma sát tĩnh GA25
        speed = constrain(speed, 0, 255);
        ledcWrite(pin1, speed);
        ledcWrite(pin2, 0);
    } else if (speed < 0) {
        int abs_spd = abs(speed);
        if (abs_spd < 38) abs_spd = 38; // Khử vùng chết ma sát tĩnh GA25
        abs_spd = constrain(abs_spd, 0, 255);
        ledcWrite(pin1, 0);
        ledcWrite(pin2, abs_spd);
    } else {
        // Active Short Brake: Kích cả 2 chân cầu H lên cao để ghìm cứng động cơ tức thì
        ledcWrite(pin1, 255);
        ledcWrite(pin2, 255);
    }
}

void motor_init(void) {
    pinMode(IN_L1, OUTPUT);
    pinMode(IN_L2, OUTPUT);
    pinMode(IN_R1, OUTPUT);
    pinMode(IN_R2, OUTPUT);

    // Gắn PWM tần số cao 20kHz
    ledcAttach(IN_L1, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    ledcAttach(IN_L2, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    ledcAttach(IN_R1, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    ledcAttach(IN_R2, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);

    motor_stop();
}

void motor_stop(void) {
    motor_set_speeds(0, 0);
}

void motor_coast(void) {
    s_speed_L = 0;
    s_speed_R = 0;
    ledcWrite(IN_L1, 0);
    ledcWrite(IN_L2, 0);
    ledcWrite(IN_R1, 0);
    ledcWrite(IN_R2, 0);
}

void motor_set_speeds(int16_t left, int16_t right) {
    s_speed_L = left;
    s_speed_R = right;

    int cmd_L = left * s_dir_motor_L;
    int cmd_R = right * s_dir_motor_R;

    drive_hbridge_channel(IN_L1, IN_L2, cmd_L);
    drive_hbridge_channel(IN_R1, IN_R2, cmd_R);
}

void motor_set_left(int16_t speed) {
    motor_set_speeds(speed, s_speed_R);
}

void motor_set_right(int16_t speed) {
    motor_set_speeds(s_speed_L, speed);
}

int16_t motor_get_left_speed(void) {
    return s_speed_L;
}

int16_t motor_get_right_speed(void) {
    return s_speed_R;
}

void motor_set_dir(int8_t dir_left, int8_t dir_right) {
    s_dir_motor_L = dir_left;
    s_dir_motor_R = dir_right;
}

int8_t motor_get_dir_left(void) {
    return s_dir_motor_L;
}

int8_t motor_get_dir_right(void) {
    return s_dir_motor_R;
}
