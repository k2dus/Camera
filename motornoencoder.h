#ifndef MOTORNOENCODER_H
#define MOTORNOENCODER_H

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_NOENCODER_NO_FAULT_PIN ((unsigned int)~0u)

typedef struct {
    unsigned int lfwd_pin;
    unsigned int rfwd_pin;
    unsigned int lrev_pin;
    unsigned int rrev_pin;
    unsigned int right_fault_pin;
    unsigned int left_fault_pin;
    unsigned int pwm_wrap;
    unsigned int reverse_delay_ms;
    bool last_ldir;
    bool last_rdir;
    bool has_last_dir;
    bool overcurrent_right;
    bool overcurrent_left;
    int oc_count_r;
    int oc_count_l;
    int oc_threshold;
} MotorNoEncoder;

void motor_noencoder_init(
    MotorNoEncoder *motor,
    unsigned int lfwd_pin,
    unsigned int rfwd_pin,
    unsigned int lrev_pin,
    unsigned int rrev_pin,
    unsigned int right_fault_pin,
    unsigned int left_fault_pin,
    unsigned int pwm_frequency_hz,
    unsigned int reverse_delay_ms,
    int oc_threshold
);
void motor_noencoder_check_overcurrent(MotorNoEncoder *motor);
void motor_noencoder_stop_all(MotorNoEncoder *motor);
void motor_noencoder_move(MotorNoEncoder *motor, float speed, bool left_direction, bool right_direction);

#endif