#include "motornoencoder.h"
#include "global.h"
#include <stdio.h>
#include <math.h>

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

static bool motor_noencoder_fault_pin_valid(unsigned int pin) {
    return pin != MOTOR_NOENCODER_NO_FAULT_PIN;
}

static void motor_noencoder_setup_pwm_pin(unsigned int pin, unsigned int wrap, float clkdiv) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    unsigned int slice = pwm_gpio_to_slice_num(pin);
    pwm_set_clkdiv(slice, clkdiv);
    pwm_set_wrap(slice, wrap);
    pwm_set_enabled(slice, true);
    pwm_set_gpio_level(pin, 0);
}

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
) {
    motor->lfwd_pin = lfwd_pin;
    motor->rfwd_pin = rfwd_pin;
    motor->lrev_pin = lrev_pin;
    motor->rrev_pin = rrev_pin;
    motor->right_fault_pin = right_fault_pin;
    motor->left_fault_pin = left_fault_pin;
    motor->reverse_delay_ms = reverse_delay_ms;
    motor->has_last_dir = false;
    motor->overcurrent_right = false;
    motor->overcurrent_left = false;
    motor->oc_count_r = 0;
    motor->oc_count_l = 0;
    motor->oc_threshold = oc_threshold;

    motor->pwm_wrap = 1000000u / pwm_frequency_hz;
    float clkdiv = 125.0f;

    motor_noencoder_setup_pwm_pin(lfwd_pin, motor->pwm_wrap, clkdiv);
    motor_noencoder_setup_pwm_pin(rfwd_pin, motor->pwm_wrap, clkdiv);
    motor_noencoder_setup_pwm_pin(lrev_pin, motor->pwm_wrap, clkdiv);
    motor_noencoder_setup_pwm_pin(rrev_pin, motor->pwm_wrap, clkdiv);

    if (motor_noencoder_fault_pin_valid(right_fault_pin)) {
        gpio_init(right_fault_pin);
        gpio_set_dir(right_fault_pin, GPIO_IN);
        gpio_pull_up(right_fault_pin);
    }
    if (motor_noencoder_fault_pin_valid(left_fault_pin)) {
        gpio_init(left_fault_pin);
        gpio_set_dir(left_fault_pin, GPIO_IN);
        gpio_pull_up(left_fault_pin);
    }
}

void motor_noencoder_check_overcurrent(MotorNoEncoder *motor) {
    if (!motor_noencoder_fault_pin_valid(motor->right_fault_pin)) {
        motor->oc_count_r = 0;
        motor->overcurrent_right = false;
    } else if (gpio_get(motor->right_fault_pin)) {
        motor->oc_count_r++;
        if (motor->oc_count_r >= motor->oc_threshold) {
            motor->overcurrent_right = true;
        }
    } else {
        motor->oc_count_r = 0;
        motor->overcurrent_right = false;
    }

    if (!motor_noencoder_fault_pin_valid(motor->left_fault_pin)) {
        motor->oc_count_l = 0;
        motor->overcurrent_left = false;
    } else if (gpio_get(motor->left_fault_pin)) {
        motor->oc_count_l++;
        if (motor->oc_count_l >= motor->oc_threshold) {
            motor->overcurrent_left = true;
        }
    } else {
        motor->oc_count_l = 0;
        motor->overcurrent_left = false;
    }
}

void motor_noencoder_stop_all(MotorNoEncoder *motor) {
    pwm_set_gpio_level(motor->lfwd_pin, 0);
    pwm_set_gpio_level(motor->rfwd_pin, 0);
    pwm_set_gpio_level(motor->lrev_pin, 0);
    pwm_set_gpio_level(motor->rrev_pin, 0);
}

void motor_noencoder_move(MotorNoEncoder *motor, float speed, bool left_direction, bool right_direction) {
    if (motor->has_last_dir &&
        ((left_direction != motor->last_ldir) || (right_direction != motor->last_rdir))) {
        motor_noencoder_stop_all(motor);
        sleep_ms(motor->reverse_delay_ms);
    }

    if (speed <= 0.0f) {
        motor_noencoder_stop_all(motor);
        return;
    }
    if (speed > 1.0f) {
        speed = 1.0f;
    }

    unsigned int duty = (unsigned int)(speed * motor->pwm_wrap);
    pwm_set_gpio_level(motor->lfwd_pin, 0);
    pwm_set_gpio_level(motor->rfwd_pin, 0);
    pwm_set_gpio_level(motor->lrev_pin, 0);
    pwm_set_gpio_level(motor->rrev_pin, 0);

    if (motor->overcurrent_right) {
        printf("Right Side Overcurrent\n");
    } else if (left_direction) {
        pwm_set_gpio_level(motor->lfwd_pin, duty);
    } else {
        pwm_set_gpio_level(motor->lrev_pin, duty);
    }

    if (motor->overcurrent_left) {
        printf("Left Side Overcurrent\n");
    } else if (right_direction) {
        pwm_set_gpio_level(motor->rfwd_pin, duty);
    } else {
        pwm_set_gpio_level(motor->rrev_pin, duty);
    }

    motor->last_ldir = left_direction;
    motor->last_rdir = right_direction;
    motor->has_last_dir = true;
    printf("xpos: %d"
           "ypos: %d", x_pos, y_pos);
}