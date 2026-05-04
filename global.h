#ifndef GLOBAL_H
#define GLOBAL_H

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/spi.h"

#include "ak8963.h"
#include "camera.h"
#include "colorsensor.h"
#include "compass.h"
#include "encoder.h"
#include "hcsr04.h"
#include "motornoencoder.h"
#include "mpu6050.h"
#include "mpu9250.h"
#include "servo.h"

// // ===== GLOBAL STATE =====
extern volatile int leftCount;
extern volatile int rightCount;
extern volatile int x_pos; // starting position: middle of field
extern volatile int y_pos;
extern volatile int heading;
extern int max_x;    //we can move roughly 205cm to the right from 0,0
extern int min_x;
extern int max_y;    //we can move roughly 245cm towards the enemy
extern int min_y;

// extern MotorNoEncoder motor;
extern TCS34725 sensor;
extern Servo scoop;
extern Servo sorter;
extern Servo chamber_stop;
extern Compass compass;

extern Encoder left_enc;
extern Encoder right_enc;

void sleepcheck(int x);
void turn(float desiredHeading);

#endif
