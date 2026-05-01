#ifndef MPU9250_H
#define MPU9250_H

#include <stdbool.h>
#include <stdint.h>

#include "ak8963.h"
#include "mpu6050.h"

typedef struct {
    MPU6050 mpu6050;
    AK8963 ak8963;
} MPU9250;

bool mpu9250_init(MPU9250 *sensor, i2c_inst_t *i2c);
bool mpu9250_get_acceleration(MPU9250 *sensor, mpu6050_vector_t *acceleration);
bool mpu9250_get_gyro(MPU9250 *sensor, mpu6050_vector_t *gyro);
bool mpu9250_get_temperature(MPU9250 *sensor, float *temperature_c);
bool mpu9250_get_magnetic(MPU9250 *sensor, ak8963_vector_t *magnetic);
bool mpu9250_get_whoami(MPU9250 *sensor, uint8_t *whoami);

#endif