#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include <stdint.h>

#include "hardware/i2c.h"

#define MPU6050_ACCEL_FS_SEL_2G 0x00
#define MPU6050_ACCEL_FS_SEL_4G 0x08
#define MPU6050_ACCEL_FS_SEL_8G 0x10
#define MPU6050_ACCEL_FS_SEL_16G 0x18

#define MPU6050_GYRO_FS_SEL_250DPS 0x00
#define MPU6050_GYRO_FS_SEL_500DPS 0x08
#define MPU6050_GYRO_FS_SEL_1000DPS 0x10
#define MPU6050_GYRO_FS_SEL_2000DPS 0x18

#define MPU6050_SF_G 1.0f
#define MPU6050_SF_M_S2 9.80665f
#define MPU6050_SF_DEG_S 1.0f
#define MPU6050_SF_RAD_S 0.017453292519943f

typedef struct {
    float x;
    float y;
    float z;
} mpu6050_vector_t;

typedef struct {
    i2c_inst_t *i2c;
    uint8_t address;
    float accel_so;
    float gyro_so;
    float accel_sf;
    float gyro_sf;
    mpu6050_vector_t gyro_offset;
} MPU6050;

typedef MPU6050 MPU6500;

bool mpu6050_init(
    MPU6050 *sensor,
    i2c_inst_t *i2c,
    uint8_t address,
    uint8_t accel_fs,
    uint8_t gyro_fs,
    float accel_sf,
    float gyro_sf,
    mpu6050_vector_t gyro_offset
);
bool mpu6050_get_acceleration(MPU6050 *sensor, mpu6050_vector_t *acceleration);
bool mpu6050_get_gyro(MPU6050 *sensor, mpu6050_vector_t *gyro);
bool mpu6050_get_temperature(MPU6050 *sensor, float *temperature_c);
bool mpu6050_get_whoami(MPU6050 *sensor, uint8_t *whoami);
bool mpu6050_calibrate_gyro(MPU6050 *sensor, int count, int delay_ms, mpu6050_vector_t *gyro_offset);
bool mpu6050_read_u8(MPU6050 *sensor, uint8_t reg, uint8_t *value);
bool mpu6050_write_u8(MPU6050 *sensor, uint8_t reg, uint8_t value);

#define mpu6500_init mpu6050_init
#define mpu6500_get_acceleration mpu6050_get_acceleration
#define mpu6500_get_gyro mpu6050_get_gyro
#define mpu6500_get_temperature mpu6050_get_temperature
#define mpu6500_get_whoami mpu6050_get_whoami
#define mpu6500_calibrate_gyro mpu6050_calibrate_gyro
#define mpu6500_read_u8 mpu6050_read_u8
#define mpu6500_write_u8 mpu6050_write_u8

#endif