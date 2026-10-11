/*
 * mpu6050.c
 *
 *  Created on: Nov 23, 2025
 *      Author: nayeong
 */

#include "mpu6050.h"

#define MPU6050_ADDR 0xD0   // 0x68 << 1

void mpu6050_init(void)
{
    uint8_t data;

    // Sleep off
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &data, 1, 100);

    // Gyro ±250deg/s
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1B, 1, &data, 1, 100);

    // Accel ±2g
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1C, 1, &data, 1, 100);
}

void mpu6050_read_accel(float *ax, float *ay, float *az)
{
    uint8_t buf[6];
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, buf, 6, 100);

    int16_t x = (buf[0] << 8) | buf[1];
    int16_t y = (buf[2] << 8) | buf[3];
    int16_t z = (buf[4] << 8) | buf[5];

    *ax = x / 16384.0f;
    *ay = y / 16384.0f;
    *az = z / 16384.0f;
}

void mpu6050_read_gyro(float *gx, float *gy, float *gz)
{
    uint8_t buf[6];
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x43, 1, buf, 6, 100);

    int16_t x = (buf[0] << 8) | buf[1];
    int16_t y = (buf[2] << 8) | buf[3];
    int16_t z = (buf[4] << 8) | buf[5];

    *gx = x / 131.0f;
    *gy = y / 131.0f;
    *gz = z / 131.0f;
}

