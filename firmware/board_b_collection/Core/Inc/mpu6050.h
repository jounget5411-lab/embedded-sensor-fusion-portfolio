/*
 * mpu6050.h
 *
 *  Created on: Nov 23, 2025
 *      Author: nayeong
 */

#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "stm32f1xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

void mpu6050_init(void);
void mpu6050_read_accel(float *ax, float *ay, float *az);
void mpu6050_read_gyro(float *gx, float *gy, float *gz);

#endif
