/*
 * feature.c
 *
 *  Created on: Nov 23, 2025
 *      Author: nayeong
 */

#include "feature.h"
#include "flex_sensor.h"
#include "mpu6050.h"

void extract_feature(float *feature)
{
    // Flex 0~4
    for(int i = 0; i < FLEX_COUNT; i++)
        feature[i] = FlexSensor_GetValue(i);

    // Accel 5~7
    float ax, ay, az;
    mpu6050_read_accel(&ax, &ay, &az);
    feature[5] = ax;
    feature[6] = ay;
    feature[7] = az;

    // Gyro 8~10
    float gx, gy, gz;
    mpu6050_read_gyro(&gx, &gy, &gz);
    feature[8] = gx;
    feature[9] = gy;
    feature[10] = gz;
}
