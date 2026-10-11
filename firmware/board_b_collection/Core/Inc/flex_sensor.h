/*
 * flex_seonsor.h
 *
 *  Created on: Nov 20, 2025
 *      Author: nayeong
 */

#ifndef __FLEX_SENSOR_H__
#define __FLEX_SENSOR_H__

#include "stm32f1xx_hal.h"

#define FLEX_COUNT 5   // 5개의 플렉스 센서

extern ADC_HandleTypeDef hadc1;
extern uint16_t flex_raw[FLEX_COUNT];   // DMA 버퍼

void FlexSensor_Init(void);
void FlexSensor_Start(void);            // DMA 시작
float FlexSensor_GetValue(int index);   // float 변환된 값

#endif
