


#include "flex_sensor.h"

uint16_t flex_raw[FLEX_COUNT] = {0};

void FlexSensor_Init(void)
{
    // ADC1은 CubeMX에서 Scan Mode + Continuous + DMA로 설정해둔다.
    // 여기서는 DMA만 시작하면 자동으로 flex_raw[]에 채워짐
}

void FlexSensor_Start(void)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)flex_raw, FLEX_COUNT);
}

float FlexSensor_GetValue(int index)
{
    if(index < 0 || index >= FLEX_COUNT) return 0.0f;

    // 간단 ADC → 전압 변환
    return (float)flex_raw[index] * (3.3f / 4095.0f);
}
