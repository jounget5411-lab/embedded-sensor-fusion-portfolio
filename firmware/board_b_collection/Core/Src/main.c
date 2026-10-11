/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Flex Sensor 5EA + MPU6050 (6DOF Filter for stable Roll/Pitch)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h> // MPU6050 6축 기울기 계산을 위해 추가
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// MPU6050 데이터 구조체
typedef struct {
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    float   Ax;
    float   Ay;
    float   Az;
    // --- [추가] 자이로스코프 데이터
    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    float   Gx;        // 각속도 (deg/s)
    float   Gy;
    float   Gz;
    // --- [수정] 필터링된 각도 (Roll, Pitch)
    float   roll;
    float   pitch;
    float   yaw;       // 자이로 적분 (드리프트 발생 가능)
} MPU6050_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SAMPLE_HZ   100
#define WIN_SEC     2
#define WIN_LEN     (SAMPLE_HZ * WIN_SEC)   // 200

// [프레임][채널]
float flex_win[WIN_LEN][5]; // ADC값 or 각도값
float imu_win[WIN_LEN][6];  // ax,ay,az,gx,gy,gz

uint16_t win_idx = 0;
uint8_t  capturing = 0;      // 0: 대기, 1: 2초 수집 중

float a_mag = 0.0f;
uint8_t trigger_on = 0;



// MPU6050 레지스터 주소/레지스터
#define MPU6050_ADDR        0xD0      // (0x68 << 1)
#define SMPLRT_DIV_REG      0x19
#define GYRO_CONFIG_REG     0x1B
#define ACCEL_CONFIG_REG    0x1C
#define ACCEL_XOUT_H_REG    0x3B
#define GYRO_XOUT_H_REG     0x43      // 자이로 데이터 레지스터 시작
#define PWR_MGMT_1_REG      0x6B
#define WHO_AM_I_REG        0x75

// ★ 6축 필터링을 위한 상수 (MPU6050 설정: ±2g, ±250 deg/s)
#define ACCEL_SCALE_FACTOR  16384.0f  // 2g 범위 (16bit)
#define GYRO_SCALE_FACTOR   131.0f    // 250 deg/s 범위 (16bit)
#define DT_SEC              0.15f     // 메인 루프 딜레이 시간 (150ms)
#define COMPLEMENTARY_ALPHA 0.98f     // 상보 필터 가중치 (자이로스코프 신뢰도)

/* Flex Sensor Constants */
const float VCC  = 3.3f;
const float R_DIV = 47500.0f;              // 47.5k 저항
const float STRAIGHT_RESISTANCE = 21650.0f; // 펼침
const float BEND_RESISTANCE     = 752000.0f; // 90도 굽힘

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// --- 제스처 윈도우 설정 ---
#define SAMPLE_HZ   100          // 100 Hz
#define WIN_SEC     2            // 2초
#define WIN_LEN     (SAMPLE_HZ * WIN_SEC)   // 200 프레임

// flex 5개, IMU(일단 accel 3축만) 윈도우 버퍼
float flex_win[WIN_LEN][5];      // [프레임][손가락]
float imu_win[WIN_LEN][3];       // [프레임][Ax, Ay, Az]

// 윈도우 상태
uint16_t win_idx   = 0;          // 현재 채운 프레임 수
uint8_t  capturing = 0;          // 0: 대기, 1: 2초 수집 중

// 트리거용
float a_mag = 0.0f;
const float A_TRIG_ON  = 1.20f;  // 시작 임계값 (1.2 g 정도 예시)
const float A_TRIG_OFF = 1.05f;  // 끝난 후 다시 안정되는 기준(필요하면 사용)

// === Flex Sensor 설정 ===
uint16_t adcBuf[5];   // DMA 버퍼 (센서 5개)

// === MPU6050 변수 ===
MPU6050_t MPU6050;
uint8_t   mpu_ready = 0;   // 센서 준비 여부 플래그

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
// Flex Functions
float   Flex_CalcAngle(float flexR);
float   Flex_GetAngle_FromADC(uint16_t raw);
uint8_t is90(float angle);

// MPU6050 Functions
void              MPU6050_Init(void);
HAL_StatusTypeDef MPU6050_Read_6DOF_Data(void); // 6축 데이터 읽기 및 필터링
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// printf 출력을 위한 _write 함수 재정의
int _write(int file, char *data, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)data, (uint16_t)len, HAL_MAX_DELAY);
    return len;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  printf("\r\n--- System Starting (Flex x5 + MPU6050 6DOF Filter) ---\r\n");

  // MPU6050 초기화
  MPU6050_Init();
  if (mpu_ready)
      printf("MPU6050 Ready.\r\n");
  else
      printf("MPU6050 NOT Ready. (Check wiring/address)\r\n");

  // ADC DMA 시작 : 버퍼 크기 5개
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuf, 5) != HAL_OK)
  {
      printf("ADC DMA Start Error!\r\n");
      Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t printCounter = 0;
  uint8_t shakeCount   = 0;

  // 초기 가속도/각도 값 설정
  MPU6050.Ax = 0.0f;
  MPU6050.Ay = 0.0f;
  MPU6050.Az = 1.0f;
  MPU6050.roll = 0.0f;
  MPU6050.pitch = 0.0f;
  MPU6050.yaw = 0.0f;

  while (1)
  {
	  while (1)
	  {
	      // 1) 항상 최신 flex / IMU 읽기
	      //  - flex: adcBuf[] → 각도 or raw 그대로
	      //  - IMU: MPU6050_Read_Accel() + (추가로 gyro도 읽으면 gx,gy,gz까지)

	      // flex 각도 계산
	      float angle[5];
	      for (int i = 0; i < 5; i++)
	          angle[i] = Flex_GetAngle_FromADC(adcBuf[i]);

	      // IMU 읽기
	      if (mpu_ready)
	      {
	          MPU6050_Read_Accel();   // 여기에 자이로까지 읽는 함수로 확장해야 함
	      }

	      // 2) a의 magnitude 계산해서 트리거
	      a_mag = sqrtf(MPU6050.Ax*MPU6050.Ax +
	                    MPU6050.Ay*MPU6050.Ay +
	                    MPU6050.Az*MPU6050.Az);

	      if (!capturing)
	      {
	          // 아직 제스처 대기 상태
	          if (a_mag > A_THRESH)   // 예: 1.2g 같은 임계값
	          {
	              capturing = 1;
	              win_idx   = 0;
	          }
	      }

	      if (capturing)
	      {
	          // 3) 현재 샘플을 윈도우 버퍼에 저장
	          if (win_idx < WIN_LEN)
	          {
	              for (int i = 0; i < 5; i++)
	                  flex_win[win_idx][i] = angle[i];   // 또는 adcBuf[i] 그대로

	              imu_win[win_idx][0] = MPU6050.Ax;
	              imu_win[win_idx][1] = MPU6050.Ay;
	              imu_win[win_idx][2] = MPU6050.Az;
	              // gyro 추가되면 3개 더

	              win_idx++;
	          }

	          if (win_idx >= WIN_LEN)
	          {
	              // 4) 2초(200프레임) 다 채워짐
	              capturing = 0;

	              // 여기서:
	              //  - CSV로 한 번 쫙 찍어서 PC에 저장 (학습용)
	              //  - 나중에는 여기서 185차원 feature 계산 + 분류 호출
	          }
	      }

	      HAL_Delay(10); // 100Hz
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// 200프레임짜리 윈도우를 CSV 포맷으로 UART2로 쏘기
void DumpWindowAsCSV(void)
{
    // 구분자 (Python에서 BEGIN_WINDOW ~ END_WINDOW 사이를 한 샘플로 저장)
    printf("BEGIN_WINDOW\r\n");
    // 헤더 (원하면 안 써도 되지만 있으면 편함)
    printf("frame,flex0,flex1,flex2,flex3,flex4,Ax,Ay,Az\r\n");

    for (uint16_t i = 0; i < WIN_LEN; i++)
    {
        printf("%u", i); // frame index
        for (int j = 0; j < 5; j++)
        {
            printf(",%.3f", flex_win[i][j]);   // 각도(도) 값
        }
        printf(",%.4f,%.4f,%.4f\r\n",
               imu_win[i][0], imu_win[i][1], imu_win[i][2]); // Ax, Ay, Az
    }

    printf("END_WINDOW\r\n");
}

// --- Flex Sensor Logic ---
float Flex_CalcAngle(float flexR)
{
    float angle = (flexR - STRAIGHT_RESISTANCE) * 90.0f /
                  (BEND_RESISTANCE - STRAIGHT_RESISTANCE);
    if (angle < 0.0f)  angle = 0.0f;
    if (angle > 90.0f) angle = 90.0f;
    return angle;
}

uint8_t is90(float angle)
{
    // 70도 이상 굽힘으로 판단
    return (angle > 70.0f);
}

float Flex_GetAngle_FromADC(uint16_t raw)
{
    float flexV = (float)raw * VCC / 4095.0f;
    float flexR = R_DIV * (VCC / flexV - 1.0f);
    return Flex_CalcAngle(flexR);
}

// --- MPU6050 Logic ---
void MPU6050_Init(void)
{
    uint8_t check;
    uint8_t Data;

    // WHO_AM_I 읽어서 I2C 통신 확인
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG,
                         1, &check, 1, 1000) != HAL_OK)
    {
        printf("MPU6050 I2C Error or not connected!\r\n");
        mpu_ready = 0;
        return;
    }

    printf("MPU6050 WHO_AM_I = 0x%02X\r\n", check);

    if (check != 0x68)
    {
        printf("WARNING: Unexpected ID (expect 0x68)\r\n");
    }

    mpu_ready = 1;

    // 1. Sleep 해제 (PWR_MGMT_1)
    Data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG,
                      1, &Data, 1, 1000);

    // 2. Sample Rate (1kHz / (1 + 7) = 125 Hz)
    Data = 0x07;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG,
                      1, &Data, 1, 1000);

    // 3. Accelerometer Range ±2g
    Data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG,
                      1, &Data, 1, 1000);

    // ★ 4. Gyroscope Range ±250 deg/s
    Data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG,
                      1, &Data, 1, 1000);
}

HAL_StatusTypeDef MPU6050_Read_6DOF_Data(void)
{
    // 가속도 6 bytes + 온도 2 bytes + 자이로 6 bytes = 총 14 bytes
    uint8_t Rec_Data[14];
    HAL_StatusTypeDef status;
    const float PI = 3.14159265359f;

    // 1. I2C 데이터 읽기 (ACCEL_XOUT_H_REG 부터 14 bytes)
    status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG,
                              1, Rec_Data, 14, 1000);

    if (status != HAL_OK) return status;

    // 2. RAW 데이터 추출 및 스케일링
    // Accel (Bytes 0-5)
    MPU6050.Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    MPU6050.Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    MPU6050.Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    // Gyro (Bytes 8-13, 온도 2 bytes 건너뜀)
    MPU6050.Gyro_X_RAW  = (int16_t)(Rec_Data[8]  << 8 | Rec_Data[9]);
    MPU6050.Gyro_Y_RAW  = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    MPU6050.Gyro_Z_RAW  = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    // 스케일링 (단위: g, deg/s)
    MPU6050.Ax = MPU6050.Accel_X_RAW / ACCEL_SCALE_FACTOR;
    MPU6050.Ay = MPU6050.Accel_Y_RAW / ACCEL_SCALE_FACTOR;
    MPU6050.Az = MPU6050.Accel_Z_RAW / ACCEL_SCALE_FACTOR;
    MPU6050.Gx = MPU6050.Gyro_X_RAW / GYRO_SCALE_FACTOR;
    MPU6050.Gy = MPU6050.Gyro_Y_RAW / GYRO_SCALE_FACTOR;
    MPU6050.Gz = MPU6050.Gyro_Z_RAW / GYRO_SCALE_FACTOR;

    // 3. 가속도계만으로 Roll/Pitch 계산 (절대 기울기)
    float Accel_Roll  = atan2(MPU6050.Ay, MPU6050.Az) * 180.0f / PI;
    float AyAz_Magnitude = sqrt(MPU6050.Ay * MPU6050.Ay + MPU6050.Az * MPU6050.Az);
    float Accel_Pitch = atan2(-MPU6050.Ax, AyAz_Magnitude) * 180.0f / PI;

    // 4. 자이로스코프 적분 (회전 추적)
    float Gyro_Roll  = MPU6050.roll  + MPU6050.Gx * DT_SEC;
    float Gyro_Pitch = MPU6050.pitch + MPU6050.Gy * DT_SEC;

    // 5. 상보 필터(Complementary Filter) 적용
    MPU6050.roll  = COMPLEMENTARY_ALPHA * Gyro_Roll  + (1.0f - COMPLEMENTARY_ALPHA) * Accel_Roll;
    MPU6050.pitch = COMPLEMENTARY_ALPHA * Gyro_Pitch + (1.0f - COMPLEMENTARY_ALPHA) * Accel_Pitch;

    MPU6050.yaw   = MPU6050.yaw + MPU6050.Gz * DT_SEC; // Yaw는 자이로 적분만

    return HAL_OK;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
