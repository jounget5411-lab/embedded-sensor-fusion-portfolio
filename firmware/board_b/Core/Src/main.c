/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : B-Board (Flex 5EA + MPU6050 + ML Inference for Gesture)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>

// === ML weight header ===
// weights_gesture.h: 20-class quantized model
//  - CLASS_NAMES[0..19]
//  - FEAT_MEAN[185], FEAT_INV_STD[185]
//  - W_LAYER[20][185], B_LAYER[20]
#include "weights_gesture.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// MPU6050 데이터 구조체
typedef struct {
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    float   Ax;
    float   Ay;
    float   Az;
    float   Gx;
    float   Gy;
    float   Gz;
} MPU6050_t;

// 제스처 수집 상태
typedef enum {
    GESTURE_IDLE = 0,
    GESTURE_COLLECTING
} GestureState_t;

// 문장 모드에서 S/V/O 단계
typedef enum {
    SENT_STAGE_S = 0,
    SENT_STAGE_V,
    SENT_STAGE_O
} SentenceStage_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define FEATURE_DIM   INPUT_DIM      // = 185
#define N_CLASSES     NUM_CLASSES    // = 20

// ===== 샘플링/윈도우 설정 =====
#define SAMPLE_RATE_HZ       100U
#define SAMPLE_PERIOD_MS     10U          // 100Hz → 10 ms
#define WINDOW_LEN           200U         // 2초 × 100Hz = 200 프레임
#define N_SEG                5U
#define SEG_LEN              (WINDOW_LEN / N_SEG) // 40 프레임

#define N_IMU_CH             6U           // ax, ay, az, gx, gy, gz
#define N_FLEX_CH            5U           // flex 센서 5개

// === IMU(가속도 크기) 기반 제스처 시작 트리거 ===
#define ANORM_START_TH       1.20f        // 동작 시작 임계값 (필요시 튜닝)

// ===== MPU6050 레지스터/상수 =====
#define MPU6050_ADDR         (0x68 << 1)  // 7bit 주소 0x68
#define SMPLRT_DIV_REG       0x19
#define GYRO_CONFIG_REG      0x1B
#define ACCEL_CONFIG_REG     0x1C
#define ACCEL_XOUT_H_REG     0x3B
#define PWR_MGMT_1_REG       0x6B
#define WHO_AM_I_REG         0x75

#define ACCEL_SCALE_FACTOR   16384.0f     // ±2g
#define GYRO_SCALE_FACTOR    131.0f       // ±250 deg/s

/* Flex Sensor Constants */
const float VCC  = 3.3f;
const float R_DIV = 47500.0f;              // 47.5k 저항
const float STRAIGHT_RESISTANCE = 21650.0f; // 펼침
const float BEND_RESISTANCE     = 752000.0f; // 90도 굽힘

#define FEAT_QSCALE  512.0f

// ----- NN class index mapping (weights_gesture.h 의 CLASS_NAMES 순서) -----
enum {
    NN_COME = 0,
    NN_DRINK,
    NN_EAT,
    NN_GO,
    NN_HE,
    NN_HELLO,
    NN_HELP,
    NN_HOME,
    NN_HOSPITAL,
    NN_I,
    NN_IM_OKAY,
    NN_MONEY,
    NN_NO_THANK_YOU,
    NN_THANK_YOU,
    NN_THEY,
    NN_TOILET,
    NN_WANT,
    NN_WATER,
    NN_WE,
    NN_YOU,
};

// ----- B보드 외부로 보내는 1~6 코드에 대응되는 후보 집합 -----
#define B_SIMPLE_NUM 5
#define B_STAGE_NUM  5

// 간단어 모드: 2=HELLO, 3=IM_OKAY, 4=THANK_YOU, 5=NO_THANK_YOU, 6=HELP
static const uint8_t SIMPLE_SET[B_SIMPLE_NUM] = {
    NN_HELLO, NN_IM_OKAY, NN_THANK_YOU, NN_NO_THANK_YOU, NN_HELP
};

// 문장 모드 S단계: 2=I, 3=YOU, 4=WE, 5=HE, 6=THEY
static const uint8_t SENT_S_SET[B_STAGE_NUM] = {
    NN_I, NN_YOU, NN_WE, NN_HE, NN_THEY
};

// 문장 모드 V단계: 2=WANT, 3=GO, 4=COME, 5=EAT, 6=DRINK
static const uint8_t SENT_V_SET[B_STAGE_NUM] = {
    NN_WANT, NN_GO, NN_COME, NN_EAT, NN_DRINK
};

// 문장 모드 O단계: 2=WATER, 3=MONEY, 4=HOME, 5=HOSPITAL, 6=TOILET
static const uint8_t SENT_O_SET[B_STAGE_NUM] = {
    NN_WATER, NN_MONEY, NN_HOME, NN_HOSPITAL, NN_TOILET
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */

// ===== 센서 버퍼 =====
uint16_t adcBuf[N_FLEX_CH];            // Flex 5채널 ADC DMA 버퍼

MPU6050_t MPU6050;
uint8_t   mpu_ready = 0;

// 2초(200프레임) 윈도우 버퍼
float imu_buf[WINDOW_LEN][N_IMU_CH];   // [t][0..5] = ax,ay,az,gx,gy,gz
float flex_buf[WINDOW_LEN][N_FLEX_CH]; // [t][0..4] = 각도(도 단위)
float a_norm_buf[WINDOW_LEN];          // |a| = sqrt(ax^2+ay^2+az^2)
float g_norm_buf[WINDOW_LEN];          // |g| = sqrt(gx^2+gy^2+gz^2)

// 제스처 수집 상태
GestureState_t  g_gesture_state = GESTURE_IDLE;
uint16_t        g_sample_count  = 0;   // 현재 윈도우 내 저장된 샘플 수
uint8_t         g_window_ready  = 0;   // 200프레임 모였는지 플래그

// C보드 ↔ B보드 모드 정보 (SPI 수신)
// 1 = 간단어 모드, 2 = 문장 모드
volatile uint8_t g_mode_flag = 1;

// B보드가 결정한 최종 클래스 (1~6, 1=IDLE)
volatile uint8_t g_current_class = 0;

// 문장 모드에서 B보드 내부 S/V/O 단계
SentenceStage_t g_sent_stage = SENT_STAGE_S;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

// Flex 함수
static float   Flex_CalcAngle(float flexR);
static float   Flex_GetAngle_FromADC(uint16_t raw);

// MPU6050 함수
static void              MPU6050_Init(void);
static HAL_StatusTypeDef MPU6050_Read6Axis(void);

// B보드 내부 로직
static void     B_SampleStep(void);                  // 10ms마다 호출
static void     B_FeatureExtract(float feat[FEATURE_DIM]);
static void     B_RunInference(void);
static uint8_t  B_Argmax(const float *x, int n);
static uint8_t  B_SelectFromSet(const float logits[N_CLASSES],
                                const uint8_t *idx_set,
                                int set_size,
                                uint8_t *out_nn_idx);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// printf → UART2
int _write(int file, char *data, int len)
{
    (void)file;
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
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();

  /* USER CODE BEGIN 2 */

  printf("\r\n==== B-Board Boot (Flex+MPU6050+ML) ====\r\n");

  // MPU6050 초기화
  MPU6050_Init();
  if (mpu_ready) {
      printf("MPU6050 Ready\r\n");
  } else {
      printf("MPU6050 NOT Ready (check wiring)\r\n");
  }

  // ADC DMA 시작 (Flex 5채널)
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuf, N_FLEX_CH) != HAL_OK)
  {
      printf("ADC DMA Start Error\r\n");
      Error_Handler();
  }

  // 초기 상태
  g_gesture_state = GESTURE_IDLE;
  g_sample_count  = 0;
  g_window_ready  = 0;
  g_mode_flag     = 1;  // 기본: 간단어
  g_current_class = 1;  // IDLE
  g_sent_stage    = SENT_STAGE_S;

  uint32_t last_sample_ms = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      uint32_t now = HAL_GetTick();

      // ===== 100Hz(10ms) 샘플링 =====
      if ((uint32_t)(now - last_sample_ms) >= SAMPLE_PERIOD_MS)
      {
          last_sample_ms += SAMPLE_PERIOD_MS;
          B_SampleStep();
      }

      // ===== 윈도우(200프레임) 준비되면 특징 추출 + ML 판별 =====
      if (g_window_ready)
      {
          g_window_ready = 0;
          B_RunInference();
      }

      // ===== SPI2 Slave: C보드와 1바이트 교환 =====
      //  - Master(C보드) TX: g_c2b_flag (1=간단어,2=문장)
      //  - Slave(B보드) TX: g_current_class (1~6)
      uint8_t tx = g_current_class;
      uint8_t rx = 0;
      uint8_t prev_mode = g_mode_flag;

      if (HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, 1) == HAL_OK)
      {
          if (rx == 1 || rx == 2)
          {
              g_mode_flag = rx;
          }
      }

      // 모드가 1↔2 바뀌면, 문장 모드 진입 시 S단계로 리셋
      if (prev_mode != g_mode_flag)
      {
          if (g_mode_flag == 2)
          {
              g_sent_stage = SENT_STAGE_S;
              printf("[MODE] -> SENTENCE, reset stage=S\r\n");
          }
          else
          {
              printf("[MODE] -> SIMPLE\r\n");
          }
      }

      // 필요시 약간의 idle
      // HAL_Delay(1);
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK
                              | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1
                              | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function  (Flex 5채널: CH0,1,4,5,6)
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance                    = ADC1;
  hadc1.Init.ScanConvMode           = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode     = ENABLE;
  hadc1.Init.DiscontinuousConvMode  = DISABLE;
  hadc1.Init.ExternalTrigConv       = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign              = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion        = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  // CH0
  sConfig.Channel      = ADC_CHANNEL_0;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // CH1
  sConfig.Channel      = ADC_CHANNEL_1;
  sConfig.Rank         = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // CH4
  sConfig.Channel      = ADC_CHANNEL_4;
  sConfig.Rank         = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // CH5
  sConfig.Channel      = ADC_CHANNEL_5;
  sConfig.Rank         = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // CH6
  sConfig.Channel      = ADC_CHANNEL_6;
  sConfig.Rank         = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function (MPU6050)
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance             = I2C1;
  hi2c1.Init.ClockSpeed      = 100000;
  hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1     = 0;
  hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2     = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function (디버그 printf)
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance          = USART2;
  huart2.Init.BaudRate     = 115200;
  huart2.Init.WordLength   = UART_WORDLENGTH_8B;
  huart2.Init.StopBits     = UART_STOPBITS_1;
  huart2.Init.Parity       = UART_PARITY_NONE;
  huart2.Init.Mode         = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function (Slave, C보드와 통신)
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{
  hspi2.Instance               = SPI2;
  hspi2.Init.Mode              = SPI_MODE_SLAVE;
  hspi2.Init.Direction         = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize          = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity       = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase          = SPI_PHASE_1EDGE;
  hspi2.Init.NSS               = SPI_NSS_SOFT;   // CS는 C보드에서 GPIO로 제어
  hspi2.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode            = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial     = 7;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
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

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // 기본 B1 버튼 (필요시)
  GPIO_InitStruct.Pin  = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  // SPI2 핀 (PB13=SCK, PB14=MISO, PB15=MOSI)
  GPIO_InitStruct.Pin   = GPIO_PIN_13 | GPIO_PIN_15; // SCK, MOSI
  GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin   = GPIO_PIN_14; // MISO
  GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // EXTI 설정 (필요시)
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */

// ===== Flex Sensor Logic =====
static float Flex_CalcAngle(float flexR)
{
    float angle = (flexR - STRAIGHT_RESISTANCE) * 90.0f /
                  (BEND_RESISTANCE - STRAIGHT_RESISTANCE);
    if (angle < 0.0f)  angle = 0.0f;
    if (angle > 90.0f) angle = 90.0f;
    return angle;
}

static float Flex_GetAngle_FromADC(uint16_t raw)
{
    // ADC 12bit → 전압 → 저항 → 각도
    float flexV = (float)raw * VCC / 4095.0f;
    if (flexV <= 0.001f) flexV = 0.001f;  // 0 division 보호
    float flexR = R_DIV * (VCC / flexV - 1.0f);
    return Flex_CalcAngle(flexR);
}

// ===== MPU6050 Logic =====
static void MPU6050_Init(void)
{
    uint8_t check;
    uint8_t Data;

    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG,
                         1, &check, 1, 1000) != HAL_OK)
    {
        printf("MPU6050 I2C read error\r\n");
        mpu_ready = 0;
        return;
    }

    printf("MPU6050 WHO_AM_I = 0x%02X\r\n", check);
    if (check != 0x68)
    {
        printf("WARNING: unexpected ID (expect 0x68)\r\n");
    }

    mpu_ready = 1;

    // PWR_MGMT_1: sleep 해제
    Data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG,
                      1, &Data, 1, 1000);

    // Sample rate: 1kHz / (1+7) = 125 Hz
    Data = 0x07;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG,
                      1, &Data, 1, 1000);

    // Accel ±2g
    Data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG,
                      1, &Data, 1, 1000);

    // Gyro ±250 deg/s
    Data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG,
                      1, &Data, 1, 1000);
}

static HAL_StatusTypeDef MPU6050_Read6Axis(void)
{
    uint8_t Rec_Data[14];
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR,
                              ACCEL_XOUT_H_REG, 1,
                              Rec_Data, 14, 1000);
    if (status != HAL_OK) return status;

    MPU6050.Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    MPU6050.Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    MPU6050.Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    MPU6050.Gyro_X_RAW  = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    MPU6050.Gyro_Y_RAW  = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    MPU6050.Gyro_Z_RAW  = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    MPU6050.Ax = MPU6050.Accel_X_RAW / ACCEL_SCALE_FACTOR;
    MPU6050.Ay = MPU6050.Accel_Y_RAW / ACCEL_SCALE_FACTOR;
    MPU6050.Az = MPU6050.Accel_Z_RAW / ACCEL_SCALE_FACTOR;

    MPU6050.Gx = MPU6050.Gyro_X_RAW  / GYRO_SCALE_FACTOR;
    MPU6050.Gy = MPU6050.Gyro_Y_RAW  / GYRO_SCALE_FACTOR;
    MPU6050.Gz = MPU6050.Gyro_Z_RAW  / GYRO_SCALE_FACTOR;

    return HAL_OK;
}

// ===== 10ms마다 호출되는 샘플링 + 윈도우 수집 =====
static void B_SampleStep(void)
{
    // 1) IMU 읽기
    if (mpu_ready)
    {
        if (MPU6050_Read6Axis() != HAL_OK)
        {
            // 실패 시 이전 값 유지
        }
    }
    // 가속도/각속도 크기
    float ax = MPU6050.Ax;
    float ay = MPU6050.Ay;
    float az = MPU6050.Az;
    float gx = MPU6050.Gx;
    float gy = MPU6050.Gy;
    float gz = MPU6050.Gz;

    float a_norm = sqrtf(ax*ax + ay*ay + az*az);
    float g_norm = sqrtf(gx*gx + gy*gy + gz*gz);

    // 2) 제스처 시작 트리거 (가속도 크기 기준)
    if (g_gesture_state == GESTURE_IDLE)
    {
        if (a_norm > ANORM_START_TH)
        {
            g_gesture_state = GESTURE_COLLECTING;
            g_sample_count  = 0;
            // printf("Gesture start (a_norm=%.2f)\r\n", a_norm);
        }
        return;
    }

    // 3) COLLECTING 상태에서 200프레임 저장
    if (g_gesture_state == GESTURE_COLLECTING)
    {
        if (g_sample_count < WINDOW_LEN)
        {
            uint16_t n = g_sample_count;

            // IMU 저장
            imu_buf[n][0] = ax;
            imu_buf[n][1] = ay;
            imu_buf[n][2] = az;
            imu_buf[n][3] = gx;
            imu_buf[n][4] = gy;
            imu_buf[n][5] = gz;

            a_norm_buf[n] = a_norm;
            g_norm_buf[n] = g_norm;

            // Flex 각도 저장 (ADC DMA 버퍼에서 최신 값 사용)
            for (int i = 0; i < (int)N_FLEX_CH; i++)
            {
                flex_buf[n][i] = Flex_GetAngle_FromADC(adcBuf[i]);
            }

            g_sample_count++;

            if (g_sample_count >= WINDOW_LEN)
            {
                g_gesture_state = GESTURE_IDLE;
                g_window_ready  = 1;
                // printf("Gesture window ready (200 samples)\r\n");
            }
        }
        else
        {
            g_gesture_state = GESTURE_IDLE;
        }
    }
}

// ===== 특징 추출 (200프레임 → 185차원) =====
static void B_FeatureExtract(float feat[FEATURE_DIM])
{
    int k = 0;

    // ---------- (1) IMU 6축: 각 구간별 mean, std, energy → 6×5×3 = 90 ----------
    for (int seg = 0; seg < (int)N_SEG; seg++)
    {
        int start = seg * SEG_LEN;
        int end   = start + SEG_LEN;

        float mean[N_IMU_CH]   = {0};
        float var[N_IMU_CH]    = {0};
        float energy[N_IMU_CH] = {0};

        // mean
        for (int t = start; t < end; t++)
        {
            for (int ch = 0; ch < (int)N_IMU_CH; ch++)
            {
                mean[ch] += imu_buf[t][ch];
            }
        }
        for (int ch = 0; ch < (int)N_IMU_CH; ch++)
        {
            mean[ch] /= (float)SEG_LEN;
        }

        // var, energy
        for (int t = start; t < end; t++)
        {
            for (int ch = 0; ch < (int)N_IMU_CH; ch++)
            {
                float v = imu_buf[t][ch];
                float d = v - mean[ch];
                var[ch]    += d * d;
                energy[ch] += v * v;
            }
        }
        for (int ch = 0; ch < (int)N_IMU_CH; ch++)
        {
            var[ch]    = sqrtf(var[ch] / (float)SEG_LEN);   // std
            energy[ch] = energy[ch] / (float)SEG_LEN;       // 평균 제곱값
        }

        // feature stacking
        for (int ch = 0; ch < (int)N_IMU_CH; ch++)
        {
            feat[k++] = mean[ch];
            feat[k++] = var[ch];
            feat[k++] = energy[ch];
        }
    }
    // 여기까지 k == 90

    // ---------- (2) |a|, |g|: 각 구간별 mean, max → 2×5×2 = 20 ----------
    for (int seg = 0; seg < (int)N_SEG; seg++)
    {
        int start = seg * SEG_LEN;
        int end   = start + SEG_LEN;

        float a_sum = 0.0f, g_sum = 0.0f;
        float a_max = 0.0f, g_max = 0.0f;

        for (int t = start; t < end; t++)
        {
            float a = a_norm_buf[t];
            float g = g_norm_buf[t];

            a_sum += a;
            g_sum += g;

            if (t == start || a > a_max) a_max = a;
            if (t == start || g > g_max) g_max = g;
        }

        float a_mean = a_sum / (float)SEG_LEN;
        float g_mean = g_sum / (float)SEG_LEN;

        feat[k++] = a_mean;
        feat[k++] = a_max;
        feat[k++] = g_mean;
        feat[k++] = g_max;
    }
    // 여기까지 k == 110

    // ---------- (3) Flex 5개: 각 구간/손가락별 mean, std, range → 5×5×3 = 75 ----------
    for (int seg = 0; seg < (int)N_SEG; seg++)
    {
        int start = seg * SEG_LEN;
        int end   = start + SEG_LEN;

        float mean[N_FLEX_CH] = {0};
        float var[N_FLEX_CH]  = {0};
        float f_min[N_FLEX_CH];
        float f_max[N_FLEX_CH];

        // 초기 min/max
        for (int f = 0; f < (int)N_FLEX_CH; f++)
        {
            f_min[f] = flex_buf[start][f];
            f_max[f] = flex_buf[start][f];
        }

        // mean, min, max
        for (int t = start; t < end; t++)
        {
            for (int f = 0; f < (int)N_FLEX_CH; f++)
            {
                float v = flex_buf[t][f];
                mean[f] += v;

                if (v < f_min[f]) f_min[f] = v;
                if (v > f_max[f]) f_max[f] = v;
            }
        }
        for (int f = 0; f < (int)N_FLEX_CH; f++)
        {
            mean[f] /= (float)SEG_LEN;
        }

        // var
        for (int t = start; t < end; t++)
        {
            for (int f = 0; f < (int)N_FLEX_CH; f++)
            {
                float v = flex_buf[t][f];
                float d = v - mean[f];
                var[f] += d * d;
            }
        }

        for (int f = 0; f < (int)N_FLEX_CH; f++)
        {
            float std   = sqrtf(var[f] / (float)SEG_LEN);
            float range = f_max[f] - f_min[f];

            feat[k++] = mean[f];
            feat[k++] = std;
            feat[k++] = range;
        }
    }

    // 최종적으로 k == FEATURE_DIM(185) 이어야 함
    // (디버그용) 필요하면 아래 주석 풀어서 확인
    // printf("Feature dim k=%d\r\n", k);
}

// ===== ML Inference (185차원 + 가중치 6×185) =====
// ===== argmax 그대로 사용 =====
static uint8_t B_Argmax(const float *x, int n)
{
    int   best_idx = 0;
    float best_val = x[0];

    for (int i = 1; i < n; i++)
    {
        if (x[i] > best_val)
        {
            best_val = x[i];
            best_idx = i;
        }
    }
    return (uint8_t)best_idx; // 0~(n-1)
}

// 주어진 logits에서 idx_set에 들어있는 클래스들만 비교해서
// B보드 코드(2~6)를 리턴, out_nn_idx에는 NN 내부 인덱스를 기록
static uint8_t B_SelectFromSet(const float logits[N_CLASSES],
                               const uint8_t *idx_set,
                               int set_size,
                               uint8_t *out_nn_idx)
{
    int     best_k   = 0;
    uint8_t best_nn  = idx_set[0];
    float   best_val = logits[best_nn];

    for (int k = 1; k < set_size; k++)
    {
        uint8_t cls = idx_set[k];
        float   v   = logits[cls];
        if (v > best_val)
        {
            best_val = v;
            best_nn  = cls;
            best_k   = k;
        }
    }

    if (out_nn_idx)
    {
        *out_nn_idx = best_nn;
    }

    // B보드 외부 코드: 2,3,4,5,6
    return (uint8_t)(best_k + 2);
}

// ===== ML Inference: weights_gesture.h 사용 =====
static void B_RunInference(void)
{
    float feat[FEATURE_DIM];
    B_FeatureExtract(feat);   // 185차원 feature 계산

    // 1) 표준화
    float feat_norm[FEATURE_DIM];

    for (int i = 0; i < (int)FEATURE_DIM; i++)
    {
        float mu      = (float)FEAT_MEAN[i]     / FEAT_QSCALE;  // ~ 실제 mean
        float inv_std = (float)FEAT_INV_STD[i]  / FEAT_QSCALE;  // ~ 1/std

        feat_norm[i] = (feat[i] - mu) * inv_std;
    }

    // 2) 20클래스에 대한 선형결합 (logits)
    float logits[N_CLASSES];

    for (int c = 0; c < (int)N_CLASSES; c++)
    {
        float sum = (float)B_LAYER[c];  // bias

        for (int i = 0; i < (int)FEATURE_DIM; i++)
        {
            sum += (float)W_LAYER[c][i] * feat_norm[i];
        }

        logits[c] = sum;
    }

    // 3) 모드/단계별 후보 집합 안에서만 argmax
    uint8_t nn_idx = 0;
    uint8_t code   = 1;  // 기본은 IDLE

    if (g_mode_flag == 1)
    {
        // ===== 간단어 모드 =====
        // 2=HELLO, 3=IM_OKAY, 4=THANK_YOU, 5=NO_THANK_YOU, 6=HELP
        code = B_SelectFromSet(logits, SIMPLE_SET, B_SIMPLE_NUM, &nn_idx);
        g_current_class = code;

        printf("[SIMPLE] code=%d → %s\r\n",
               g_current_class,
               CLASS_NAMES[nn_idx]);
    }
    else
    {
        // ===== 문장 모드: S → V → O 순서 =====
        switch (g_sent_stage)
        {
            case SENT_STAGE_S:
                code = B_SelectFromSet(logits, SENT_S_SET, B_STAGE_NUM, &nn_idx);
                g_current_class = code;
                printf("[SENT S] code=%d → %s\r\n",
                       g_current_class,
                       CLASS_NAMES[nn_idx]);
                g_sent_stage = SENT_STAGE_V;
                break;

            case SENT_STAGE_V:
                code = B_SelectFromSet(logits, SENT_V_SET, B_STAGE_NUM, &nn_idx);
                g_current_class = code;
                printf("[SENT V] code=%d → %s\r\n",
                       g_current_class,
                       CLASS_NAMES[nn_idx]);
                g_sent_stage = SENT_STAGE_O;
                break;

            case SENT_STAGE_O:
            default:
                code = B_SelectFromSet(logits, SENT_O_SET, B_STAGE_NUM, &nn_idx);
                g_current_class = code;
                printf("[SENT O] code=%d → %s\r\n",
                       g_current_class,
                       CLASS_NAMES[nn_idx]);
                g_sent_stage = SENT_STAGE_S;
                break;
        }
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */

