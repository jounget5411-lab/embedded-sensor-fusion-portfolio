/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>  // printf 사용
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define QUIET_END_FRAMES     5u   // 조용한 프레임이 이만큼 연속되면 이벤트 종료

// ===== 기본 설정 =====
#define N_CH        3
#define FRAME_LEN   160                         // 20 ms @ 8 kHz
#define DMA_BUF_LEN (N_CH * FRAME_LEN * 2)      // 더블 버퍼 (Half/Full)
#define FRONT_BACK_RATIO_X100  150   // front >= 1.5x back 이면 "앞소리"로 간주

// ===== 이벤트·다수결 관련 설정 =====
// 한 번 소리가 난 뒤, 최대 몇 프레임까지 모아서 판단할지
// 70프레임 × 20ms = 1400ms
#define EVENT_MAX_FRAMES     70

// 유효한 이벤트로 인정할 최소 프레임 수
// 15프레임 × 20ms = 300ms
#define EVENT_MIN_FRAMES     15

// LB / RB / BACK 중 하나가 "거의 대부분"일 때 기준 (%)
#define STRONG_BACKSIDE_PCT  80    // 80% 이상이면 LEFT / RIGHT / BACK으로 승격

// TEST_PIN 토글은 레지스터(BSRR)로 처리
#define CPU_BUSY_ON()   (TEST_PIN_GPIO_Port->BSRR = TEST_PIN_Pin)
#define CPU_BUSY_OFF()  (TEST_PIN_GPIO_Port->BSRR = (uint32_t)TEST_PIN_Pin << 16U)

// 방향 개수 (DIR_QUIET ~ DIR_RIGHT까지 6개)
#define DIR_COUNT            6

// ===== SPI 관련 설정 =====
#define SPI_LEN              5    // C보드가 5바이트 클럭 준다고 가정

// ===== 소음 임계값 관련 설정 =====
typedef enum {
    NOISE_MODE_QUIET = 0,   // 조용함 (작은 소리도 감지)
    NOISE_MODE_NORMAL = 1,  // 보통
    NOISE_MODE_NOISY = 2    // 시끄러움 (큰 소리만 감지)
} NoiseMode;

// 모드별 임계값 (실험하면서 이 숫자들을 튜닝!)
#define TH_SILENCE_QUIET   10000000ULL
#define TH_SILENCE_NORMAL  10000000ULL
#define TH_SILENCE_NOISY   10000000ULL

#define K_EDGE_STRONG_PCT    80   // k = ±5가 이 비율(%) 이상이면 LEFT/RIGHT
#define K_CENTER_STRONG_PCT  80   // k = -1,0,+1 이 이 비율(%) 이상이면 BACK

// 방향 이름 정의 (0~5)
typedef enum {
    DIR_QUIET = 0,    // 조용
    DIR_LEFT,         // 왼쪽
    DIR_LEFT_BACK,    // 왼쪽 뒤
    DIR_BACK,         // 뒤 (가운데)
    DIR_RIGHT_BACK,   // 오른쪽 뒤
    DIR_RIGHT         // 오른쪽
} SoundDir;

// 결과 저장용 구조체
typedef struct {
    SoundDir dir;
    uint16_t level;
    uint32_t E_L;
    uint32_t E_F;
    uint32_t E_R;
} SoundResult;

// 한 번 말하기(이벤트) 동안 프레임들을 누적할 구조체
typedef struct {
    uint8_t  active;             // 현재 이벤트 진행 중인가? (0/1)
    uint8_t  n_frames;           // 이번 이벤트에 포함된 프레임 수
    uint16_t counts[DIR_COUNT];  // 방향별 프레임 카운트 (디버그용)
    uint16_t k_hist[11];         // k = -5 ~ +5 히스토그램 (idx 0 -> -5, 10 -> +5)

    uint64_t sum_front_E;        // 이벤트 동안 FRONT 에너지 누적
    uint64_t sum_back_E;         // 이벤트 동안 BACK(L+R) 에너지 누적
} DirEvent;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

volatile uint8_t g_quiet_run = 0;   // 이벤트 진행 중 연속 "조용 프레임" 개수
// DMA가 쉴 새 없이 채우는 원형 버퍼 (날것 데이터)
uint16_t adc_buf[DMA_BUF_LEN];

// 계산하기 좋게 정제된 프레임 (오프셋 뺀 값 저장)
int16_t  frame_buf[N_CH][FRAME_LEN];

// "프레임 다 찼다! 계산해라!" 깃발 (0:대기, 1:준비완료)
volatile uint8_t frame_ready = 0;

SoundResult last_result;
DirEvent    g_event = {0};

// === SPI 통신용 전역 변수 ===
uint8_t slaveTxBuf[SPI_LEN];
uint8_t slaveRxBuf[SPI_LEN];

// A보드가 C보드로 보내는 방향 코드 (0~5)
// 0: QUIET, 1: LEFT, 2: LEFT_BACK, 3: BACK, 4: RIGHT_BACK, 5: RIGHT
volatile uint8_t g_dir_code = 0;

// 현재 적용된 소음 모드와 임계값
volatile NoiseMode g_noise_mode = NOISE_MODE_NORMAL;
volatile uint64_t  g_th_silence = TH_SILENCE_NORMAL;

// SPI 연결 여부 플래그
// 0: C보드 연결 안 됨 (테스트 모드) -> SPI 오류 무시
// 1: C보드 연결 됨 (실전 모드) -> SPI 통신 수행
uint8_t g_spi_enabled = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

// ADC DMA 버퍼 → frame_buf로 복사
void copy_to_frame_buf(uint32_t start_idx);

// 프레임 처리
void process_frame(void);

// 문자열 변환
const char* dir_to_str(SoundDir d);

// 이벤트 종료 시 최종 방향 결정
void finalize_event(void);

// SPI 관련
uint8_t dir_to_code(SoundDir d);
void A_SPI_SendDir(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  // 1) ADC 캘리브레이션
  HAL_ADCEx_Calibration_Start(&hadc1);

  // 2) ADC + DMA 시작 (adc_buf는 USER CODE PV에 선언)
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, DMA_BUF_LEN);

  // 3) TIM3 베이스 타이머 시작 (TRGO로 8kHz 트리거 발생)
  HAL_TIM_Base_Start(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // 프레임 하나 완성될 때마다 처리
      if (frame_ready) {
          frame_ready = 0;
          process_frame();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  *  CH0 -> back-left (PA0)
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  *  CH1 -> front (PB0)
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  *  CH14 -> back-right (PC4)
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  *
  * TIM3: 8 kHz 업데이트, TRGO = UPDATE (ADC 트리거)
  */
static void MX_TIM3_Init(void)
{
  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  /* USER CODE BEGIN TIM3_Init 1 */

  // TIM3 클럭 enable
  __HAL_RCC_TIM3_CLK_ENABLE();

  // 핸들에 인스턴스만 채워서 HAL_TIM_Base_Start가 사용할 수 있게 함
  htim3.Instance = TIM3;

  // 타이머 클럭: 72 MHz (APB1=36MHz, 타이머는 x2)
  // 72MHz / ((PSC+1)*(ARR+1)) = 8kHz
  // (PSC+1) = 72, (ARR+1) = 125 => PSC=71, ARR=124
  TIM3->PSC = 71;
  TIM3->ARR = 124;

  // 업카운터, 자동 리로드, UEV on overflow
  TIM3->CR1 = 0;
  TIM3->CR1 |= TIM_CR1_ARPE;   // Auto-reload preload enable

  // TRGO: update 이벤트를 TRGO로 내보내기
  TIM3->CR2 &= ~TIM_CR2_MMS;
  TIM3->CR2 |= TIM_CR2_MMS_1;  // MMS = 010: Update event

  /* USER CODE END TIM3_Init 1 */

  /* USER CODE BEGIN TIM3_Init 2 */
  // 나머지 설정은 기본값 그대로 사용
  /* USER CODE END TIM3_Init 2 */
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
  // 핀 설정/클럭 enable은 HAL_UART_Init + MSP에서 처리
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TEST_PIN_GPIO_Port, TEST_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TEST_PIN_Pin */
  GPIO_InitStruct.Pin = TEST_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(TEST_PIN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// printf용 (UART2에 printf 연결) - 레지스터 기반 송신
int _write(int file, char *ptr, int len)
{
    (void)file;
    for (int i = 0; i < len; i++) {
        // TXE 플래그가 세트될 때까지 대기
        while (!(USART2->SR & USART_SR_TXE)) {
            // wait
        }
        USART2->DR = (uint8_t)ptr[i];
    }
    return len;
}

// 방향 → 코드
uint8_t dir_to_code(SoundDir d)
{
    // 원본 코드 기준: enum 값에 +1 (QUIET=0 → 1, LEFT=1 → 2, ...)
    return (uint8_t)d + 1;
}

// SPI로 방향 전송
void A_SPI_SendDir(void)
{
    // 테스트 모드: C보드 연결 없이 동작 확인용
    if (g_spi_enabled == 0) {
        printf("[TEST_MODE] Dir Determined: code=%d (%s) | Current TH=%llu\r\n",
               g_dir_code, dir_to_str(last_result.dir), g_th_silence);
        return;
    }

    // 첫 바이트에 방향 코드, 나머지는 0으로 채움
    slaveTxBuf[0] = g_dir_code;
    for (int i = 1; i < SPI_LEN; i++) {
        slaveTxBuf[i] = 0;
    }

    // C보드(Master)가 SPI_LEN 바이트만큼 클럭을 줄 때,
    // A보드(Slave)는 slaveTxBuf를 MISO로 내보내고, MOSI 데이터를 slaveRxBuf에 받음.
    HAL_StatusTypeDef status =
        HAL_SPI_TransmitReceive(&hspi1, slaveTxBuf, slaveRxBuf, SPI_LEN, 1000);

    if (status == HAL_OK) {
        // 수신된 데이터로 모드 및 임계값 갱신
        uint8_t rx_mode = slaveRxBuf[0];
        if (rx_mode <= NOISE_MODE_NOISY) {
            g_noise_mode = (NoiseMode)rx_mode;
            switch (g_noise_mode) {
                case NOISE_MODE_QUIET:  g_th_silence = TH_SILENCE_QUIET;  break;
                case NOISE_MODE_NORMAL: g_th_silence = TH_SILENCE_NORMAL; break;
                case NOISE_MODE_NOISY:  g_th_silence = TH_SILENCE_NOISY;  break;
            }
        }

        printf("[SPI] Sent dir_code=%d, C->A MOSI: %d | Updated TH=%llu\r\n",
               g_dir_code, slaveRxBuf[0], g_th_silence);

    } else {
        printf("[SPI] TransmitReceive error (status=%d)\r\n", status);
    }
}

// DMA 절반 완료 콜백 (앞쪽 20ms)
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1) {
        copy_to_frame_buf(0);           // 앞쪽 절반 복사
        frame_ready = 1;
    }
}

// DMA 전체 완료 콜백 (뒤쪽 20ms)
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1) {
        copy_to_frame_buf(DMA_BUF_LEN / 2); // 뒤쪽 절반 복사
        frame_ready = 1;
    }
}

// ADC DMA 버퍼 → frame_buf로 복사 (+ DC 오프셋 제거)
void copy_to_frame_buf(uint32_t start_idx)
{
    for (int n = 0; n < FRAME_LEN; n++) {
        uint16_t rawL = adc_buf[start_idx + n * N_CH + 0]; // PA0 (back-left)
        uint16_t rawC = adc_buf[start_idx + n * N_CH + 1]; // PB0 (front)
        uint16_t rawR = adc_buf[start_idx + n * N_CH + 2]; // PC4 (back-right)

        frame_buf[0][n] = (int16_t)rawL - 2048;
        frame_buf[1][n] = (int16_t)rawC - 2048;
        frame_buf[2][n] = (int16_t)rawR - 2048;
    }
}

// 프레임 처리 (에너지 계산 + Cross-correlation + 이벤트 누적)
void process_frame(void)
{
    uint64_t E_L = 0;
    uint64_t E_F = 0;   // FRONT
    uint64_t E_R = 0;
    uint64_t total_E = 0;

    for (int n = 0; n < FRAME_LEN; n++) {
        int16_t l = frame_buf[0][n];  // back-left
        int16_t f = frame_buf[1][n];  // front
        int16_t r = frame_buf[2][n];  // back-right

        E_L += (uint64_t)l * l;
        E_F += (uint64_t)f * f;
        E_R += (uint64_t)r * r;
    }

    // VAD용 전체 에너지: (L + F + R)
    total_E = E_L + E_F + E_R;

    // 소음 임계값 (환경에 맞춰 조절)
    uint64_t current_th = g_th_silence;

    // === 1) 조용해진 프레임 처리 (hangover 적용) ===
    if (total_E < current_th) {

        if (g_event.active) {
            // 이벤트 중인데, 에너지가 잠깐 떨어진 상황
            g_quiet_run++;

            // 아직 충분히 "오래" 조용하지 않으면, 이벤트 유지
            if (g_quiet_run < QUIET_END_FRAMES)
            {
                // 이 프레임에서는 k 계산도 안 하고 그냥 넘어감
                return;
            }

            // 여기까지 왔다는 건: 조용 프레임이 QUIET_END_FRAMES 이상 연속
            if (g_event.n_frames >= EVENT_MIN_FRAMES) {
                finalize_event();
            }
        }

        // 이벤트 완전 종료 및 리셋
        g_event.active   = 0;
        g_event.n_frames = 0;
        for (int i = 0; i < DIR_COUNT; i++) {
            g_event.counts[i] = 0;
        }
        for (int i = 0; i < 11; i++) {
            g_event.k_hist[i] = 0;
        }

        g_quiet_run = 0;  // 조용 카운터도 리셋
        return;
    }

    // 소리가 있는 프레임이 나오면 quiet 카운터 리셋
    g_quiet_run = 0;

    // === 여기부터가 "CPU가 진짜 바쁜 구간" ===
    CPU_BUSY_ON();   // ★ 오실로스코프 HIGH 구간 시작

    // 디버그 출력
    printf("[ENERGY] L=%lu F=%lu R=%lu total=%lu\r\n",
           (uint32_t)E_L,
           (uint32_t)E_F,
           (uint32_t)E_R,
           (uint32_t)total_E);

    // 2. Cross-Correlation (back-left vs back-right, 시간차 계산)
    const int MAX_LAG = 5;      // ±5 샘플 범위
    int64_t corr[11];

    for (int i = 0; i < 11; i++) {
        corr[i] = 0;
    }

    for (int n = MAX_LAG; n < FRAME_LEN - MAX_LAG; n++) {
        int16_t val_L = frame_buf[0][n];

        for (int k = -MAX_LAG; k <= MAX_LAG; k++) {
            int16_t val_R = frame_buf[2][n - k];
            corr[k + MAX_LAG] += (int64_t)val_L * val_R;
        }
    }

    // 3. 가장 큰 상관계수에 해당하는 lag(best_k) 찾기
    int     best_k        = 0;
    int64_t max_corr_val  = -9223372036854775807LL;

    for (int k = -MAX_LAG; k <= MAX_LAG; k++) {
        int idx = k + MAX_LAG;
        if (corr[idx] > max_corr_val) {
            max_corr_val = corr[idx];
            best_k       = k;
        }
    }

    // 4. k값을 방향으로 매핑 (프레임 단위 방향 d)
    //    (의도: ±5만 LEFT/RIGHT, 나머지는 BACK 계열 느낌)
    SoundDir d;

    if (best_k >= 5) {
        d = DIR_RIGHT;
    } else if (best_k >= 2) {
        d = DIR_RIGHT_BACK;
    } else if (best_k >= -1) {
        d = DIR_BACK;
    } else if (best_k >= -4) {
        d = DIR_LEFT_BACK;
    } else {
        d = DIR_LEFT;
    }

    // === 이벤트 단위 누적 ===
    if (!g_event.active) {
        // 새 이벤트 시작
        g_event.active   = 1;
        g_event.n_frames = 0;
        for (int i = 0; i < DIR_COUNT; i++) {
            g_event.counts[i] = 0;
        }
        for (int i = 0; i < 11; i++) {
            g_event.k_hist[i] = 0;
        }
        g_event.sum_front_E = 0;
        g_event.sum_back_E  = 0;
    }

    if (g_event.n_frames < EVENT_MAX_FRAMES) {
        g_event.counts[d]++;        // 방향별 카운트(디버그용)
        g_event.n_frames++;

        // k 히스토그램 누적: index 0~10 -> k = -5~+5
        int idx = best_k + 5;       // -5 -> 0, ..., +5 -> 10
        if (idx >= 0 && idx < 11) {
            g_event.k_hist[idx]++;
        }
        // 이번 프레임의 front/back 에너지 누적
        g_event.sum_front_E += E_F;
        g_event.sum_back_E  += (E_L + E_R);
    }

    // 디버그용 프레임 로그
    printf("[FRAME] k=%2d | d=%-10s | totalE=%lu | event_frames=%d\r\n",
           best_k, dir_to_str(d), (uint32_t)total_E, g_event.n_frames);

    // 프레임을 충분히 모았으면 이벤트 종료 후 최종 판단
    if (g_event.n_frames >= EVENT_MAX_FRAMES) {
        finalize_event();

        g_event.active   = 0;
        g_event.n_frames = 0;
        for (int i = 0; i < DIR_COUNT; i++) {
            g_event.counts[i] = 0;
        }
        for (int i = 0; i < 11; i++) {
            g_event.k_hist[i] = 0;
        }
        g_event.sum_front_E = 0;
        g_event.sum_back_E  = 0;
    }

    CPU_BUSY_OFF();  // ★ heavy 연산 구간 끝나면 LOW로 내려줌
}

// 한 번 말하기 이벤트가 끝났을 때, 프레임들을 모아서 최종 방향 결정
void finalize_event(void)
{
    uint16_t total = g_event.n_frames;
    if (total < EVENT_MIN_FRAMES) {
        // 너무 짧은 이벤트는 버림
        return;
    }

    // 1) 기존 방향 카운트 (디버그용)
    uint16_t cL  = g_event.counts[DIR_LEFT];
    uint16_t cLB = g_event.counts[DIR_LEFT_BACK];
    uint16_t cB  = g_event.counts[DIR_BACK];
    uint16_t cRB = g_event.counts[DIR_RIGHT_BACK];
    uint16_t cR  = g_event.counts[DIR_RIGHT];

    // 2) k 히스토그램 기반 그룹 집계 (의도: 끝단 ±5를 순수 LEFT/RIGHT로)
    // idx 0~10 -> k = -5 ~ +5
    uint16_t c_left      = g_event.k_hist[0];                                  // k = -5
    uint16_t c_left_back = g_event.k_hist[1] + g_event.k_hist[2] + g_event.k_hist[3]; // k = -4,-3,-2
    uint16_t c_back      = g_event.k_hist[4] + g_event.k_hist[5] + g_event.k_hist[6]; // k = -1,0,1
    uint16_t c_right_back= g_event.k_hist[7] + g_event.k_hist[8] + g_event.k_hist[9]; // k = 2,3,4
    uint16_t c_right     = g_event.k_hist[10];                                 // k = +5

    uint16_t left_group_total  = c_left + c_left_back;        // left 계열 전체
    uint16_t right_group_total = c_right + c_right_back;      // right 계열 전체

    // 각 핵심 bin 비율 (%)
    uint16_t p_left  = (uint16_t)(100u * c_left  / total);
    uint16_t p_right = (uint16_t)(100u * c_right / total);
    uint16_t p_back  = (uint16_t)(100u * c_back  / total);

    SoundDir final_dir;

    // ===== k 기반 방향 판단 로직 =====
    // 1) right bin(끝단 +5) 비율이 크면 -> RIGHT
    if (p_right >= K_EDGE_STRONG_PCT && p_right >= p_left && p_right >= p_back) {
        final_dir = DIR_RIGHT;
    }
    // 2) left bin(끝단 -5) 비율이 크면 -> LEFT
    else if (p_left >= K_EDGE_STRONG_PCT && p_left >= p_right && p_left >= p_back) {
        final_dir = DIR_LEFT;
    }
    // 3) back bin(-1,0,1) 비율이 크면 -> BACK
    else if (p_back >= K_CENTER_STRONG_PCT) {
        final_dir = DIR_BACK;
    }
    // 4) 셋 다 애매하면: 계열 비교로 LEFT_BACK / RIGHT_BACK
    else {
        if (left_group_total > right_group_total) {
            final_dir = DIR_LEFT_BACK;
        } else if (right_group_total > left_group_total) {
            final_dir = DIR_RIGHT_BACK;
        } else {
            // 완전 비슷하면 BACK으로 처리
            final_dir = DIR_BACK;
        }
    }
    // =========================

    // ===== 여기부터 앞/뒤 에너지 비율로 필터링 =====
    uint64_t frontE = g_event.sum_front_E;      // 이벤트 전체 FRONT 에너지
    uint64_t backE  = g_event.sum_back_E;       // 이벤트 전체 BACK(L+R) 에너지
    uint8_t  is_front = 0;

    if (backE == 0) {
        // 등쪽이 거의 0인데 앞만 크면 그냥 앞소리로 본다
        if (frontE > 0) {
            is_front = 1;
        }
    } else {
        // frontE / backE >= FRONT_BACK_RATIO_X100 / 100 ?
        if (frontE * 100 >= backE * FRONT_BACK_RATIO_X100) {
            is_front = 1;
        }
    }

    if (is_front) {
        // 앞에서 난 소리면: 방향 안 보냄
        last_result.dir = DIR_QUIET;
        g_dir_code = 0;   // C보드에서 0을 "방향 없음(앞소리)"로 처리

        printf(">>> FRONT-DOMINANT EVENT: frontE=%llu, backE=%llu -> IGNORE DIRECTION\r\n",
               (unsigned long long)frontE,
               (unsigned long long)backE);
    } else {
        // 뒤쪽(등)에서 난 소리면 기존대로 방향 사용
        last_result.dir = final_dir;
        g_dir_code = dir_to_code(final_dir);
    }

    // 이벤트 요약 로그
    printf("=== EVENT DONE ===\r\n");
    printf("  DIR-counts L=%d LB=%d B=%d RB=%d R=%d (frames=%d)\r\n",
           cL, cLB, cB, cRB, cR, total);
    printf("  k-bins: L(-5)=%d, LB(-4,-3,-2)=%d, B(-1,0,1)=%d, RB(2,3,4)=%d, R(+5)=%d\r\n",
           c_left, c_left_back, c_back, c_right_back, c_right);
    printf("  pL=%d pR=%d pB=%d | leftG=%d rightG=%d -> FINAL=%s (code=%d)\r\n",
           p_left, p_right, p_back,
           left_group_total, right_group_total,
           dir_to_str(is_front ? DIR_QUIET : final_dir), g_dir_code);

    // ★ 최종 방향 결정되었을 때 SPI로 바로 전송 ★
    A_SPI_SendDir();
}

// 문자열 변환
const char* dir_to_str(SoundDir d)
{
    switch (d) {
        case DIR_QUIET:      return "QUIET";
        case DIR_LEFT:       return "LEFT";
        case DIR_LEFT_BACK:  return "LEFT_BACK";
        case DIR_BACK:       return "BACK";
        case DIR_RIGHT_BACK: return "RIGHT_BACK";
        case DIR_RIGHT:      return "RIGHT";
        default:             return "???";
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
