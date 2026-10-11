/*
 * Reference source recovered from project 2 report, PDF physical pages 74-92.
 * Page markers removed, trailing whitespace trimmed, missing first comment
 * opener restored. The truncated assert_failed closing brace and #endif were
 * restored at EOF. main.h and project configuration are required.
 * This is not a rebuilt or hardware-verified standalone project.
 */

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
#include <stdio.h>
#include <string.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// 레지스터 기반 GPIO write
static inline void GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    if (state == GPIO_PIN_SET)
        port->BSRR = pin;
    else
        port->BRR = pin;
}

// 레지스터 기반 GPIO read
static inline GPIO_PinState GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    return (port->IDR & pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


// A 보드 CS : PB0
#define CS_A_GPIO_Port   GPIOB
#define CS_A_Pin         (1U << 0)     // PB0
#define CS_A_LOW()       GPIO_WritePin(CS_A_GPIO_Port, CS_A_Pin, GPIO_PIN_RESET)
#define CS_A_HIGH()      GPIO_WritePin(CS_A_GPIO_Port, CS_A_Pin, GPIO_PIN_SET)

// B 보드 CS : PB1
#define CS_B_GPIO_Port   GPIOB
#define CS_B_Pin         (1U << 1)     // PB1
#define CS_B_LOW()       GPIO_WritePin(CS_B_GPIO_Port, CS_B_Pin, GPIO_PIN_RESET)
#define CS_B_HIGH()      GPIO_WritePin(CS_B_GPIO_Port, CS_B_Pin, GPIO_PIN_SET)

// SW_A, SW_B : 실제 핀 번호에 맞게 수정 (예: PA10, PA11)
#define SW_A_GPIO_Port   GPIOA
#define SW_A_Pin         (1U << 10)    // PA10
#define SW_B_GPIO_Port   GPIOA
#define SW_B_Pin         (1U << 11)    // PA11

// LED 핀들 : 실제 핀 번호에 맞게 수정 (예시: PB5~PB9)
#define LED_LEFT_Pin       (1U << 5)   // PB5
#define LED_LEFTBACK_Pin   (1U << 6)   // PB6
#define LED_BACK_Pin       (1U << 7)   // PB7
#define LED_RIGHTBACK_Pin  (1U << 8)   // PB8
#define LED_RIGHT_Pin      (1U << 9)   // PB9

// 디바운스 시간 (ms)
#define SW_DEBOUNCE_MS   30

// B보드 분류 결과 클래스 개수 (IDLE + 5동작, ID = 1~6 사용)
#define B_NUM_CLASSES 7   // index 0은 미사용
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// 디바운싱용 전역 변수
static uint8_t  sw_a_stable_state     = GPIO_PIN_SET;   // 풀업이라 기본 HIGH
static uint8_t  sw_a_last_raw         = GPIO_PIN_SET;
static uint32_t sw_a_last_change_ms   = 0;
uint8_t g_c2a_flag = 2; // 기본값 (HIGH → 2 / LOW → 1)



static uint8_t  sw_b_stable_state     = GPIO_PIN_SET;   // 풀업이라 기본 HIGH
static uint8_t  sw_b_last_raw         = GPIO_PIN_SET;
static uint32_t sw_b_last_change_ms   = 0;
uint8_t g_c2b_flag = 1; // 1 = 간단어, 2 = 문장

// === 간단어 모드 (g_c2b_flag == 1)에서의 클래스 → 단어 매핑 ===
static const char* B_Phrases_Simple[B_NUM_CLASSES] =
{
    "",             // [0]
    "IDLE",         // [1]
    "HELLO",        // [2] = HELLO
    "I'm Okay",     // [3] = IM_OKAY
    "Thank you",    // [4] = THANK_YOU
    "No thank you", // [5] = NO_THANK_YOU
    "HELP"          // [6] = HELP
};


// 문장 모드 S/V/O 단계
typedef enum {
    SENT_STAGE_S = 0,
    SENT_STAGE_V,
    SENT_STAGE_O
} SentenceStage_t;

// 현재 문장 모드에서 S/V/O 중 어디 단계인지
static SentenceStage_t g_sentence_stage = SENT_STAGE_S;

// 현재까지 인식된 S, V, O 단어를 저장해두는 버퍼
static const char* g_word_S = " ";
static const char* g_word_V = " ";
static const char* g_word_O = " ";

uint8_t last_dir = 0;  // 방향 정보

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void C_Send_ReadDir_From_A(void);
void C_SetLED_ByCode(uint8_t code);
void C_ReadSentence_From_B(void);
static void C_ClearSentenceDisplay();
// LCD 함수 프로토타입
void LCD_Init(void);


void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_Print(const char *str);

void SW_A_UpdateFlag(void);
void SW_B_UpdateFlag(void);

/* SPI 레지스터 기반 1바이트 TxRx */
static uint8_t SPI_TxRx(SPI_TypeDef *SPIx, uint8_t tx);

/* I2C 레지스터 기반 1바이트 송신(LCD용) */
static void LCD_I2C_Send(uint8_t data);
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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();


  /* USER CODE BEGIN 2 */
  LCD_Init();
  LCD_Clear();
  LCD_SetCursor(0,0);
  LCD_Print("System Ready");   // 초기 LCD 메시지
  uint32_t last_spi_a_ms = 0;   // A보드(LED용) SPI1 타이머
  uint32_t last_spi_b_ms = 0;   // B보드(LCD용) SPI2 타이머
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
   // 1) 스위치는 최대한 자주 체크 (디바운싱 포함)
   SW_A_UpdateFlag();
   SW_B_UpdateFlag();
   // 2) SPI 는 1초마다만 수행
   uint32_t now = HAL_GetTick();
   // === A 보드: LED용 SPI1 — 1 초마다 ===
   if ((uint32_t)(now - last_spi_a_ms) >= 1000)   // 1000ms = 1초
   {
       last_spi_a_ms = now;
       C_Send_ReadDir_From_A();   // 방향 코드 받아서 LED 제어
   }
   if ((uint32_t)(now - last_spi_b_ms) >= 1000)   // 1000ms = 1초
   {
       last_spi_b_ms = now;
       C_ReadSentence_From_B();   // 수화 문장
   }


   // 필요하면 아주 짧게 쉼 (CPU 너무 바쁜게 신경 쓰이면)
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

  /** Initializes the RCC Oscillators according to the specified parameters


  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
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
  * @brief SPI2 Initialization Function
  * @param None


  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  // SCK + MOSI
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // MISO
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB,
LED_LEFTBACK_Pin|LED_LEFT_Pin|LED_RIGHT_Pin|LED_BACK_Pin|LED_RIGHTBACK_Pin,
GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */


  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 LED_LEFT_Pin LED_LEFTBACK_Pin
                           LED_RIGHT_Pin LED_BACK_Pin LED_RIGHTBACK_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|LED_LEFT_Pin|LED_LEFTBACK_Pin
                          |LED_RIGHT_Pin|LED_BACK_Pin|LED_RIGHTBACK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_B_Pin SW_A_Pin */
  GPIO_InitStruct.Pin = SW_B_Pin|SW_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // ★ SW_B 입력 핀 설정 (PA11, 풀업 입력)
  GPIO_InitStruct.Pin  = SW_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;   // 스위치 한 쪽을 GND에 연결했다면 PULLUP
  HAL_GPIO_Init(SW_B_GPIO_Port, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void delay_ms(uint32_t ms)
{
    uint32_t start = HAL_GetTick();  // SysTick으로부터 현재 ms 읽기
    while ((HAL_GetTick() - start) < ms) {
        __NOP();                    // 최적화 방지용 (있어도 되고 없어도 됨)
    }
}

// printf → UART 출력
int _write(int file, char *ptr, int len)
{
    (void)file;
    for (int i = 0; i < len; i++) {
        while (!(USART2->SR & USART_SR_TXE)); // TXE=1 대기
        USART2->DR = (uint8_t)ptr[i];
    }


    while (!(USART2->SR & USART_SR_TC));      // 전송 완료
    return len;
}
// SPI 1바이트 송수신 (Master)
static uint8_t SPI_TxRx(SPI_TypeDef *SPIx, uint8_t tx)
{
    // TXE=1 대기
    while (!(SPIx->SR & SPI_SR_TXE));
    *(volatile uint8_t *)&SPIx->DR = tx;

    // RXNE=1 대기
    while (!(SPIx->SR & SPI_SR_RXNE));
    return *(volatile uint8_t *)&SPIx->DR;
}
//uint8_t last_dir = 0;  // 0 = 아직 유효한 방향 없음

void C_Send_ReadDir_From_A(void)
{
    uint8_t txData = g_c2a_flag;
    uint8_t rxData[5];

    CS_A_LOW();
    delay_ms(1);
    for (int i = 0; i < 5; i++)
        rxData[i] = SPI_TxRx(SPI1, txData);
    CS_A_HIGH();

    uint8_t dir_code = rxData[0];  // 0(없음) or 1~6(유효)
    static uint8_t prev_printed_dir = 0xFF;     // UART 로그용
    static uint32_t led_deadline_ms = 0;       // LED 꺼질 시간(ms)

    // 새 방향 코드가 들어오면 last_dir 갱신
    if (dir_code != 0) {
        last_dir = dir_code;

        printf("[A] dir_now=%d, dir_hold=%d\r\n", dir_code, last_dir);
        // LED 켜고, 1초 뒤에 끌 수 있도록 데드라인 설정
        C_SetLED_ByCode(last_dir);
        led_deadline_ms = HAL_GetTick() + 1000;

        if (last_dir != prev_printed_dir) {
            prev_printed_dir = last_dir;
        }
    }

    // ★ 1초 경과 시 LED 모두 OFF
    if (led_deadline_ms &&
        (int32_t)(HAL_GetTick() - led_deadline_ms) >= 0)
    {
        led_deadline_ms = 0;
        C_SetLED_ByCode(1);   // 1번을 ALLOFF로 정의해 놓았으니까 All OFF


    }
}

// ★ 방향 코드에 따라 LED 제어
void C_SetLED_ByCode(uint8_t code)
{
  // 모든 LED OFF
     GPIO_WritePin(GPIOB, LED_LEFT_Pin | LED_LEFTBACK_Pin |
                          LED_BACK_Pin | LED_RIGHTBACK_Pin |
                          LED_RIGHT_Pin,
                   GPIO_PIN_RESET);
    switch (code)
    {
        case 1: // ALLOFF
            // 이미 다 꺼져 있음
            break;

        case 2: // LEFT
            GPIO_WritePin(GPIOB, LED_LEFT_Pin, GPIO_PIN_SET);
            break;

        case 3: // LEFT_BACK
            GPIO_WritePin(GPIOB, LED_LEFTBACK_Pin, GPIO_PIN_SET);
            break;

        case 4: // BACK
            GPIO_WritePin(GPIOB, LED_BACK_Pin, GPIO_PIN_SET);
            break;

        case 5: // RIGHT_BACK
            GPIO_WritePin(GPIOB, LED_RIGHTBACK_Pin, GPIO_PIN_SET);
            break;

        case 6: // RIGHT
            GPIO_WritePin(GPIOB, LED_RIGHT_Pin, GPIO_PIN_SET);
            break;

        default:
            // 이상한 값 → 그냥 ALL OFF 유지
            break;
    }
}

// === 문장 모드: 단계(S/V/O) + 클래스 → 단어 매핑 ===
const char* B_GetWord_S(uint8_t stage, uint8_t class_id)
{
    switch (stage)
    {
    // ===================== S 단계 =====================
    case SENT_STAGE_S:
        switch (class_id)


        {
        case 2: return "I";       // SENT_S_SET[0] = NN_I
        case 3: return "YOU";     // SENT_S_SET[1] = NN_YOU
        case 4: return "WE";      // SENT_S_SET[2] = NN_WE
        case 5: return "HE";      // SENT_S_SET[3] = NN_HE
        case 6: return "THEY";    // SENT_S_SET[4] = NN_THEY
        default: return "S?";
        }

    // ===================== V 단계 =====================
    case SENT_STAGE_V:
        switch (class_id)
        {
        case 2: return "WANT";    // SENT_V_SET[0] = NN_WANT
        case 3: return "GO";      // SENT_V_SET[1] = NN_GO
        case 4: return "COME";    // SENT_V_SET[2] = NN_COME
        case 5: return "EAT";     // SENT_V_SET[3] = NN_EAT
        case 6: return "DRINK";   // SENT_V_SET[4] = NN_DRINK
        default: return "V?";
        }

    // ===================== O 단계 =====================
    case SENT_STAGE_O:
        switch (class_id)
        {
        case 2: return "WATER";     // SENT_O_SET[0] = NN_WATER
        case 3: return "MONEY";     // SENT_O_SET[1] = NN_MONEY
        case 4: return "HOME";      // SENT_O_SET[2] = NN_HOME
        case 5: return "HOSPITAL";  // SENT_O_SET[3] = NN_HOSPITAL
        case 6: return "TOILET";    // SENT_O_SET[4] = NN_TOILET
        default: return "O?";
        }

    default:
        return "?";
    }
}


void C_ReadSentence_From_B(void)
{
    uint8_t tx       = g_c2b_flag; // 1=간단어, 2=문장
    uint8_t rx_class = 0;

    CS_B_LOW();
    delay_ms(1);
    rx_class = SPI_TxRx(SPI2, tx);
    CS_B_HIGH();

    static uint8_t last_class = 0xFF;



    // 유효 ID 범위: 1~6 (1=IDLE)
    if (rx_class < 1 || rx_class > 6)
    {
        if (last_class != 1)
        {
            printf("[B] invalid class=%d, go IDLE\r\n", rx_class);
            C_ClearSentenceDisplay();
            last_class = 1;
        }
        return;
    }

    // === 모드 1: 간단어 모드 ===
    if (g_c2b_flag == 1)
    {
        if (rx_class == 1)
        {
            if (last_class != 1)
            {
                printf("[B] SIMPLE: IDLE, clear display\r\n");
                C_ClearSentenceDisplay();
                last_class = 1;
            }
            return;
        }

        const char* msg = B_Phrases_Simple[rx_class];
        printf("[B] SIMPLE: class=%d, msg=%s\r\n", rx_class, msg);

        LCD_Clear();
        LCD_SetCursor(0, 0);
        LCD_Print("SIGN:");
        LCD_SetCursor(1, 0);
        LCD_Print(msg);

        last_class = rx_class;
        return;
    }

    // === 모드 2: 문장 모드 ===
    if (g_c2b_flag == 2)
    {
        if (rx_class == 1)
        {
            printf("[B] SENT: class=IDLE, keep sentence\r\n");
            return;
        }

        const char* new_word = B_GetWord_S(g_sentence_stage, rx_class);

        if (g_sentence_stage == SENT_STAGE_S)


        {
            g_word_S = new_word;
            g_word_V = " ";
            g_word_O = " ";
        }
        else if (g_sentence_stage == SENT_STAGE_V)
        {
            g_word_V = new_word;
        }
        else if (g_sentence_stage == SENT_STAGE_O)
        {
            g_word_O = new_word;
        }

        printf("[B] SENT: stage=%d, class=%d, S=%s, V=%s, O=%s\r\n",
               g_sentence_stage, rx_class, g_word_S, g_word_V, g_word_O);

        LCD_Clear();
        LCD_SetCursor(0, 0);
        LCD_Print("SIGN:");
        LCD_SetCursor(1, 0);
        LCD_Print(g_word_S);
        LCD_Print(" ");
        LCD_Print(g_word_V);
        LCD_Print(" ");
        LCD_Print(g_word_O);

        if (g_sentence_stage == SENT_STAGE_S)
            g_sentence_stage = SENT_STAGE_V;
        else if (g_sentence_stage == SENT_STAGE_V)
            g_sentence_stage = SENT_STAGE_O;
        else
            g_sentence_stage = SENT_STAGE_S;

        last_class = rx_class;
        return;
    }

    // 혹시 g_c2b_flag가 1,2가 아니면 Idle 처리
    if (last_class != 1)
    {
        printf("[B] unknown flag=%d, go IDLE\r\n", g_c2b_flag);
        C_ClearSentenceDisplay();
        last_class = 1;
    }
}

// ★ LCD를 Idle 화면으로 초기화
static void C_ClearSentenceDisplay(void)
{
    LCD_Clear();


    LCD_SetCursor(0, 0);
    LCD_Print("SIGN:");
    LCD_SetCursor(1, 0);
    LCD_Print("Idle");   // 원하는 기본 문구
}


// ===== LCD (QAPASS 1602 + I2C PCF8574) 설정 =====
#define LCD_ADDR (0x27 << 1)
#define LCD_BACKLIGHT 0x08
#define LCD_EN        0x04
#define LCD_RS        0x01

static void LCD_I2C_Send(uint8_t data)
{
    // START
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));
    (void)I2C1->SR1;

    // Address (write)
    I2C1->DR = LCD_ADDR;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR1;
    (void)I2C1->SR2;

    // Data
    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = data;

    // BTF
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    // STOP
    I2C1->CR1 |= I2C_CR1_STOP;
}

static void LCD_PulseEnable(uint8_t data)
{
    LCD_I2C_Send(data | LCD_EN);
    delay_ms(1);
    LCD_I2C_Send(data & ~LCD_EN);
    delay_ms(1);
}

static void LCD_Send4Bits(uint8_t data)
{
    LCD_I2C_Send(data | LCD_BACKLIGHT);
    LCD_PulseEnable(data | LCD_BACKLIGHT);
}

static void LCD_SendCmd(uint8_t cmd)


{
    uint8_t high = cmd & 0xF0;
    uint8_t low  = (cmd << 4) & 0xF0;

    // RS=0 (명령)
    LCD_Send4Bits(high);
    LCD_Send4Bits(low);
}

static void LCD_SendData(uint8_t data)
{
    uint8_t high = data & 0xF0;
    uint8_t low  = (data << 4) & 0xF0;

    // RS=1 (데이터)
    LCD_Send4Bits(high  | LCD_RS);
    LCD_Send4Bits(low   | LCD_RS);
}

void LCD_Init(void)
{
    delay_ms(50);   // 파워온 대기

    // 4비트 모드 초기화 시퀀스
    LCD_Send4Bits(0x30);
    delay_ms(5);
    LCD_Send4Bits(0x30);
    delay_ms(5);
    LCD_Send4Bits(0x30);
    delay_ms(5);
    LCD_Send4Bits(0x20);   // 4-bit mode

    // 함수 설정: 4bit, 2라인, 5x8 폰트
    LCD_SendCmd(0x28);
    // 디스플레이 ON, 커서 OFF, 블링크 OFF
    LCD_SendCmd(0x0C);
    // 엔트리 모드: 증가, shift off
    LCD_SendCmd(0x06);
    // 화면 클리어
    LCD_SendCmd(0x01);
    delay_ms(2);
}

void LCD_Clear(void)
{
    LCD_SendCmd(0x01);
    delay_ms(2);
}

void LCD_SetCursor(uint8_t row, uint8_t col)
{


    uint8_t addr = (row == 0) ? 0x00 : 0x40;
    addr += col;
    LCD_SendCmd(0x80 | addr);
}

void LCD_Print(const char *str)
{
    while (*str)
    {
        LCD_SendData((uint8_t)(*str));
        str++;
    }
}

void SW_A_UpdateFlag(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t raw = GPIO_ReadPin(SW_A_GPIO_Port, SW_A_Pin);  // 0=LOW, 1=HIGH

    if (raw != sw_a_last_raw)
    {
        sw_a_last_raw       = raw;
        sw_a_last_change_ms = now;
        return;
    }

    if ((now - sw_a_last_change_ms) < SW_DEBOUNCE_MS)
        return;

    if (raw != sw_a_stable_state)
    {
        // HIGH → LOW 되는 순간(하강 에지)
        if (sw_a_stable_state == GPIO_PIN_SET && raw == GPIO_PIN_RESET)
        {
            if (g_c2a_flag == 1)      g_c2a_flag = 2;
            else if (g_c2a_flag == 2) g_c2a_flag = 3;
            else                      g_c2a_flag = 1;

            printf("[C] SW_A toggle: flag=%d\r\n", g_c2a_flag);
        }
        sw_a_stable_state = raw;
    }
}

void SW_B_UpdateFlag(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t raw = GPIO_ReadPin(SW_B_GPIO_Port, SW_B_Pin);  // 0=LOW, 1=HIGH

    if (raw != sw_b_last_raw)
    {


        sw_b_last_raw       = raw;
        sw_b_last_change_ms = now;
        return;
    }

    if ((now - sw_b_last_change_ms) < SW_DEBOUNCE_MS)
        return;

    if (raw != sw_b_stable_state)
    {
        // HIGH → LOW 되는 순간(하강 에지)
        if (sw_b_stable_state == GPIO_PIN_SET && raw == GPIO_PIN_RESET)
        {
            g_c2b_flag = (g_c2b_flag == 1) ? 2 : 1;
            printf("[C] SW_B falling edge, toggle flag=%d\r\n", g_c2b_flag);
        }
        sw_b_stable_state = raw;
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
  /* Restoration note: the report ends here; close its assert_failed template. */
}
#endif /* USE_FULL_ASSERT: closing directive restored from truncated report */
