/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "SX1278.h"
#include "BME280_I2C.h"
#include "esp01s_driver.h"
#include "debug.h"
#include "st7735.h"
#include "fonts.h"
#include "image.h"

#include "sd_logger.h"
#include "structs.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CYCLE_PERIOD_MS 60000    // Базовий хвилинний інтервал опитування
#define SLOT_OFFSET_MS  2000     // Зсув слоту (по 2 секунди для кожного вузла)

#define DIO0_GPIO_Port GPIOB
#define DIO0_Pin GPIO_PIN_1

#define NSS_GPIO_Port GPIOA
#define NSS_Pin GPIO_PIN_4

#define RESET_GPIO_Port GPIOB
#define RESET_Pin GPIO_PIN_0

#define MAX_NODES 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DBG_UART &huart1
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

uint32_t last_button_press = 0;

NodeData node_cache[MAX_NODES];
uint8_t node_valid[MAX_NODES] = {0};
uint8_t node_unsaved[MAX_NODES] = {0};
volatile uint8_t selected_node = 0;
volatile uint8_t node_switch_flag = 0;


uint32_t last_log_time = 0;
const uint32_t LOG_INTERVAL = 5000;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_IWDG_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void Led_Blink_Count(uint8_t n, uint8_t delay);
uint8_t crc8(uint8_t *data, uint8_t len);
NodeData read_lora_packet(uint8_t *buf);
void draw_intro(){
	ST7735_FillScreen(ST7735_BLACK);
	ST7735_DrawString(28,90,"Vadym Minder", Font_5x7, ST7735_WHITE, ST7735_BLACK);
	ST7735_DrawString(49,98,"KV-22", Font_5x7, ST7735_WHITE, ST7735_BLACK);
	ST7735_DrawImage(40, 8, 48, 67, icon);
}

void draw_interface(){
	ST7735_DrawRect(0, 16, 128, 112, ST7735_WHITE);
	ST7735_DrawFastHLine(0, 80, 128, ST7735_WHITE);
	//ST7735_DrawFastHLine(0, 104, 128, ST7735_WHITE);
	ST7735_DrawFastVLine(64, 80, 48, ST7735_WHITE);
	ST7735_DrawString(98, 46, "^C", Font_11x18, ST7735_WHITE, ST7735_BLACK);
	ST7735_DrawString(9, 110, "HUMIDITY", Font_5x7, ST7735_WHITE, ST7735_BLACK);
	ST7735_DrawString(88, 110, "hPa", Font_5x7, ST7735_WHITE, ST7735_BLACK);
}

void draw_measurements(const NodeData *d);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float Temperature, Pressure, Humidity;

SX1278_hw_t SX1278_hw;
SX1278_t SX1278;

volatile uint8_t lora_rx_flag = 0;
char buffer[64];
char buffer_BME[64];

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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_USART6_UART_Init();
  MX_IWDG_Init();
  MX_FATFS_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(500);
  debug_log("STM32_CORE","MODULES INIT START");

  char dbg[32];
  snprintf(dbg, sizeof(dbg), "USERPath: '%s' ret:%d\r\n", USERPath, retUSER);
  HAL_UART_Transmit(&huart1, (uint8_t*)dbg, strlen(dbg), 100);
  if (SD_Logger_Init() != SD_LOG_OK) {
          debug_log("STM32_CORE", "SD INIT FAIL");
      }
  ST7735_Init();
  ST7735_Backlight_On();
  ST7735_SetRotation(1);
  draw_intro();



  SX1278_hw.dio0.port  = DIO0_GPIO_Port;
  SX1278_hw.dio0.pin   = DIO0_Pin;

  SX1278_hw.nss.port   = NSS_GPIO_Port;
  SX1278_hw.nss.pin    = NSS_Pin;

  SX1278_hw.reset.port = RESET_GPIO_Port;
  SX1278_hw.reset.pin  = RESET_Pin;

  SX1278_hw.spi        = &hspi1;

  SX1278.hw = &SX1278_hw;

  SX1278_init(&SX1278, 434000000, SX1278_POWER_17DBM, SX1278_LORA_SF_7, SX1278_LORA_BW_125KHZ, SX1278_LORA_CR_4_5, SX1278_LORA_CRC_EN, 10);

  SX1278_LoRaEntryRx(&SX1278, 16, 3000);

  BME280_Config(OSRS_2, OSRS_16, OSRS_1, MODE_NORMAL, T_SB_0p5, IIR_16);


  uint8_t lora_packet[9];
  NodeData hub = {0, Temperature, Humidity, Pressure};
  node_cache[0] = hub;
  node_valid[0] = 1;
  debug_log("STM32_CORE","MODULES INIT END");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint8_t intro_complete = 0; //[cite: 1]
    while (1)
    {
      HAL_IWDG_Refresh(&hiwdg); //[cite: 1]

      // 1. Блок збору та логування даних
      if (HAL_GetTick() - last_log_time >= LOG_INTERVAL) { //[cite: 1]
          last_log_time = HAL_GetTick(); //[cite: 1]

          RTC_TimeTypeDef sTime = {0};
          RTC_DateTypeDef sDate = {0};
          HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
          HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

          //debug_log("STM32_CORE","BME280 Measure START"); //[cite: 1]
          BME280_Measure(); //[cite: 1]
          NodeData hub = {0, Temperature, Humidity, Pressure/100.0f}; //[cite: 1]

          node_cache[0] = hub; //[cite: 1]
          node_valid[0] = 1;     // Для екрану (ніколи не скидаємо)[cite: 1]
          node_unsaved[0] = 1;   // Для SD/ESP (скинемо після запису)

          debug_log("STM32_CORE","BME280 Measure END"); //[cite: 1]

          // Пакетне збереження накопичених даних
          for (uint8_t i = 0; i < MAX_NODES; i++) { //[cite: 1]
              if (node_unsaved[i]) { // Змінили перевірку на новий масив
                  if (i == 0) {
                      //debug_log("STM32_CORE","SD_Log START"); //[cite: 1]
                      SD_Logger_Write(&node_cache[i]); //[cite: 1]
                      //debug_log("STM32_CORE","SD_Log END"); //[cite: 1]

                      //debug_log("STM32_CORE","ESP SEND START"); //[cite: 1]
                      ESP_SendMessage_Node(&node_cache[i]); //[cite: 1]
                      debug_log("STM32_CORE","ESP SEND END"); //[cite: 1]
                  } else {
                      SD_Logger_Write(&node_cache[i]); //[cite: 1]
                      ESP_SendMessage_Node(&node_cache[i]); //[cite: 1]
                      debug_log("STM32_CORE","ESP SEND lora END"); //[cite: 1]
                  }
                  node_unsaved[i] = 0; // Скидаємо ТІЛЬКИ прапорець збереження!
              }
          }
          SD_Logger_Flush(); //[cite: 1]
      }

      // 2. Блок обробки вхідних радіопакетів
      if (lora_rx_flag) { //[cite: 1]
          lora_rx_flag = 0; //[cite: 1]

          uint8_t len = SX1278_available(&SX1278); //[cite: 1]
          if (len > 0) { //[cite: 1]
              SX1278_read(&SX1278, (uint8_t*)lora_packet, len); //[cite: 1]
              NodeData node = read_lora_packet(lora_packet); //[cite: 1]

              if (node.id > 0 && node.id < MAX_NODES) {
                  node_cache[node.id] = node;
                  node_valid[node.id] = 1;
                  node_unsaved[node.id] = 1;

                  if (selected_node == node.id) {
                      draw_measurements(&node_cache[node.id]);
                  }

                  // --- РОЗРАХУНОК СЛОТУ ---
                  // Фіксуємо час ЗАРАЗ — до будь-яких затримок передачі
                  uint32_t now        = HAL_GetTick();
                  uint32_t cycle_time = now % CYCLE_PERIOD_MS;
                  uint32_t target_offset = (uint32_t)node.id * SLOT_OFFSET_MS;

                  // Скільки мс до наступного входження вузла у свій слот
                  int32_t time_to_slot = (int32_t)target_offset - (int32_t)cycle_time;
                  if (time_to_slot <= 500) {
                      // Слот вже минув або дуже близько — беремо наступний цикл
                      time_to_slot += (int32_t)CYCLE_PERIOD_MS;
                  }
                  uint32_t sleep_ms = (uint32_t)time_to_slot;

                  // Drift для діагностики (позитивний = вузол прийшов пізніше слоту)
                  int32_t drift_ms = (int32_t)cycle_time - (int32_t)target_offset;
                  if (drift_ms >  (int32_t)(CYCLE_PERIOD_MS / 2)) drift_ms -= CYCLE_PERIOD_MS;
                  if (drift_ms < -(int32_t)(CYCLE_PERIOD_MS / 2)) drift_ms += CYCLE_PERIOD_MS;

                  char drift_log[48];
                  snprintf(drift_log, sizeof(drift_log),
                           "Node %d drift: %ld ms  sleep: %lu ms", node.id, drift_ms, sleep_ms);
                  debug_log("STM32_CORE", drift_log);
                  char fine_log[48];
                  uint32_t fine_ms_real = (uint32_t)node.flags * 10;
                  snprintf(fine_log, sizeof(fine_log),
                           "Node %d fine: %lu ms", node.id, fine_ms_real);
                  debug_log("STM32_CORE", fine_log);

                  // --- ACK ---
                  uint8_t ack_packet[6];
                  ack_packet[0] = node.id;
                  ack_packet[1] = (sleep_ms >> 24) & 0xFF;
                  ack_packet[2] = (sleep_ms >> 16) & 0xFF;
                  ack_packet[3] = (sleep_ms >>  8) & 0xFF;
                  ack_packet[4] =  sleep_ms        & 0xFF;
                  ack_packet[5] = crc8(ack_packet, 5);

                  SX1278_standby(&SX1278);
                  HAL_Delay(20);
                  SX1278_LoRaEntryTx(&SX1278, 16, 3000);
                  SX1278_transmit(&SX1278, ack_packet, 6, 1000);
                  debug_log("STM32_CORE", "ACK Sent to Node");
              }
          }

          SX1278_LoRaEntryRx(&SX1278, 16, 3000); //[cite: 1]
      }

      // 3. Інтерфейс
      if (intro_complete == 0) { //[cite: 1]
          ST7735_FillScreen(ST7735_BLACK); //[cite: 1]
          draw_interface(); //[cite: 1]
          intro_complete = 1; //[cite: 1]
      }

      // Перемикання кнопкою (працюватиме, бо node_valid не скидається)
      if (node_switch_flag) { //[cite: 1]
          node_switch_flag = 0; //[cite: 1]
          if (node_valid[selected_node]) { //[cite: 1]
              draw_measurements(&node_cache[selected_node]); //[cite: 1]
          }
      }

      // Авто-оновлення для HUB
      if (node_valid[selected_node] && node_cache[selected_node].id == 0) { //[cite: 1]
          if (HAL_GetTick() - last_log_time < 100) { //[cite: 1]
              draw_measurements(&node_cache[selected_node]); //[cite: 1]
          }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_14, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB2 PB12 PB13
                           PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == DIO0_Pin) {
        lora_rx_flag = 1;

    }

    if (GPIO_Pin == GPIO_PIN_3) {
		// Програмний антидребезг: ігноруємо натискання частіше ніж раз на 250 мс
		if ((HAL_GetTick() - last_button_press) > 250) {
			last_button_press = HAL_GetTick();
    	debug_log("STM32_CORE", "BTN CLICK");
            // шукаємо наступну валідну ноду по колу
            for (uint8_t i = 1; i <= MAX_NODES; i++) {
                uint8_t next = (selected_node + i) % MAX_NODES;
                if (node_valid[next]) {
                    selected_node = next;
                    node_switch_flag = 1;
                    break;
                }
            }
        }
    }
}

void Led_Blink_Count(uint8_t n, uint8_t delay){
	for(uint8_t i =0;i<n;i++){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_Delay(delay);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		HAL_Delay(delay);

	}
}

uint8_t crc8(uint8_t *data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc <<= 1;
        }
    }
    return crc;
}

NodeData read_lora_packet(uint8_t *buf)
{
	NodeData d = {0};

    // CRC CHECK
    if (crc8(buf, 8) != buf[8])
    {
    	debug_log("STM32_CORE", "CRC ERROR");
    	return d;
    }

    // PARSE PACKET

    // 0 byte - ID
    d.id = buf[0];

    // 1-2 byte - temperature
    int16_t temp = (int16_t)((buf[1] << 8) | buf[2]);
    d.temp = temp / 100.0f;

    // 3-4 byte - humidity
    uint16_t hum = (uint16_t)((buf[3] << 8) | buf[4]);
    d.humidity = hum / 100.0f;

    // 5-6 byte - Pressure
    uint16_t press = (uint16_t)((buf[5] << 8) | buf[6]);
    d.pressure = press;

	// 7 byte - Flags
    d.flags = buf[7];


    debug_log("STM32_CORE", "LoRa read success");
    return d;
}

void draw_measurements(const NodeData *d){

	char str[12];

	static int prevId = -999;

	char nodeName[12];

	if(d->id != prevId){
		prevId = d->id;

		if(d->id == 0){
			snprintf(nodeName, sizeof(nodeName), "%-10s", "HUB");
		}else if(d->id > 0){
			snprintf(nodeName, sizeof(nodeName), "NODE %-5d", d->id);
		}else{
			snprintf(nodeName, sizeof(nodeName), "%-10s", "UNKNOWN");
		}

		ST7735_DrawString(4, 20, nodeName,
		                  Font_11x18,
		                  ST7735_WHITE,
		                  ST7735_BLACK);
	}

	int16_t temp10 = (int16_t)(d->temp * 10 + 0.5f);
	static int prevTemp10 = -999;

	if(temp10 != prevTemp10){
		prevTemp10 = temp10;

		char sign = '+';
		int16_t value = temp10;

		if (temp10 < 0)
		{
		    sign = '-';
		    value = -temp10;
		}

		sprintf(str, "%c%d.%d", sign, value / 10, value % 10);
		ST7735_DrawString(14, 46, str, Font_16x26, ST7735_WHITE, ST7735_BLACK);
	}

	uint8_t humInt = (uint8_t)(d->humidity + 0.5f);
	static int16_t prevHumInt = -999;

	if(humInt != prevHumInt){
		prevHumInt = humInt;

		sprintf(str, "%-3u%%", humInt);
		ST7735_DrawString(9, 90, str, Font_11x18, ST7735_WHITE, ST7735_BLACK);
	}


	uint16_t pressureHpa = (uint16_t)(d->pressure + 0.5f);
	static uint16_t prevPressureHpa = 0;

	if(pressureHpa != prevPressureHpa){
		prevPressureHpa = pressureHpa;

		sprintf(str, "%u", pressureHpa);
		ST7735_DrawString(80, 90, str, Font_11x18, ST7735_WHITE, ST7735_BLACK);
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

#ifdef  USE_FULL_ASSERT
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
