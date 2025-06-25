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
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "nrf24_comm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// External declarations
extern NRF24_Status_t nrf24_status;

// Local variables
uint16_t adc_raw_buffer[4];
uint8_t AUX1=0;
uint8_t AUX2=0;
uint8_t AUX3=0;  // Added AUX3
uint8_t uart_buffer[12];  // Single buffer for UART communication

// Calibration variables
uint16_t adc_center[4];    // Center values for each channel
int16_t adc_calib_error[4];      // Calibration error between raw ADC and center values
uint16_t adc_min[4];       // Minimum ADC values for each channel
uint16_t adc_max[4];       // Maximum ADC values for each channel

// Filtering variables
float alpha = 0.1;         // Filter coefficient (0-1), lower = more smoothing
uint16_t adc_filtered[4];  // Filtered ADC values

// Remapping constants
#define CENTER_REGION 30   // Center region size (30 around center)
#define ADC_MAX 4095       // Maximum ADC value
#define ADC_MIN 0          // Minimum ADC value

// Buzzer control
#define BUZZER_PIN GPIO_PIN_12
#define BUZZER_PORT GPIOA

Beep_Control_t beep_control = {0};

// Function to start a non-blocking beep
static void start_beep(uint32_t duration_ms) {
    beep_control.start_time = HAL_GetTick();
    beep_control.duration = duration_ms;
    beep_control.is_beeping = true;
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
}

// Function to update beep state (call in main loop)
static void update_beep(void) {
    if (beep_control.is_beeping) {
        uint32_t current_time = HAL_GetTick();
        if (current_time - beep_control.start_time >= beep_control.duration) {
            beep_control.is_beeping = false;
            HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
        }
    }
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t remap_adc_value(uint16_t raw_value, uint16_t center, uint16_t min_val, uint16_t max_val)
{
    // Ensure input is within bounds
    if (raw_value > ADC_MAX) raw_value = ADC_MAX;
    if (raw_value < ADC_MIN) raw_value = ADC_MIN;
    
    // If value is in center region, keep it unchanged
    if(abs((int16_t)raw_value - (int16_t)center) <= CENTER_REGION) {
        return center;
    }
    
    // Calculate mapping for lower region (min_val to center)
    if(raw_value < center) {
        // Ensure we don't divide by zero
        if (min_val >= center) return center;
        float ratio = (float)(center - ADC_MIN) / (float)(center - min_val);
        int32_t result = center - (int32_t)((center - raw_value) * ratio);
        // Ensure result stays within bounds
        if (result < ADC_MIN) result = ADC_MIN;
        if (result > ADC_MAX) result = ADC_MAX;
        return (uint16_t)result;
    }
    // Calculate mapping for upper region (center to max_val)
    else {
        // Ensure we don't divide by zero
        if (max_val <= center) return center;
        float ratio = (float)(ADC_MAX - center) / (float)(max_val - center);
        int32_t result = center + (int32_t)((raw_value - center) * ratio);
        // Ensure result stays within bounds
        if (result < ADC_MIN) result = ADC_MIN;
        if (result > ADC_MAX) result = ADC_MAX;
        return (uint16_t)result;
    }
}

void filter_adc_values(void)
{
    for(int i = 0; i < 4; i++) {
        // Calculate new filtered value with float precision
        float new_value = alpha * (float)adc_raw_buffer[i] + (1.0f - alpha) * (float)adc_filtered[i];
        
        // Ensure the filtered value stays within bounds
        if (new_value > ADC_MAX) new_value = ADC_MAX;
        if (new_value < ADC_MIN) new_value = ADC_MIN;
        
        // Update the filtered value
        adc_filtered[i] = (uint16_t)new_value;
    }
}

/*
void pack_and_transmit_uart(void)
{
    // Apply low-pass filtering
    filter_adc_values();
    
    // Process ADC values directly into tx_buffer
    tx_buffer.forward_vel = remap_adc_value(adc_filtered[2], adc_center[2], adc_min[2], adc_max[2]);
    tx_buffer.left_vel = remap_adc_value(adc_filtered[3], adc_center[3], adc_min[3], adc_max[3]);
    tx_buffer.angular_vel = remap_adc_value(adc_filtered[0], adc_center[0], adc_min[0], adc_max[0]);
    
    // Process dribbler
    uint16_t dribbler_val = remap_adc_value(adc_filtered[1], adc_center[1], adc_min[1], adc_max[1]);
    if (dribbler_val > 2048) {
        tx_buffer.dribbler = 0xC000 | (dribbler_val & 0x3FFF);
    } else {
        tx_buffer.dribbler = 0x0000;
    }
    
    // Update kicker states
    tx_buffer.kickers = (AUX1 ? 0xF0 : 0x00) | (AUX2 ? 0x0F : 0x00);
    
    // Pack into UART buffer with frame markers
    uart_buffer[0] = FRAME_HEAD;
    memcpy(uart_buffer + 1, &tx_buffer, NRF24_TX_BUFFER_SIZE);
    uart_buffer[11] = FRAME_END;
    
    // Transmit via UART
    while(huart1.gState != HAL_UART_STATE_READY);
    HAL_UART_Transmit(&huart1, uart_buffer, UART_BUFFER_SIZE, 100);
}
*/

void buzzer_beep(uint32_t duration_ms)
{
  HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
  HAL_Delay(duration_ms);
  HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
}

void double_beep(void)
{
  buzzer_beep(100);
  HAL_Delay(100);
  buzzer_beep(100);
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
    SystemClock_Config();
    
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_USART1_UART_Init();
    MX_I2C1_Init();
    MX_SPI2_Init();
    
    /* USER CODE BEGIN 2 */
    // Initialize buzzer
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

    // Start ADC DMA
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw_buffer, 4);
    HAL_Delay(100);  // Wait for ADC to stabilize

    // Initialize NRF24
    bool nrf24_init_success = NRF24_Comm_Init();
    (void)nrf24_init_success;
    HAL_Delay(500);

    // Wait until AUX3 (PC14) is pressed (input is LOW)
    while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) != GPIO_PIN_RESET) {
    }
    
    // Turn off LED
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    
    // Now do calibration using raw ADC values
    adc_center[0] = 2048; // CH1
    adc_center[1] = 0;    // CH2
    adc_center[2] = 2048; // CH3
    adc_center[3] = 2048; // CH4
    
    // Set min/max values for each channel
    adc_min[0] = 68;    // CH1 min
    adc_max[0] = 4098;  // CH1 max
    adc_min[1] = 0;     // CH2 min
    adc_max[1] = 4027;  // CH2 max
    adc_min[2] = 88;    // CH3 min
    adc_max[2] = 4119;  // CH3 max
    adc_min[3] = 114;   // CH4 min
    adc_max[3] = 4147;  // CH4 max
    
    // Calculate errors between raw ADC values and center values
    for(int i = 0; i < 4; i++) {
        adc_calib_error[i] = (int16_t)adc_raw_buffer[i] - (int16_t)adc_center[i];
    }
    
    // Initialize filtered values with current raw values
    for(int i = 0; i < 4; i++) {
        adc_filtered[i] = adc_raw_buffer[i];
    }
    
    // Double beep to indicate calibration complete
    double_beep();

    // NOW start UART reception after calibration
    HAL_UART_Receive_IT(&huart1, uart_buffer, UART_BUFFER_SIZE);
    
    while (1)
    {
        // Update beep state
        update_beep();
        
        // Check for UART timeout
        // if (nrf24_status.control_mode == CONTROL_MODE_UPLEVEL_UART) {
        //     uint32_t current_time = HAL_GetTick();
        //     if (current_time - nrf24_status.last_uart_time > UART_TIMEOUT_MS) {
        //         // Switch back to joystick control if no UART data received
        //         nrf24_status.control_mode = CONTROL_MODE_JOYSTICK;
        //     }
        // }

        // if (NRF24_Comm_CanLocalControlWrite()) {
        //     filter_adc_values();
            
        //     uint16_t adc_processed_values[4];
        //     for(int i = 0; i < 4; i++) {
        //         // Use signed arithmetic for error compensation
        //         int32_t value = (int32_t)adc_filtered[i] - (int32_t)adc_calib_error[i];
                
        //         // Bound check the result
        //         if (value < ADC_MIN) value = ADC_MIN;
        //         if (value > ADC_MAX) value = ADC_MAX;
                
        //         // Convert back to uint16_t after bounds checking
        //         uint16_t compensated = (uint16_t)value;
                
        //         // Now do the remapping
        //         adc_processed_values[i] = remap_adc_value(compensated, adc_center[i], adc_min[i], adc_max[i]);
        //     }
            
        //     NRF24_Comm_UpdateTxBuffer(adc_processed_values, AUX1, AUX2);
        // }
        
        NRF24_Comm_Transmit();
        NRF24_Comm_StatusFeedback();
    }
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
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
  hadc1.Init.NbrOfConversion = 4;
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
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
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
  hi2c1.Init.ClockSpeed = 200000;
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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_2;
  huart1.Init.Parity = UART_PARITY_EVEN;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CE_Pin|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY5_Pin KEY6_Pin */
  GPIO_InitStruct.Pin = KEY5_Pin|KEY6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY7_Pin KEY8_Pin */
  GPIO_InitStruct.Pin = KEY7_Pin|KEY8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY2_Pin */
  GPIO_InitStruct.Pin = KEY2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CSN_Pin */
  GPIO_InitStruct.Pin = CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CSN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IRQ_Pin */
  GPIO_InitStruct.Pin = IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CE_Pin */
  GPIO_InitStruct.Pin = CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY1_Pin */
  GPIO_InitStruct.Pin = KEY1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t last_toggle_time_aux1 = 0;
    static uint32_t last_toggle_time_aux2 = 0;
    static uint32_t last_toggle_time_robotid = 0;
    uint32_t current_time = HAL_GetTick();
    
    // Handle Robot ID buttons (PB0 and PB1)
    if(GPIO_Pin == GPIO_PIN_0 || GPIO_Pin == GPIO_PIN_1) {
        if(current_time - last_toggle_time_robotid >= 100) {
            // PB0 is decrement, PB1 is increment
            if(GPIO_Pin == GPIO_PIN_0) {
                NRF24_Comm_UpdateRobotID(false);  // Decrement
            } else {
                NRF24_Comm_UpdateRobotID(true);   // Increment
            }
            last_toggle_time_robotid = current_time;
            start_beep(10);  // Short beep for robot ID change
        }
    }
    
    // Only process AUX buttons if in joystick mode
    if (NRF24_Comm_CanLocalControlWrite()) {
        // Handle AUX1 (PA15)
        if(GPIO_Pin == GPIO_PIN_15) {
            if(current_time - last_toggle_time_aux1 >= 100) {
                // Only toggle if button is actually pressed (LOW)
                if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET) {
                    AUX1 ^= 0x01;
                    last_toggle_time_aux1 = current_time;
                    start_beep(5);  // Very short beep for AUX1
                }
            }
        }
        
        // Handle AUX2 (PB11)
        if(GPIO_Pin == GPIO_PIN_11) {
            if(current_time - last_toggle_time_aux2 >= 100) {
                // Only toggle if button is actually pressed (LOW)
                if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET) {
                    AUX2 ^= 0x01;
                    last_toggle_time_aux2 = current_time;
                    start_beep(5);  // Very short beep for AUX2
                }
            }
        }
    }
    
    // Handle NRF24 IRQ interrupt (PA8)
    if(GPIO_Pin == IRQ_Pin) {
        NRF24_Comm_IRQ_Handler();
    }
}

// Add UART reception callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        // Process received UART data
        NRF24_Comm_UpdateFromUART(uart_buffer);
        
        // Restart UART reception
        HAL_UART_Receive_IT(&huart1, uart_buffer, UART_BUFFER_SIZE);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        // On UART error, return control to joystick
        nrf24_status.control_mode = CONTROL_MODE_JOYSTICK;
        
        // Restart UART reception
        HAL_UART_Receive_IT(&huart1, uart_buffer, UART_BUFFER_SIZE);
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
