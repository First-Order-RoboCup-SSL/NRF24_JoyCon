/**
  ******************************************************************************
  * @file    nrf24_comm.c
  * @brief   NRF24 Communication Module Implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "nrf24_comm.h"
#include "NRF24.h"
#include "NRF24_reg_addresses.h"
#include <string.h>

/* Private define ------------------------------------------------------------*/
#define BUZZER_PIN           GPIO_PIN_12
#define BUZZER_PORT          GPIOA
#define STATUS_FEEDBACK_INTERVAL    5000    // 5 seconds

/* Private variables ---------------------------------------------------------*/
NRF24_Status_t nrf24_status = {0};
NRF24_TxBuffer_t tx_buffer = {0};
static uint8_t tx_addr[NRF24_TX_ADDR_SIZE] = {'1', 'N', 'o', 'd', 'e'};
static uint32_t last_feedback_time = 0;
static bool status_feedback_pending = false;
static uint8_t current_robot_id = 0;

// Default maximum robot number (can be changed externally)
volatile uint8_t MAX_ROBOT_NUMBER = 16;

/* Private function prototypes -----------------------------------------------*/
static void NRF24_Comm_BuzzerBeep(uint32_t duration_ms);
static void NRF24_Comm_BuzzerDoubleBeep(void);
static bool NRF24_Comm_TestCommunication(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Generate a buzzer beep
  * @param  duration_ms: Duration in milliseconds
  * @retval None
  */
static void NRF24_Comm_BuzzerBeep(uint32_t duration_ms)
{
  HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
  HAL_Delay(duration_ms);
  HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
}

/**
  * @brief  Generate a double beep
  * @retval None
  */
static void NRF24_Comm_BuzzerDoubleBeep(void)
{
  NRF24_Comm_BuzzerBeep(100);
  HAL_Delay(100);
  NRF24_Comm_BuzzerBeep(100);
}

/**
  * @brief  Test communication with NRF24 module
  * @retval bool: true if communication is working
  */
static bool NRF24_Comm_TestCommunication(void)
{
  // Write a test value to a non-critical register and read it back
  uint8_t test_channel = 75;
  nrf24_set_channel(test_channel);
  HAL_Delay(1);
  
  uint8_t read_channel = nrf24_r_reg(RF_CH, 1);
  
  // Restore original channel
  nrf24_set_channel(NRF24_CHANNEL);
  
  return (read_channel == test_channel);
}

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Initialize NRF24 communication module
  * @retval bool: true if initialization successful
  */
bool NRF24_Comm_Init(void)
{
    // Reset status
    memset(&nrf24_status, 0, sizeof(nrf24_status));
    nrf24_status.control_mode = CONTROL_MODE_JOYSTICK;  // Start in joystick mode
    nrf24_status.last_uart_time = HAL_GetTick();       // Initialize timestamp
    
    // Initialize NRF24 pins
    csn_high();
    ce_high();
    HAL_Delay(5);
    ce_low();
    
    // Initialize NRF24 module
    nrf24_init();
    nrf24_stop_listen();
    
    // Configure NRF24 settings
    nrf24_auto_ack_all(auto_ack);
    nrf24_en_ack_pld(enable);
    nrf24_dpl(enable);
    nrf24_set_crc(en_crc, _2byte);
    nrf24_tx_pwr(n12dbm);                    // Maximum power
    nrf24_data_rate(_1mbps);                // 1Mbps data rate
    nrf24_set_channel(NRF24_CHANNEL);       // Channel 90
    nrf24_set_addr_width(NRF24_TX_ADDR_SIZE);
    
    // Disable dynamic payload for all pipes
    for(int i = 0; i < 6; i++) {
        nrf24_set_rx_dpl(i, enable);
    }
    
    // Set payload size
    nrf24_pipe_pld_size(0, NRF24_PAYLOAD_SIZE);
    
    // Set auto retransmission
    nrf24_auto_retr_delay(4);               // 1.25ms delay
    nrf24_auto_retr_limit(10);              // 10 retries
    
    // Open TX pipe
    nrf24_open_tx_pipe(tx_addr);
    nrf24_open_rx_pipe(0, tx_addr);
    
    ce_high();
    
    // Test communication
    HAL_Delay(10);
    bool comm_test = NRF24_Comm_TestCommunication();
    
    nrf24_status.initialized = true;
    nrf24_status.communication_ok = comm_test;
    
    // Provide audio feedback
    if (comm_test) {
        // Two quick beeps = successful initialization
        NRF24_Comm_BuzzerDoubleBeep();
        status_feedback_pending = false;
    } else {
        // Long beep = initialization failed
        NRF24_Comm_BuzzerBeep(500);
        status_feedback_pending = true;
    }
    
    last_feedback_time = HAL_GetTick();
    
    return comm_test;
}

bool NRF24_Comm_CanLocalControlWrite(void)
{
    return (nrf24_status.control_mode == CONTROL_MODE_JOYSTICK);
}

/**
  * @brief  Update TX buffer with new data
  * @param  adc_values: Array of 4 ADC values
  * @param  aux1: AUX1 button state
  * @param  aux2: AUX2 button state
  * @retval None
  */
void NRF24_Comm_UpdateTxBuffer(uint16_t adc_values[4], uint8_t aux1, uint8_t aux2)
{
    // Only update if in joystick control mode
    if (!NRF24_Comm_CanLocalControlWrite()) {
        return;
    }
    
    // Map joystick channels to velocities
    tx_buffer.angular_vel = adc_values[0];  // Channel 0 for rotation
    tx_buffer.forward_vel = adc_values[2];  // Channel 2 for forward/backward
    tx_buffer.left_vel = adc_values[3];     // Channel 3 for left/right
    
    // Map Channel 1 to dribbler (bits 15:14 for state, 13:0 for speed)
    uint16_t dribbler_val = adc_values[1];
    if (dribbler_val > 2048) {  // Above center means active
        tx_buffer.dribbler = 0xC000 | (dribbler_val & 0x3FFF);  // Set state bits to 11
    } else {
        tx_buffer.dribbler = 0x0000;  // Dribbler off
    }
    
    // Map AUX buttons to kicker values
    tx_buffer.kickers = (aux1 ? 0xF0 : 0x00) | (aux2 ? 0x0F : 0x00);
}

/**
  * @brief  Update from UART
  * @param  uart_data: Pointer to UART data
  * @retval None
  */
void NRF24_Comm_UpdateFromUART(uint8_t* uart_data)
{
    // Verify frame markers
    if (uart_data[0] != FRAME_HEAD || uart_data[11] != FRAME_END) {
        return;
    }
    
    // Switch to UART control mode and update timestamp
    nrf24_status.control_mode = CONTROL_MODE_UPLEVEL_UART;
    nrf24_status.last_uart_time = HAL_GetTick();
    
    // Copy data from UART buffer to TX buffer (skipping frame markers)
    memcpy(&tx_buffer, uart_data + 1, NRF24_TX_BUFFER_SIZE);
}

/**
  * @brief  Update robot ID
  * @param  increment: True to increment, False to decrement
  * @retval None
  */
void NRF24_Comm_UpdateRobotID(bool increment)
{
    if (increment) {
        current_robot_id = (current_robot_id + 1) % MAX_ROBOT_NUMBER;
    } else {
        // For decrement, handle underflow by wrapping around
        if (current_robot_id == 0) {
            current_robot_id = MAX_ROBOT_NUMBER - 1;
        } else {
            current_robot_id--;
        }
    }
    
    tx_buffer.robot_id = current_robot_id;
}

/**
  * @brief  Transmit current TX buffer data
  * @retval bool: true if transmission successful
  */
bool NRF24_Comm_Transmit(void)
{
  if (!nrf24_status.initialized) {
    return false;
  }
  
  // Attempt transmission
  uint8_t result = nrf24_transmit((uint8_t*)&tx_buffer, NRF24_PAYLOAD_SIZE);
  
  // Update status
  nrf24_status.last_tx_time = HAL_GetTick();
  
  if (result == 0) {
    // Success
    nrf24_status.tx_success = true;
    nrf24_status.tx_failed = false;
    nrf24_status.tx_count++;
    nrf24_status.communication_ok = true;
    status_feedback_pending = false;
    return true;
  } else {
    // Failed
    nrf24_status.tx_success = false;
    nrf24_status.tx_failed = true;
    nrf24_status.tx_error_count++;
    nrf24_status.communication_ok = false;
    status_feedback_pending = true;
    return false;
  }
}

/**
  * @brief  Get current communication status
  * @retval NRF24_Status_t: Current status
  */
NRF24_Status_t NRF24_Comm_GetStatus(void)
{
  return nrf24_status;
}

/**
  * @brief  Get current TX buffer for monitoring
  * @retval NRF24_TxBuffer_t: Current TX buffer
  */
NRF24_TxBuffer_t NRF24_Comm_GetTxBuffer(void)
{
  return tx_buffer;
}

/**
  * @brief  Provide status feedback via buzzer
  * @retval None
  */
void NRF24_Comm_StatusFeedback(void)
{
  uint32_t current_time = HAL_GetTick();
  
  // Check if it's time for periodic feedback
  if (status_feedback_pending && 
      (current_time - last_feedback_time) > STATUS_FEEDBACK_INTERVAL) {
    
    if (!nrf24_status.communication_ok) {
      // Three short beeps = communication problem
      for(int i = 0; i < 3; i++) {
        NRF24_Comm_BuzzerBeep(50);
        HAL_Delay(100);
      }
    }
    
    last_feedback_time = current_time;
    
    // If communication is working, disable pending feedback
    if (nrf24_status.communication_ok) {
      status_feedback_pending = false;
    }
  }
}

/**
  * @brief  IRQ interrupt handler
  * @retval None
  */
void NRF24_Comm_IRQ_Handler(void)
{
  // Handle NRF24 interrupt
  uint8_t status = nrf24_r_status();
  
  // Clear interrupt flags
  if (status & (1 << TX_DS)) {
    nrf24_clear_tx_ds();
  }
  
  if (status & (1 << MAX_RT)) {
    nrf24_clear_max_rt();
    nrf24_flush_tx();
  }
  
  if (status & (1 << RX_DR)) {
    nrf24_clear_rx_dr();
  }
} 