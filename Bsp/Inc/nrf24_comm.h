/**
  ******************************************************************************
  * @file    nrf24_comm.h
  * @brief   NRF24 Communication Module for JoyStick Remote Controller
  *          Provides TX functionality with status monitoring and buffer management
  ******************************************************************************
  */

#ifndef __NRF24_COMM_H
#define __NRF24_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "NRF24.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/
typedef enum {
    CONTROL_MODE_JOYSTICK,
    CONTROL_MODE_UPLEVEL_UART
} ControlMode_t;

/**
  * @brief  NRF24 Communication Status Structure
  */
typedef struct {
    bool initialized;           // NRF24 module initialized successfully
    bool communication_ok;      // Communication with NRF24 is working
    bool tx_success;           // Last transmission was successful
    bool tx_failed;            // Last transmission failed
    uint32_t tx_count;         // Total successful transmissions
    uint32_t tx_error_count;   // Total failed transmissions
    uint32_t last_tx_time;     // Last transmission timestamp
    ControlMode_t control_mode; // Current control mode
    uint32_t last_uart_time;   // Last time UART data was received
} NRF24_Status_t;

/**
  * @brief  TX Buffer Structure (10 bytes total)
  */
typedef struct {
    uint8_t robot_id;          // Robot ID
    uint16_t forward_vel;      // Forward velocity (float16)
    uint16_t left_vel;         // Left velocity (float16)
    uint16_t angular_vel;      // Angular velocity (float16)
    uint16_t dribbler;         // Dribbler state and speed
    uint8_t kickers;           // Upper and lower kicker power
} __attribute__((packed)) NRF24_TxBuffer_t;

/**
  * @brief  Physical velocity values from remapped ADC (in float32)
  */
typedef struct {
    float forward_vel;      // Forward velocity in m/s
    float left_vel;        // Left velocity in m/s
    float angular_vel;     // Angular velocity in rad/s
} PhysicalVelocity_t;

/* Exported constants --------------------------------------------------------*/
#define NRF24_TX_BUFFER_SIZE    10        // Pure data size
#define NRF24_CHANNEL           76        // Communication channel
#define NRF24_TX_ADDR_SIZE      5         // Address size
#define NRF24_PAYLOAD_SIZE      10        // Payload size

#define UART_BUFFER_SIZE        12        // Including frame markers
#define UART_TIMEOUT_MS         100       // Switch back to joystick if no UART data for 100ms
#define FRAME_HEAD              0xAA
#define FRAME_END              0x55

// Robot ID range
extern volatile uint8_t MAX_ROBOT_NUMBER;  // Maximum robot ID (0 to MAX_ROBOT_NUMBER-1)

// Physical limits for velocities
#define MAX_FORWARD_VEL 2.0f    // Maximum forward velocity in m/s
#define MAX_LEFT_VEL 2.0f       // Maximum left velocity in m/s  
#define MAX_ANGULAR_VEL 4.0f    // Maximum angular velocity in rad/s

// ADC center and range values
#define ADC_CENTER 2048
#define ADC_MAX 4095
#define ADC_MIN 0

// Float16 conversion constants
#define FLOAT16_SIGN_MASK    0x8000
#define FLOAT16_EXP_MASK     0x7C00
#define FLOAT16_FRAC_MASK    0x03FF
#define FLOAT16_EXP_BIAS     15

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  Initialize NRF24 communication module
  * @retval bool: true if initialization successful, false otherwise
  */
bool NRF24_Comm_Init(void);

/**
  * @brief  Update TX buffer with new ADC and AUX data
  * @param  adc_values: Array of 4 ADC values
  * @param  aux1: AUX1 button state
  * @param  aux2: AUX2 button state
  * @retval None
  */
void NRF24_Comm_UpdateTxBuffer(uint16_t adc_values[4], uint8_t aux1, uint8_t aux2);

/**
  * @brief  Update TX buffer from UART data
  * @param  uart_data: Pointer to UART data
  * @retval None
  */
void NRF24_Comm_UpdateFromUART(uint8_t* uart_data);

/**
  * @brief  Transmit current TX buffer data
  * @retval bool: true if transmission successful, false otherwise
  */
bool NRF24_Comm_Transmit(void);

/**
  * @brief  Get current communication status
  * @retval NRF24_Status_t: Current status structure
  */
NRF24_Status_t NRF24_Comm_GetStatus(void);

/**
  * @brief  Get current TX buffer data for monitoring
  * @retval NRF24_TxBuffer_t: Current TX buffer structure
  */
NRF24_TxBuffer_t NRF24_Comm_GetTxBuffer(void);

/**
  * @brief  Play status feedback via buzzer (call from main loop)
  * @retval None
  */
void NRF24_Comm_StatusFeedback(void);

/**
  * @brief  IRQ interrupt handler (call from EXTI interrupt)
  * @retval None
  */
void NRF24_Comm_IRQ_Handler(void);

/**
  * @brief  Check UART timeout
  * @retval None
  */
void NRF24_Comm_CheckUARTTimeout(void);

/**
  * @brief  Update robot ID
  * @param  increment: True to increment, false to decrement
  * @retval None
  */
void NRF24_Comm_UpdateRobotID(bool increment);

/**
  * @brief  Check if local control can write
  * @retval bool: true if local control can write, false otherwise
  */
bool NRF24_Comm_CanLocalControlWrite(void);

PhysicalVelocity_t NRF24_Comm_GetPhysicalVelocity(void);

extern NRF24_TxBuffer_t tx_buffer;
extern NRF24_Status_t nrf24_status;

#ifdef __cplusplus
}
#endif

#endif /* __NRF24_COMM_H */ 