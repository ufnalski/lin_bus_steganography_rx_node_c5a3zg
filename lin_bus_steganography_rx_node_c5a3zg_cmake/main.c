/**
 ******************************************************************************
 * file           : main.c
 * brief          : Main program body
 *                  Calls target system initialization then loop in main.
 ******************************************************************************
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "mx_gpio_default.h"
#include "mx_usart2.h"
#include "stm32c5xx_hal_gpio.h"
#include "stm32c5xx_hal_tim.h"
#include "stm32c5xx_hal_uart.h"

#include <stdio.h>
#include <string.h>

/* Private typedef -----------------------------------------------------------*/

typedef enum {
  PULSE_SHORT = 0,
  PULSE_LONG = 1,
  PULSE_ERROR,
} pulse_t;

/* Private define ------------------------------------------------------------*/

#define LIN_USART USART3

#define LIN_RX_BUFFER_SIZE 16

#define STEGO_PAYLOAD_SIZE 128

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

hal_uart_handle_t *huart3_lin;

uint8_t LinRxData[LIN_RX_BUFFER_SIZE];

volatile uint16_t cnt_frame_received;
volatile uint16_t cnt_break_detected;
volatile uint16_t cnt_input_captured;
volatile uint16_t break_length_fifo[2];
volatile uint16_t break_length;
volatile uint16_t break_length_missed;
volatile uint8_t lin_break_detected_flag = 0;

hal_tim_handle_t *tim1_lin_break_ic;

uint8_t stego_data[STEGO_PAYLOAD_SIZE];
uint8_t end_delimiter;
uint8_t potential_begin_found = 0;
uint8_t true_begin_found;

uint16_t byte_idx = 0;
uint8_t bit_idx = 0;

char hidden_msg[STEGO_PAYLOAD_SIZE];
uint16_t hidden_msg_size;

hal_uart_handle_t *vcp_uart;

/* Private functions prototype -----------------------------------------------*/

pulse_t calc_pulse(uint16_t _time);

/**
 * brief:  The application entry point.
 * retval: none but we specify int to comply with C99 standard
 */
int main(void) {
  /** System Init: this code placed in targets folder initializes your system.
   * It calls the initialization (and sets the initial configuration) of the
   * peripherals. You can use STM32CubeMX to generate and call this code or not
   * in this project. It also contains the HAL initialization and the initial
   * clock configuration.
   */
  if (mx_system_init() != SYSTEM_OK) {
    return (-1);
  } else {
    /*
     * You can start your application code here
     */

    huart3_lin = mx_usart3_uart_gethandle();
    HAL_UART_ReceiveToIdle_IT(huart3_lin, LinRxData, LIN_RX_BUFFER_SIZE);
    LIN_USART->CR2 |= USART_CR2_LBDIE; // LIN break detection interrupt enable
    HAL_GPIO_WritePin(LIN_EN_PORT, LIN_EN_PIN, HAL_GPIO_PIN_SET);

    tim1_lin_break_ic = mx_tim1_gethandle();
    HAL_TIM_Start_IT(tim1_lin_break_ic);
    HAL_TIM_IC_StartChannel_IT(tim1_lin_break_ic, HAL_TIM_CHANNEL_1);

    vcp_uart = mx_usart2_uart_gethandle();

    while (1) {
      __NOP();
    }
  }
} /* end main */

pulse_t calc_pulse(uint16_t _time) {
  if (_time < 6000)
    return PULSE_ERROR;
  else if (_time < 7100)
    return PULSE_SHORT;
  else if (_time < 8000)
    return PULSE_LONG;
  else
    return PULSE_ERROR;
}

void HAL_UART_LINBreakCallback(hal_uart_handle_t *huart) {
  if (huart->instance == HAL_UART3) {
    LIN_USART->ICR |= USART_ICR_LBDCF; // clear flag
    break_length = TIM1->CCR1;
    lin_break_detected_flag = 1;
    HAL_GPIO_WritePin(LIN_BREAK_DETECTED_PORT, LIN_BREAK_DETECTED_PIN,
                      HAL_GPIO_PIN_SET);
    HAL_GPIO_WritePin(LIN_BREAK_DETECTED_PORT, LIN_BREAK_DETECTED_PIN,
                      HAL_GPIO_PIN_RESET);
    LIN_USART->RQR |=
        USART_RQR_RXFRQ; // flush erroneous byte (coming from break field)
    HAL_UART_ReceiveToIdle_IT(huart3_lin, LinRxData, LIN_RX_BUFFER_SIZE);
    cnt_break_detected++;

    pulse_t _new_val = calc_pulse(break_length);

    if (potential_begin_found == 1) {
      if (_new_val == PULSE_LONG) {
        true_begin_found = 1;
        potential_begin_found = 0;
      }
    }

    if (true_begin_found && (byte_idx < STEGO_PAYLOAD_SIZE)) {
      if (_new_val == PULSE_LONG) {
        stego_data[byte_idx] |= (1 << bit_idx);
      }
      bit_idx++;
      if (bit_idx == 8) {
        bit_idx = 0;
        byte_idx++;
      }
    }

    if (_new_val == PULSE_SHORT) {
      end_delimiter++;
    } else {
      end_delimiter = 0;
    }
    if (end_delimiter == 16) {
      potential_begin_found = 1;
      true_begin_found = 0;
      end_delimiter = 0;
      byte_idx = 0;
      bit_idx = 0;
      hidden_msg_size = sprintf(hidden_msg, "\r\nLIN hidden message: %s\r\n",
                                (char *)(stego_data + 1));
      memset(stego_data, 0x00, STEGO_PAYLOAD_SIZE);
      HAL_UART_Transmit_DMA(vcp_uart, hidden_msg, hidden_msg_size);
    }
  }
}

void HAL_UART_RxCpltCallback(hal_uart_handle_t *huart, uint32_t size_byte,
                             hal_uart_rx_event_types_t rx_event) {
  if (huart->instance == HAL_UART3) {
    cnt_frame_received++;
  }
}

// just for debugging
void HAL_TIM_InputCaptureCallback(hal_tim_handle_t *htim,
                                  hal_tim_channel_t channel) {
  if ((htim->instance == HAL_TIM1) && (channel == HAL_TIM_CHANNEL_1)) {
    break_length_fifo[1] = break_length_fifo[0];
    break_length_fifo[0] = TIM1->CCR1;
    if (lin_break_detected_flag == 1) { // break detection interrupt fires after
                                        // the break delimiter (return to high)
      break_length_missed = break_length_fifo[1];
      lin_break_detected_flag = 0;
      cnt_input_captured++;
    }
  }
}
