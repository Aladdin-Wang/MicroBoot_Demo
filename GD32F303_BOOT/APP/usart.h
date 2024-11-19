/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "signals_slots.h"
/* USER CODE END Includes */

#define 	UART1_TX_EN() gpio_bit_write(GPIOA, GPIO_PIN_8, SET);

#define 	UART1_RX_EN() gpio_bit_write(GPIOA, GPIO_PIN_8, RESET);

								

#define EVAL_COM0                        USART0
#define EVAL_COM0_CLK                    RCU_USART0
#define EVAL_COM0_TX_PIN                 GPIO_PIN_9
#define EVAL_COM0_RX_PIN                 GPIO_PIN_10
#define EVAL_COM0_GPIO_PORT              GPIOA
#define EVAL_COM0_GPIO_CLK               RCU_GPIOA



/* USER CODE BEGIN Private defines */
typedef struct
{
    SIG_SLOT_OBJ;
    uint8_t Byte;
    uint16_t hwLen;
}uart_data_msg_t;
extern uart_data_msg_t  tUartMsgObj;

signals(uart_sig,uart_data_msg_t *ptThis,
      args(              
            uint8_t *pchByte,
            uint16_t hwLen
          ));
/* USER CODE END Private defines */

void uart1_init(uint32_t baurd);
/* USER CODE BEGIN Prototypes */
void uart_sent_data(uint32_t huart,uint8_t *chDate, uint16_t hwLen);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

