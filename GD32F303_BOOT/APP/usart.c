/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "gd32f30x.h"
#include "perf_counter.h"
/* USER CODE BEGIN 0 */
uart_data_msg_t tUartMsgObj;
/* USER CODE END 0 */

void uart1_init(uint32_t baurd)
{
    uint32_t com_id = 0U;
	
    /* enable GPIO clock */
    rcu_periph_clock_enable(EVAL_COM0_GPIO_CLK);

    /* enable USART clock */
    rcu_periph_clock_enable(EVAL_COM0_CLK);

    /* connect port to USARTx_Tx */
    gpio_init(EVAL_COM0_GPIO_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, EVAL_COM0_TX_PIN);

    /* connect port to USARTx_Rx */
    gpio_init(EVAL_COM0_GPIO_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, EVAL_COM0_RX_PIN);

    /* USART configure */
    usart_deinit(EVAL_COM0);
    usart_baudrate_set(EVAL_COM0, baurd);
    usart_word_length_set(EVAL_COM0, USART_WL_8BIT);
    usart_stop_bit_set(EVAL_COM0, USART_STB_1BIT);
    usart_parity_config(EVAL_COM0, USART_PM_NONE);
    usart_hardware_flow_rts_config(EVAL_COM0, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(EVAL_COM0, USART_CTS_DISABLE);
    usart_receive_config(EVAL_COM0, USART_RECEIVE_ENABLE);
    usart_transmit_config(EVAL_COM0, USART_TRANSMIT_ENABLE);
    nvic_irq_enable(USART0_IRQn, 0, 0);
    usart_interrupt_enable(EVAL_COM0, USART_INT_RBNE);
    usart_enable(EVAL_COM0);
}

/*!
    \brief      this function handles USART0 exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USART0_IRQHandler(void)
{
    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE)){
        /* read one byte from the receive data register */
       tUartMsgObj.Byte = (uint8_t)usart_data_receive(USART0);
       emit(uart_sig, &tUartMsgObj,
             args(
                 &tUartMsgObj.Byte,
                 1
             ));
			  /* Clear RXNE interrupt flag */
       usart_flag_clear(USART0, USART_FLAG_RBNE);
    }       
}

void uart_sent_data(uint32_t huart, uint8_t *chDate, uint16_t hwLen)
{
    UART1_TX_EN();
    for(uint16_t i = 0; i < hwLen; i++) {
        usart_data_transmit(EVAL_COM0, chDate[i]);
    }
		delay_us(100);
    UART1_RX_EN();
}


/* USER CODE END 1 */
