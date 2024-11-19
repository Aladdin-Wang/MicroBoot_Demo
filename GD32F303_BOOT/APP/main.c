/*!
    \file    main.c
    \brief   USART echo interrupt

    \version 2021-03-23, V2.0.0, demo for GD32F30x
*/

/*
    Copyright (c) 2021, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "gd32f30x.h"
#include "gd32f303c_eval.h"
#include "perf_counter.h"
#include "systick.h"
#include "usart.h"
#include "can.h"
#include <stdio.h>

#include "check_agent_engine.h"
#include "micro_shell.h"
#include "ymodem_ota.h"

__attribute__((aligned(32)))
uint8_t s_chBuffer[2048 + 128] ;
static byte_queue_t                  s_tCheckUsePeekQueue;
static fsm(check_use_peek)           s_fsmCheckUsePeek;
static ymodem_ota_recive_t           s_tYmodemOtaReceive;
static check_shell_t                 s_tShellObj;

void board_gpio_init(void);

int reboot()
{
    SCB->AIRCR = 0X05FA0000 | (uint32_t)0x04;
    return 0;
}
MSH_CMD_EXPORT(reboot, reboot)

int64_t get_system_time_ms(void)
{
    return perfc_convert_ticks_to_ms(get_system_ticks());
}

uint16_t shell_read_data(wl_shell_t *ptObj, char *pchBuffer, uint16_t hwSize)
{
    peek_byte_t *ptReadByte = get_read_byte_interface(&s_fsmCheckUsePeek);
    return ptReadByte->fnGetByte(ptReadByte, (uint8_t *)pchBuffer, hwSize);
}

uint16_t shell_write_data(wl_shell_t *ptObj, const char *pchBuffer, uint16_t hwSize)
{
    can0_write_data((0X18AFF400 | BOARD_ADDR),(uint8_t *)pchBuffer,hwSize);
	return hwSize;
}

void updata_user_data()
{
    memcpy(MyUserData.msg_data.B, tUserData.msg_data.B, sizeof(MyUserData));
}

int main(void)
{
    /* configure systick */
    systick_config();
    init_cycle_counter(true);
    updata_user_data();
    board_gpio_init();
    nvic_irq_enable(USART0_IRQn, 0, 0);
	can_drv_init(MyUserData.msg_data.sig.wPort1Baudrate);
    queue_init(&s_tCheckUsePeekQueue, s_chBuffer, sizeof(s_chBuffer));
    init_fsm(check_use_peek, &s_fsmCheckUsePeek, args(&s_tCheckUsePeekQueue));

    ymodem_ota_receive_init(&s_tYmodemOtaReceive, get_read_byte_interface(&s_fsmCheckUsePeek));
    agent_register(&s_fsmCheckUsePeek, &s_tYmodemOtaReceive.tCheckAgent);

    connect(&tCanMsgObj, SIGNAL(can_sig), &s_tCheckUsePeekQueue, SLOT(enqueue_bytes));
    connect(&s_tYmodemOtaReceive.tYmodemReceive, SIGNAL(ymodem_rec_sig), (void *)(0X141FF400 | BOARD_ADDR), SLOT(can0_write_data));

    shell_ops_t s_tOps = {
        .fnReadData = shell_read_data,
		.fnWriteData = shell_write_data,
    };

    shell_init(&s_tShellObj, &s_tOps);
    agent_register(&s_fsmCheckUsePeek, &s_tShellObj.tCheckAgent);

    while(1) {
        call_fsm( check_use_peek,  &s_fsmCheckUsePeek );
    }
}

int stdout_putchar(unsigned char chByte)
{
    can0_tx_data(0X18AFF400 | BOARD_ADDR, (uint8_t *)(&chByte), 1);
    return 0;
}

void board_gpio_init(void)
{
	/*
    rcu_periph_clock_enable(RCU_GPIOA);
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
    GPIO_BC(GPIOA) = GPIO_PIN_8;

    rcu_periph_clock_enable(RCU_GPIOA);
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_3);
    GPIO_BOP(GPIOA) = GPIO_PIN_3;

    rcu_periph_clock_enable(RCU_GPIOB);
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_15);
    GPIO_BOP(GPIOB) = GPIO_PIN_15;
	*/
}


