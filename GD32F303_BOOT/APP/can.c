#include "can.h"
#include "gd32f30x_can.h"
#include "gd32f30x.h"
#include "bootloader.h"

CanID_t tCAN_SEND_ID = {
    .sig.HEAD = 0X14,
	.sig.PF = 0X1F,
	.sig.DA = 0XF4,
	.sig.SA = BOARD_ADDR,
};

CanID_t tCAN_RECEIVE_ID = {
    .sig.HEAD = 0X14,
	.sig.PF = 0X1F,
    .sig.DA = BOARD_ADDR,
	.sig.SA = 0XF4,
};

#define 	xCAN_ACR    (uint32_t)((tCAN_RECEIVE_ID.CanID<<3))
#define 	xCAN_MSK 	(uint32_t)((0xFF00FFFF<<3))


can_data_msg_t tCanMsgObj;

my_user_data_t MyUserData;

static void NVIC_Configuration(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2 );
    nvic_irq_enable(DEBUG_CANx_IRQ, 0, 0);
}

void CAN_TX_GPIO_Config(void)
{
  	rcu_periph_clock_enable(RCU_CAN0);
    rcu_periph_clock_enable(CAN_TX_GPIO_CLK);

    gpio_init(CAN_TX_GPIO_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, CAN_TX_GPIO_PIN);
    gpio_init(CAN_TX_GPIO_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, CAN_RX_GPIO_PIN);
	
}

/*!
    \brief      CAN工作参数配置
    \param[in]  none
    \param[out] none
    \retval     none
*/
void CAN_Configuration(uint32_t baurd)
{
    can_parameter_struct can_parameter;
    can_filter_parameter_struct can_filter;
    /* initialize CAN structures */
    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
    can_struct_para_init(CAN_FILTER_STRUCT, &can_filter);
    /* initialize CAN register */
    can_deinit(CAN0);
    
    /* initialize CAN */
    can_parameter.time_triggered = DISABLE;
    can_parameter.auto_bus_off_recovery = ENABLE;
    can_parameter.auto_wake_up = DISABLE;
    can_parameter.auto_retrans = ENABLE;
    can_parameter.rec_fifo_overwrite = DISABLE;
    can_parameter.trans_fifo_order = DISABLE;
    can_parameter.working_mode = CAN_NORMAL_MODE;
    can_parameter.resync_jump_width = CAN_BT_SJW_1TQ;
    can_parameter.time_segment_1 = CAN_BT_BS1_7TQ;
    can_parameter.time_segment_2 = CAN_BT_BS2_2TQ;
	if(baurd == 1000000){
		can_parameter.prescaler = 6;
	}else if(baurd == 250000){
        can_parameter.prescaler = 24;			
	}else {
        can_parameter.prescaler = 12;
	}
    can_init(CAN0, &can_parameter);

    /* initialize filter */
    /* CAN0 filter number */
    can_filter.filter_number = 0;

    /* initialize filter */    
    can_filter.filter_mode = CAN_FILTERMODE_MASK;
    can_filter.filter_bits = CAN_FILTERBITS_32BIT;
    can_filter.filter_list_high =  (uint16_t)((xCAN_ACR >> 16) & 0xffff);
    can_filter.filter_list_low = (uint16_t) (xCAN_ACR | CAN_FF_EXTENDED);
    can_filter.filter_mask_high =  (uint16_t) ((xCAN_MSK >> 16) & 0xffff);
    can_filter.filter_mask_low = (uint16_t) (xCAN_MSK | CAN_FF_EXTENDED);
    can_filter.filter_fifo_number = CAN_FIFO0;
    can_filter.filter_enable = ENABLE;
    can_filter_init(&can_filter);
}

void DEBUG_CANx_IRQHandler(void)
{
    can_receive_message_struct RxMessage;
    can_interrupt_disable(CAN0, CAN_INT_RFO0);
    can_interrupt_flag_clear(DEBUG_CANx, CAN_INT_FLAG_RFL0);
    can_message_receive(DEBUG_CANx, CAN_FIFO0, &RxMessage);
	
    if(RxMessage.rx_ff == 0)
        tCanMsgObj.CanID = RxMessage.rx_sfid;
    else tCanMsgObj.CanID = RxMessage.rx_efid;	

    for(uint8_t i = 0; i < RxMessage.rx_dlen; i++) {
        tCanMsgObj.CanDATA.B[i] = RxMessage.rx_data[i];
    }
    tCanMsgObj.CanDLC = RxMessage.rx_dlen;	

    emit(can_sig, &tCanMsgObj,
         args(
             tCanMsgObj.CanDATA.B,
             tCanMsgObj.CanDLC
         ));    
}

void can0_tx_data(uint32_t extid, uint8_t *str, uint8_t strlength)
{
    uint8_t mailbox_number;
    can_trasnmit_message_struct TxMessage;
    uint32_t i = 0;
    TxMessage.tx_efid = extid;
    TxMessage.tx_ff = CAN_FF_EXTENDED;
    TxMessage.tx_ft = CAN_FT_DATA;
    TxMessage.tx_dlen = strlength;

    for(i = 0; i < strlength; i++){
        TxMessage.tx_data[i] = *str++;
	}

	do {
        mailbox_number = can_message_transmit(DEBUG_CANx, &TxMessage);
    } while(mailbox_number == CAN_NOMAILBOX);
	
}

uint16_t can0_write_data(uint32_t wCanTxId, uint8_t *chDate, uint8_t len)
{
    int index = 0;
    while (index < len)
    {
        if (len - index < 8)
        {
           can0_tx_data(wCanTxId,(uint8_t *)(chDate + index), len - index);
           break;
        }
        else
        {
            can0_tx_data(wCanTxId,(uint8_t *)(chDate + index), 8);
            index += 8;
        }
    }	
    return len;
}
int can_drv_init(uint32_t baurd)
{
    CAN_TX_GPIO_Config();
    NVIC_Configuration();
    CAN_Configuration(baurd);
    can_interrupt_enable(CAN0, CAN_INTEN_RFNEIE0);
    return 0;
}

