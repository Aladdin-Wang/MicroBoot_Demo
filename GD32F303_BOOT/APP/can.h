#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "signals_slots.h"

#define BOARD_ADDR  0XF0

/*CAN0_RX*/
#define CAN_RX_GPIO_PORT      GPIOA
#define CAN_RX_GPIO_CLK       RCU_GPIOA
#define CAN_RX_GPIO_PIN       GPIO_PIN_11

/*CAN0_TX*/
#define CAN_TX_GPIO_PORT      GPIOA
#define CAN_TX_GPIO_CLK       RCU_GPIOA
#define CAN_TX_GPIO_PIN       GPIO_PIN_12

/*CAN0*/
#define DEBUG_CANx  CAN0
#define DEBUG_CANx_CLK  RCU_CAN0

#define  DEBUG_CANx_IRQ  USBD_LP_CAN0_RX0_IRQn

#define  DEBUG_CANx_IRQHandler  USBD_LP_CAN0_RX0_IRQHandler

#define USER_DATA_SIZE            192
typedef struct {
    char chProjectName[16];
    char chHardWareVersion[16];
    char chSoftBootVersion[16];
    char chSoftAppVersion[16];
	  char chAppCompileDate[16];
    char chAppCompileTime[16];
	
	  char wBoardId;	
    char chPort1Name[16];
	  int wPort1Baudrate;
    char chPort2Name[16];
	  int wPort2Baudrate;		
    char chPort3Name[16];
	  int wPort3Baudrate;	

} mymsgSig_t;
typedef struct {
    union {
        mymsgSig_t sig;
        char B[USER_DATA_SIZE];
    } msg_data;
} my_user_data_t;
extern my_user_data_t MyUserData;

typedef struct {
    uint8_t SA: 8;
    uint8_t DA: 8;
    uint8_t PF: 8;
    uint8_t HEAD: 8;
} canID_msgType;
typedef union {
    canID_msgType sig;
    uint32_t CanID;
} CanID_t;

typedef struct {
    SIG_SLOT_OBJ;
    uint8_t  CanNO;
    uint8_t  CanDLC;
    uint32_t CanID;
    union {
        uint8_t B[8];
        uint16_t H[4];
        uint32_t W[2];
    } CanDATA;
} can_data_msg_t;
extern can_data_msg_t tCanMsgObj;

signals(can_sig, can_data_msg_t *ptThis,
        args(
            uint8_t *pchByte,
            uint16_t hwLen
        ));

int can_drv_init(uint32_t baurd);
void can0_tx_data(uint32_t extid, uint8_t *str, uint8_t strlength);
uint16_t can0_write_data(uint32_t wCanTxId, uint8_t *chDate, uint8_t len);

#endif