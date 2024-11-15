#include "check_ymodem_send_agent.h"
#include "perf_counter.h"
#include "flash_blob.h"
#include "./logging/wl_log.h"
#include "usart.h"
#include <assert.h>
#include <string.h>

#undef this
#define this        (*ptThis)

__attribute__((aligned(32)))
static uint8_t s_chBuffer[1024] ;
static uint32_t count1 = 0,count2 = 0;
static bool ymodem_send_file_name(void *pObj,uint8_t *pchBuffer,uint16_t *phwSize)
{
	ymodem_send_t *(ptThis) = pObj;	
    assert(NULL != pObj);
	
	count1++;
	count2 = 0;
	*phwSize = 128;
	memset(s_chBuffer,0,sizeof(s_chBuffer));
	if(count1 > 2){
		count1 = 0;
		return true;
	}

	sprintf((char *)pchBuffer, "%d_%s%c%d", count1,"123.txt", '\0', 102400);

    return true;	
}

static bool ymodem_send_file_data(void *pObj,uint8_t *pchBuffer,uint16_t *phwSize)
{
	ymodem_send_t *(ptThis) = pObj;	
    assert(NULL != pObj);
	
	count2++;
    *phwSize = 1024;
	
	if(count2 == (102400/1024 + 1)){
	   *phwSize = 102400%1024;
	    memset(pchBuffer,count2 + 0X30,*phwSize);
		count2 = 0;
	}else{
	    memset(pchBuffer,count2 + 0X30,1024);
	}
    return true;	
}

static ymodem_read_stat_t ymodem_read_data_with_timeout(void *pObj,uint8_t* pchByte,uint16_t hwSize,uint16_t hwFrameTimeout,uint16_t hwBytesTimeout)
{ 
	ymodem_send_t *(ptThis) = pObj;	
    assert(NULL != pObj);
	#define YMODEM_SEND_DATA_RESET_FSM()    do{this.chState = 0;}while(0)	
    enum { START = 0, GET_BYTE,IS_TIMEOUT};
    switch(this.chState){
        case START:{
			this.hwIndex = 0;
			this.hwReadSize = hwSize;
            this.lTargetTime = perfc_convert_ticks_to_ms(get_system_ticks()) + hwFrameTimeout;
            this.chState = GET_BYTE;
        }
        case GET_BYTE:{
            uint16_t hwReadLen = dequeue(&this.tByteInQueue,(pchByte + this.hwIndex),this.hwReadSize);
            if(this.hwReadSize == hwReadLen){
                YMODEM_SEND_DATA_RESET_FSM();
                return YMODEM_READ_CPL;
            }else if(hwReadLen > 0){
				this.hwIndex += hwReadLen;
				this.hwReadSize = this.hwReadSize - hwReadLen; 
				this.lTargetTime = perfc_convert_ticks_to_ms(get_system_ticks()) + hwBytesTimeout;
			}
            this.chState = IS_TIMEOUT;
        }
        case IS_TIMEOUT:{
            if(perfc_convert_ticks_to_ms(get_system_ticks()) >= this.lTargetTime){
                YMODEM_SEND_DATA_RESET_FSM();
	            return YMODEM_READ_TIMEOUT;
            }
            this.chState = GET_BYTE;
            break; 
        }
    }    	
	return YMODEM_READ_ONGOING;
}

bool ymodem_write_data(void *pObj,uint8_t* pchByte,uint16_t hwSize)
{ 
	ymodem_send_t *(ptThis) = pObj;	
    assert(NULL != pObj);
	
    uart_sent_data(&huart3,pchByte,hwSize); 
	return true;
}


check_agent_ymodem_send_t *check_agent_ymodem_send_init(check_agent_ymodem_send_t *ptObj)
{
    check_agent_ymodem_send_t *(ptThis) = ptObj;
    assert(NULL != ptObj);
	
    this.tCheckAgent.pAgent = &this.tYmodemSend.parent;
    this.tCheckAgent.fnCheck = (check_hanlder_t *)ymodem_send;
    this.tCheckAgent.ptNext = NULL;
    
    this.tYmodemSend.pchQueueBuf = (char *)malloc(8);
	if(this.tYmodemSend.pchQueueBuf != NULL){
        queue_init(&this.tYmodemSend.tByteInQueue,this.tYmodemSend.pchQueueBuf,8);
	}
    ymodem_init(&this.tYmodemSend.parent, 
		 &this.tYmodemSend,
		 s_chBuffer,
	     sizeof(s_chBuffer),
         ymodem_send_file_name,
	     ymodem_send_file_data,
	     ymodem_read_data_with_timeout,
	     ymodem_write_data);
	
    return ptObj;
}

