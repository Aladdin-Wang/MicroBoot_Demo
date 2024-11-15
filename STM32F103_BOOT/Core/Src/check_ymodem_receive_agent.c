#include "check_ymodem_receive_agent.h"
#include "perf_counter.h"
#include "wl_serve.h"
#include "can.h"
#include <assert.h>

#undef this
#define this        (*ptThis)

__attribute__((aligned(32)))
static uint8_t s_chBuffer[1024] ;

__attribute__((aligned(32)))
static char s_chQueueBuffer[2048] ;

extern uint8_t chUserData[];
extern void start_to_boot(void);
extern void end_to_boot(void);

static bool ymodem_recv_file_name(ymodem_t *ptObj, uint8_t *pchBuffer, uint16_t *phwSize)
{
    ymodem_receive_t *(ptThis) = (ymodem_receive_t *)ptObj;
    assert(NULL != ptObj);

    this.wOffSet = 0;
    this.pchFileName = (char *)&pchBuffer[0];
    this.pchFileSize = (char *)&pchBuffer[strlen(this.pchFileName) + 1];
    this.wFileSize = atol(this.pchFileSize);

    if(this.wFileSize <= 0) {
        *phwSize = 0;
        return true;
    }

    printf("Ymodem file_name:%s \r\n", this.pchFileName);
    printf("Ymodem file_size:%d \r\n", this.wFileSize);
    user_data_t *ptData = (user_data_t*)chUserData;

    if(strlen(ptData->msg_data.sig.chProjectName) > 0) {
        if(strcmp(ptData->msg_data.sig.chProjectName, this.pchFileName) != 0 ) {
            printf("Firmware Name Check failure.");
            return false;
        }
    }

    if( APP_PART_SIZE  < this.wFileSize) {
        printf("file size outrange flash size. \r\n");
        return false;
    }

    if(target_flash_init(APP_PART_ADDR) == false) {
        printf("target flash uninit. \r\n");
        return false;
    }

    uint32_t wEraseSize = target_flash_erase(APP_PART_ADDR, this.wFileSize);


    if( wEraseSize < this.wFileSize) {
        printf("target flash erase error. \r\n");
        return false;
    }

    printf("flash erase success:%d \r\n", wEraseSize);

    start_to_boot();
    return true;
}

static bool ymodem_recv_file_data(ymodem_t *ptObj, uint8_t *pchBuffer, uint16_t *phwSize)
{
    ymodem_receive_t *(ptThis) = (ymodem_receive_t *)ptObj;
    assert(NULL != ptObj);

    uint32_t  wRemainLen = this.wFileSize - this.wOffSet;
    uint32_t wWriteLen = *phwSize;

    if(wWriteLen > wRemainLen) {
        wWriteLen = wRemainLen;
    }
	
    uint16_t wWriteSize = target_flash_write(APP_PART_ADDR + this.wOffSet, pchBuffer, wWriteLen);


    if( wWriteSize != wWriteLen) {
        printf("target flash write data error. 0x%x.\r\n", wWriteSize);
        return false;
    }

    uint16_t hwWriteCheck = ymodem_crc16(pchBuffer, wWriteLen);
    uint16_t wReadSize = target_flash_read(APP_PART_ADDR + this.wOffSet, pchBuffer, wWriteLen);

    if( wReadSize != wWriteLen) {
        printf("target flash wReadSize data error. 0x%x.\r\n", wReadSize);
        return false;
    }

    uint16_t hwReadCheck = ymodem_crc16(pchBuffer, wWriteLen);

    if(hwWriteCheck != hwReadCheck) {
        printf("Check error. WriteCheck:0x%x ReadCheck:0x%x.\r\n", hwWriteCheck, hwReadCheck);
        return false;
    }

    this.wOffSet += wWriteLen;

    if(this.wOffSet == this.wFileSize) {
        end_to_boot();
		target_flash_uninit(APP_PART_ADDR);
    }
    
    return true;
}

int64_t get_system_time_ms(void)
{
    return perfc_convert_ticks_to_ms(get_system_ticks());
}

static uint16_t ymodem_read_data(ymodem_t *ptObj, uint8_t* pchByte, uint16_t hwSize)
{
    ymodem_receive_t *(ptThis) = (ymodem_receive_t *)ptObj;
    assert(NULL != ptObj);
    return dequeue(&this.tByteInQueue, pchByte, hwSize);
}

static bool ymodem_write_data(ymodem_t *ptObj, uint8_t* pchByte, uint16_t hwSize)
{
    ymodem_receive_t *(ptThis) = (ymodem_receive_t *)ptObj;
    assert(NULL != ptObj);
    CanWriteData(tCAN_SEND_ID.CanID, pchByte, hwSize);	
    return true;
}

check_agent_ymodem_recive_t *check_agent_ymodem_receive_init(check_agent_ymodem_recive_t *ptObj)
{
    check_agent_ymodem_recive_t *(ptThis) = ptObj;
    assert(NULL != ptObj);
    this.tCheckAgent.pAgent = &this.tYmodemReceive.parent;
    this.tCheckAgent.fnCheck = (check_hanlder_t *)ymodem_receive;
    this.tCheckAgent.ptNext = NULL;

    this.tYmodemReceive.pchQueueBuf = s_chQueueBuffer;

    if(this.tYmodemReceive.pchQueueBuf != NULL) {
        queue_init(&this.tYmodemReceive.tByteInQueue, this.tYmodemReceive.pchQueueBuf, sizeof(s_chQueueBuffer));
    }

    static ymodem_ops_t s_tOps = {
        .pchBuffer = s_chBuffer,
        .hwSize = sizeof(s_chBuffer),
        .fnOnFileData = ymodem_recv_file_data,
        .fnOnFilePath = ymodem_recv_file_name,
        .fnReadData = ymodem_read_data,
        .fnWriteData = ymodem_write_data,
    };
    ymodem_init(&this.tYmodemReceive.parent, &s_tOps);

    return ptObj;
}

