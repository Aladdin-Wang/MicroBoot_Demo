#ifndef APPLICATIONS_CHECK_AGENT_XMODEM_SEND_H_
#define APPLICATIONS_CHECK_AGENT_XMODEM_SEND_H_
#include "./communication/ymodem/wl_ymodem.h"
#include "./check_agent_engine/wl_check_agent_engine.h"

typedef struct ymodem_send_t{
	ymodem_t      parent;
	uint8_t       chState;
	int16_t       hwReadSize;
	int16_t       hwIndex;
    char          *pchQueueBuf;	
	byte_queue_t  tByteInQueue;
	int64_t       lTargetTime;
    char         *pchFileName;
    char         *pchFileSize;
    uint32_t      wFileSize;
    uint32_t      wOffSet;
}ymodem_send_t;

typedef struct check_agent_ymodem_send_t{
    check_agent_t tCheckAgent;
    ymodem_send_t tYmodemSend;
}check_agent_ymodem_send_t;

extern check_agent_ymodem_send_t *check_agent_ymodem_send_init(check_agent_ymodem_send_t *ptObj);

#endif /* APPLICATIONS_CHECK_AGENT_XMODEM_H_ */
