#ifndef APPLICATIONS_CHECK_AGENT_XMODEM_RECEIVE_H_
#define APPLICATIONS_CHECK_AGENT_XMODEM_RECEIVE_H_
#include "./communication/ymodem/wl_ymodem.h"
#include "./check_agent_engine/wl_check_agent_engine.h"

typedef struct ymodem_receive_t {
    ymodem_t      parent;
    char         *pchQueueBuf;
    byte_queue_t  tByteInQueue;
    char         *pchFileName;
    char         *pchFileSize;
    uint32_t      wFileSize;
    uint32_t      wOffSet;
} ymodem_receive_t;

typedef struct check_agent_ymodem_recive_t {
    check_agent_t tCheckAgent;
    ymodem_receive_t tYmodemReceive;
} check_agent_ymodem_recive_t;

extern check_agent_ymodem_recive_t *check_agent_ymodem_receive_init(check_agent_ymodem_recive_t *ptObj);

#endif /* APPLICATIONS_CHECK_AGENT_XMODEM_H_ */
