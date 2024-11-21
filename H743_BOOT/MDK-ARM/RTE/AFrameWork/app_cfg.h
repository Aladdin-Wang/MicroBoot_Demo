/* WALI Serve config file */

#ifndef __WL_SERVE_CFG_H__
#define __WL_SERVE_CFG_H__

// <<< Use Configuration Wizard in Context Menu >>>

// <h>Debug Configuration
// <c1>enable kernel debug configuration
//  <i>Default: enable kernel debug configuration
//#define RT_DEBUG
// </c>
// <o>enable components initialization debug configuration<0-1>
//  <i>Default: 0
#define RT_DEBUG_INIT 0
// <c1>thread stack over flow detect
//  <i> Diable Thread stack over flow detect
//#define RT_USING_OVERFLOW_CHECK
// </c>
// </h>

// <e>Msg Map Configuration
// <i> Enables msg_map
#define WL_USING_MSG_MAP         1
#if WL_USING_MSG_MAP == 0
    #undef WL_USING_MSG_MAP
#endif
// <o>The maximum length of parameters <0-128>
//  <i>Default: 128
#define MSG_ARG_LEN             128
// <o>The maximum number of parameters <0-16>
//  <i>Default: 8
#define MSG_ARG_MAX             8
// </e>

// <e>Shell Configuration
// <i> Enables shell
#define WL_USING_SHELL         1
#if WL_USING_SHELL == 0
    #undef WL_USING_SHELL
#endif
// <q>Use echo mode
// <i> FINSH_OPTION_ECHO=0x01 is echo mode, other values are none-echo mode.
#define SHELL_OPTION_ECHO                             0
// <o>The maximum number of history lines <0-16>
//  <i>Default: 8
#define SHELL_HISTORY_LINES                           5
// </e>

// <h>IPC(Inter-process communication) Configuration
// <c1>Using Ring Queue
//  <i>Using Ring Queue
#define WL_USING_RINGEQUEUE
// </c>
// <c1>Using Signals-Slots
//  <i>Using Signals-Slots
#define WL_USING_SIGNALS_SLOTS
// </c>
// <c1>Using Subscribe-Publish
//  <i>Using Subscribe-Publish
#define WL_USING_SUBSCRIBE_PUBLISH
// </c>
// <c1>Using Mutex
//  <i>Using Mutex
//#define WL_USING_MUTEX
// </c>
// <c1>Using Event
//  <i>Using Event
//#define WL_USING_EVENT
// </c>
// <c1>Using MailBox
//  <i>Using MailBox
//#define WL_USING_MAILBOX
// </c>
// </h>

// <h>Memory Management Configuration
// <c1>Memory Pool Management
//  <i>Memory Pool Management
//#define RT_USING_MEMPOOL
// </c>
// <c1>Dynamic Heap Management(Algorithm: small memory )
//  <i>Dynamic Heap Management
#define RT_USING_HEAP
#define RT_USING_SMALL_MEM
// </c>
// <c1>using tiny size of memory
//  <i>using tiny size of memory
//#define RT_USING_TINY_SIZE
// </c>
// </h>

// <e>BootLoader Configuration
// <i> Enables BootLoader
#define WL_USING_BOOT         1
#if WL_USING_BOOT == 0
    #undef WL_USING_BOOT
#endif
// <o>The starting address of the app
//  <i>Default: 0x8020000
//  <i>Must align with flash sector size
#define APP_PART_ADDR             0x8020000
// <o>The app size
//  <i>Default: 0X60000
//  <i>Must align with flash sector size
#define APP_PART_SIZE             0xC8000
// <o>The user data size
//  <i>Default: 96
//  <i>Must be aligned to 32 bytes
#define USER_DATA_SIZE            96
// </e>
#define USER_DATA_SIZE_ALIGND     (((USER_DATA_SIZE) + (32) - 1) & ~((32) - 1))
// <<< end of configuration section >>>

#endif
