/****************************************************************************
*  Copyright 2022 kk (https://github.com/Aladdin-Wang)                                    *
*                                                                           *
*  Licensed under the Apache License, Version 2.0 (the "License");          *
*  you may not use this file except in compliance with the License.         *
*  You may obtain a copy of the License at                                  *
*                                                                           *
*     http://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                           *
*  Unless required by applicable law or agreed to in writing, software      *
*  distributed under the License is distributed on an "AS IS" BASIS,        *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. *
*  See the License for the specific language governing permissions and      *
*  limitations under the License.                                           *
*                                                                           *
****************************************************************************/

#ifndef __USE_SERVICE_H__
#define __USE_SERVICE_H__

/*============================ INCLUDES ======================================*/
#include "./app_cfg.h"

#ifdef WL_USING_RINGEQUEUE
#include "./generic/queue/wl_queue.h"
#endif

#ifdef WL_USING_SIGNALS_SLOTS
#include "./communication/signals_slots/wl_signals_slots.h"
#endif

#ifdef WL_USING_SUBSCRIBE_PUBLISH
#include "./communication/subscribe_publish/wl_subscribe_publish.h"
#endif

#ifdef WL_USING_SHELL
#include "./communication/shell/wl_shell.h"
#endif

#ifdef WL_USING_FLASH
#include "./flash_blob/wl_flash_blob.h"
#endif


#ifdef WL_USING_BOOT
#include "./bootloader/bootloader.h"
#endif

#endif
