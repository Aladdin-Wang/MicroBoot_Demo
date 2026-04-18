/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "main.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "SEGGER_RTT.h"
#include "multiple_delay.h"
#include "micro_shell.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
__attribute__((aligned(32)))
uint8_t s_chBuffer[2048] ;
static byte_queue_t                  s_tCheckUsePeekQueue;
static fsm_check_use_peek_t          s_fsmCheckUsePeek;
static check_shell_t                 s_tShellObj;

multiple_delay_t tDelayService;
static uint8_t s_chDelayPool[10 * sizeof(multiple_delay_item_t)];

typedef struct {
	multiple_delay_item_t *ptDelayItem;
    char ch;
    uint32_t delay_tick;
    multiple_delay_request_priority_t priority;
} delay_task_param_t;

static delay_task_param_t s_tHighTask = {
    .ch = 'H',
    .delay_tick = 1000,
    .priority = MULTIPLE_DELAY_HIGH_PRIORITY,
};

static delay_task_param_t s_tNormalTask = {
    .ch = 'N',
    .delay_tick = 2000,
    .priority = MULTIPLE_DELAY_NORMAL_PRIORITY,
};

static delay_task_param_t s_tLowTask = {
    .ch = 'L',
    .delay_tick = 3000,
    .priority = MULTIPLE_DELAY_LOW_PRIORITY,
};

void on_delay_task_high(multiple_delay_report_status_t status, void *pTag)
{
    delay_task_param_t *ptTask = (delay_task_param_t *)pTag;

    if (status == MULTIPLE_DELAY_TIMEOUT) {
        printf("%c", ptTask->ch);

        ptTask->ptDelayItem = MULTIPLE_DELAY.RequestDelay(
            &tDelayService,
            ptTask->delay_tick,
            ptTask->priority,
            ptTask,
            on_delay_task_high
        );
    }
}

void on_delay_task_normal(multiple_delay_report_status_t status, void *pTag)
{
    delay_task_param_t *ptTask = (delay_task_param_t *)pTag;

    if (status == MULTIPLE_DELAY_TIMEOUT) {
        printf("%c", ptTask->ch);

        ptTask->ptDelayItem = MULTIPLE_DELAY.RequestDelay(
            &tDelayService,
            ptTask->delay_tick,
            ptTask->priority,
            ptTask,
            on_delay_task_normal
        );
    }
}

void on_delay_task_low(multiple_delay_report_status_t status, void *pTag)
{
    delay_task_param_t *ptTask = (delay_task_param_t *)pTag;

    if (status == MULTIPLE_DELAY_TIMEOUT) {
        printf("%c", ptTask->ch);

        ptTask->ptDelayItem = MULTIPLE_DELAY.RequestDelay(
            &tDelayService,
            ptTask->delay_tick,
            ptTask->priority,
            ptTask,
            on_delay_task_low
        );
    }
}


uint16_t shell_read_data(wl_shell_t *ptObj, char *pchBuffer, uint16_t hwSize)
{
    return SEGGER_RTT_Read(0, (uint8_t *)pchBuffer, hwSize);	
}

uint16_t shell_write_data(wl_shell_t *ptObj, const char *pchBuffer, uint16_t hwSize)
{
	return SEGGER_RTT_Write(0, (uint8_t *)pchBuffer, hwSize);	
}

int stdout_putchar (int ch)
{
	SEGGER_RTT_PutChar(0,(char)ch);
	return ch;
}

int reboot(void)
{
    SCB->AIRCR = 0X05FA0000 | (uint32_t)0x04;
    return 0;
}
MSH_CMD_EXPORT(reboot, reboot)

int cancel(int argc, char **argv)
{
	if(!strcmp(argv[1], "high")) {
        MULTIPLE_DELAY.Cancel(&tDelayService,s_tHighTask.ptDelayItem);
	}
	if(!strcmp(argv[1], "normal")) {
        MULTIPLE_DELAY.Cancel(&tDelayService,s_tNormalTask.ptDelayItem);
	}
	if(!strcmp(argv[1], "low")) {
        MULTIPLE_DELAY.Cancel(&tDelayService,s_tLowTask.ptDelayItem);
	}	
    return 0;
}
MSH_CMD_EXPORT(cancel,eg:cancel high/normal/low)


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	#define APP_BASE_ADDR  0x08005000

	SCB->VTOR = APP_BASE_ADDR;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  /* USER CODE BEGIN SysInit */
    SEGGER_RTT_Init();
	SEGGER_RTT_SetFlagsUpBuffer(0,SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
	SEGGER_RTT_SetFlagsDownBuffer(0,SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
    MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
    queue_init(&s_tCheckUsePeekQueue, s_chBuffer, sizeof(s_chBuffer));
    init_fsm(check_use_peek, &s_fsmCheckUsePeek, args(&s_tCheckUsePeekQueue));

    shell_ops_t s_tOps = {
        .fnReadData = shell_read_data,
		.fnWriteData = shell_write_data,
    };
    shell_init(&s_tShellObj,&s_tOps);
	agent_register(&s_fsmCheckUsePeek, &s_tShellObj.tCheckAgent); 
  
    multiple_delay_cfg_t cfg = {
        .pchBuffer = s_chDelayPool,
        .nSize = sizeof(s_chDelayPool),
    };

    MULTIPLE_DELAY.Init(&tDelayService, &cfg);

    s_tHighTask.ptDelayItem = MULTIPLE_DELAY.RequestDelay(
        &tDelayService,
        s_tHighTask.delay_tick,
        s_tHighTask.priority,
        &s_tHighTask,
        on_delay_task_high
    );

    s_tNormalTask.ptDelayItem = MULTIPLE_DELAY.RequestDelay(
        &tDelayService,
        s_tNormalTask.delay_tick,
        s_tNormalTask.priority,
        &s_tNormalTask,
        on_delay_task_normal
    );

    s_tLowTask.ptDelayItem = MULTIPLE_DELAY.RequestDelay(
        &tDelayService,
        s_tLowTask.delay_tick,
        s_tLowTask.priority,
        &s_tLowTask,
        on_delay_task_low
    );  
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  check_use_peek_task(&s_fsmCheckUsePeek );
	  MULTIPLE_DELAY.Task(&tDelayService);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

typedef struct {
    char chProjectName[16];
    char chHardWareVersion[16];
    char chSoftBootVersion[16];
    char chSoftAppVersion[16];	
    char chPortName[16];
	int wPortBaudrate;
} msgSig_t;
typedef struct {
    union {
        msgSig_t sig;
        char B[sizeof(msgSig_t)];
    } msg_data;
} user_data_t;

user_data_t  tUserData = {
    .msg_data.sig.chProjectName = "app",
};

typedef struct {
    void (*fnGoToBoot)(uint8_t *pchDate, uint16_t hwLength);
    bool (*target_flash_init)(uint32_t addr);
    bool (*target_flash_uninit)(uint32_t addr);
    int  (*target_flash_read)(uint32_t addr, uint8_t *buf, size_t size);
    int  (*target_flash_write)(uint32_t addr, const uint8_t *buf, size_t size);
    int  (*target_flash_erase)(uint32_t addr, size_t size);
} boot_ops_t;


void boot()
{
    memcpy(tUserData.msg_data.sig.chPortName, "UART1", strlen("UART1"));
    tUserData.msg_data.sig.wPortBaudrate = 115200;
    boot_ops_t *ptBootOps = (boot_ops_t *) BOOT_FLASH_OPS_ADDR;
    ptBootOps->fnGoToBoot((uint8_t *)tUserData.msg_data.B, sizeof(tUserData));
    reboot();
}
MSH_CMD_EXPORT(boot, go to bootloader);
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
