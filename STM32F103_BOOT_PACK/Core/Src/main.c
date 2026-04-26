/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "check_agent_engine.h"
#include "ymodem_ota.h"
#include "micro_shell.h"
#include "multiple_delay.h"
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
static ymodem_ota_recive_t           s_tYmodemOtaReceive;
static check_agent_shell_t           s_tShellObj;
multiple_delay_t                     tDelayService;
static uint8_t s_chDelayPool[10 * sizeof(multiple_delay_item_t)];
static multiple_delay_item_t *s_ptDelayBootItem = NULL;

int reboot(void)
{
    SCB->AIRCR = 0X05FA0000 | (uint32_t)0x04;
    return 0;
}
MSH_CMD_EXPORT(reboot, reboot)

int64_t get_system_time_ms(void)
{
    return HAL_GetTick();
}

uint16_t shell_read_data(micro_shell_t *ptObj, char *pchBuffer, uint16_t hwSize)
{
    peek_byte_t *ptReadByte = get_read_byte_interface(&s_fsmCheckUsePeek);
    return ptReadByte->fnGetByte(ptReadByte, (uint8_t *)pchBuffer, hwSize);	
}

uint16_t shell_write_data(micro_shell_t *ptObj, const char *pchBuffer, uint16_t hwSize)
{
    HAL_GPIO_WritePin(EN_485_GPIO_Port, EN_485_Pin, GPIO_PIN_SET);	
	HAL_UART_Transmit(&huart1, (uint8_t *)pchBuffer, hwSize, 100);	
	HAL_GPIO_WritePin(EN_485_GPIO_Port, EN_485_Pin, GPIO_PIN_RESET);
	return hwSize;
}



/* 
 * @brief  Boot control flag stored in NOINIT section.
 *         This variable is preserved across reset and used to control
 *         whether the system should jump to APP after reboot.
 */
NOINIT
static volatile uint32_t s_wEnterAppFlag;
#define ENTER_APP_FLAG   0x55555555 /* Magic value indicating that APP entry is allowed */

/*
 * @brief  Check if entering APP is allowed.
 * @retval true  APP can be entered
 * @retval false Stay in bootloader
 */
bool can_enter_app(void)
{
    return (s_wEnterAppFlag == ENTER_APP_FLAG);
}

/*
 * @brief  Enable APP entry.
 *         This sets the flag so that after reboot the system jumps to APP.
 */
void enable_enter_app(void)
{
    s_wEnterAppFlag = ENTER_APP_FLAG;
}

/*
 * @brief  Decide whether to stay in bootloader.
 * @retval true  Stay in bootloader
 * @retval false Jump to APP
 */
bool user_enter_bootloader(void)
{
    return !can_enter_app();
}

/*
 * @brief  Delay timer callback.
 *         If timeout occurs and APP entry is not yet enabled,
 *         enable it and reboot to enter APP.
 *
 * @param  status Timer status
 * @param  pTag   User context (unused)
 */
void on_delay_event(multiple_delay_report_status_t status, void *pTag)
{
    user_magic_data_t *ptMagicData = (user_magic_data_t *)pTag;
    if (status == MULTIPLE_DELAY_TIMEOUT) {
		if(!can_enter_app()){
		    enable_enter_app();
            reboot();/* Reboot to apply APP entry */
		}
    }
	s_ptDelayBootItem = NULL;
}



/*
 * @brief  Shell command: enter APP immediately.
 *         Cancel delay and allow APP entry.
 */
int boot(void)
{	
	enable_enter_app();
	if(s_ptDelayBootItem != NULL){
        MULTIPLE_DELAY.Cancel(&tDelayService,s_ptDelayBootItem);
	}
    return 0;
}
MSH_CMD_EXPORT(boot, enter bootloader)


void ymodem_state_handler(ymodem_state_t state)
{
    if(state == STATE_FINSH){
		enable_enter_app();
		reboot();
	}
}
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
    /* Set the Vector Table base location at 0x08000000 */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

    queue_init(&s_tCheckUsePeekQueue, s_chBuffer, sizeof(s_chBuffer));
    init_fsm(check_use_peek, &s_fsmCheckUsePeek, args(&s_tCheckUsePeekQueue));

    ymodem_ota_receive_init(&s_tYmodemOtaReceive, get_read_byte_interface(&s_fsmCheckUsePeek));
    agent_register(&s_fsmCheckUsePeek, &s_tYmodemOtaReceive.tCheckAgent);
	connect(&s_tYmodemOtaReceive.tYmodemReceive, SIGNAL(ymodem_rec_sig), &huart1, SLOT(uart_sent_data));
    connect(&tUartMsgObj, SIGNAL(uart_sig), &s_tCheckUsePeekQueue, SLOT(enqueue_bytes));
	
    shell_ops_t s_tOps = {
        .fnReadData = shell_read_data,
		.fnWriteData = shell_write_data,
    };
    shell_agent_init(&s_tShellObj,&s_tOps);
	agent_register(&s_fsmCheckUsePeek, &s_tShellObj.tCheckAgent);

    multiple_delay_cfg_t cfg = {
        .pchBuffer = s_chDelayPool,
        .nSize = sizeof(s_chDelayPool),
    };
    MULTIPLE_DELAY.Init(&tDelayService, &cfg);
	
    s_ptDelayBootItem = MULTIPLE_DELAY.RequestDelay(
        &tDelayService,
        1000,
        MULTIPLE_DELAY_NORMAL_PRIORITY,
        &tUserMagicData,
        on_delay_event
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


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
