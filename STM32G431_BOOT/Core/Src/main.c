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
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "check_agent_engine.h"
#include "micro_shell.h"
#include "ymodem_ota.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t LL_flash_write(uint32_t addr, uint64_t* pdata64, uint32_t len_64);
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
uint8_t s_chBuffer[2048 + 128] ;
static byte_queue_t                  s_tCheckUsePeekQueue;
static fsm(check_use_peek)           s_fsmCheckUsePeek;
static ymodem_ota_recive_t           s_tYmodemOtaReceive;
static check_shell_t                 s_tShellObj;

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

uint16_t shell_read_data(wl_shell_t *ptObj, char *pchBuffer, uint16_t hwSize)
{
    peek_byte_t *ptReadByte = get_read_byte_interface(&s_fsmCheckUsePeek);
    return ptReadByte->fnGetByte(ptReadByte, (uint8_t *)pchBuffer, hwSize);
}

uint16_t shell_write_data(wl_shell_t *ptObj, const char *pchBuffer, uint16_t hwSize)
{
	HAL_UART_Transmit(&huart3, (uint8_t *)pchBuffer, hwSize, 100);
	return hwSize;
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
	queue_init(&s_tCheckUsePeekQueue, s_chBuffer, sizeof(s_chBuffer));
    init_fsm(check_use_peek, &s_fsmCheckUsePeek, args(&s_tCheckUsePeekQueue));
    shell_ops_t s_tOps = {
        .fnReadData = shell_read_data,
		.fnWriteData = shell_write_data,
    };
    shell_init(&s_tShellObj,&s_tOps);
	agent_register(&s_fsmCheckUsePeek, &s_tShellObj.tCheckAgent);
		
    ymodem_ota_receive_init(&s_tYmodemOtaReceive, get_read_byte_interface(&s_fsmCheckUsePeek));
    agent_register(&s_fsmCheckUsePeek, &s_tYmodemOtaReceive.tCheckAgent);

    connect(&tUartMsgObj, SIGNAL(uart_sig), &s_tCheckUsePeekQueue, SLOT(enqueue_bytes));
		
    connect(&s_tYmodemOtaReceive.tYmodemReceive, SIGNAL(ymodem_rec_sig), &huart3, SLOT(uart_sent_data));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      call_fsm( check_use_peek,  &s_fsmCheckUsePeek );
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint8_t LL_flash_write(uint32_t addr, uint64_t* pdata64, uint32_t len_64)
{
  uint8_t ret;
  uint64_t read;
  
  HAL_FLASH_Unlock();           //上锁，以结束FLASH操作
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

  for(uint16_t j=0; j<len_64; j++) //按uint64_t计算的
  {
    for(uint8_t i=0; i<10; i++)    //不超过10次的重复操作，以保证写入成功
    {
      ret = HAL_FLASH_Program(0, addr+j*8, *(pdata64+j));
      if(ret == HAL_OK)
      {
        read = *(__IO uint64_t *)(addr+j*8);  //在MCU认为写入正确以后，再次读取一下数据，并进行比对，如果比对不成功，也说明写入出错
        if(read != *(pdata64+j))
          ret = 255;
        return ret;
      }
    }
  }
  
  HAL_FLASH_Lock();           //上锁，以结束FLASH操作

  return ret;
}
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

#ifdef  USE_FULL_ASSERT
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
