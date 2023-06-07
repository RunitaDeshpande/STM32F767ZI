/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"
#include<stdio.h>
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

/* Definitions for ReceiverTask */
osThreadId_t ReceiverTaskHandle;
const osThreadAttr_t ReceiverTask_attributes = {
  .name = "ReceiverTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Sender01 */
osThreadId_t Sender01Handle;
const osThreadAttr_t Sender01_attributes = {
  .name = "Sender01",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Sender02 */
osThreadId_t Sender02Handle;
const osThreadAttr_t Sender02_attributes = {
  .name = "Sender02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartReceiverTask(void *argument);
void StartSender01(void *argument);
void StartSender02(void *argument);

/* USER CODE BEGIN PFP */
QueueHandle_t xQueue;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef enum
{
eSender1,
eSender2
} DataSource_t;

typedef struct
{
	int ucValue;
	DataSource_t eDataSource;

} Data_t;

static const Data_t xStructsToSend[ 2 ] =
{
{ 100, eSender1 }, /* Used by Sender1. */
{ 200, eSender2 } /* Used by Sender2. */
};


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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  xQueue = xQueueCreate(3, sizeof(Data_t));

  /* USER CODE END RTOS_QUEUES */
  /* Create the thread(s) */
  /* creation of ReceiverTask */

  ReceiverTaskHandle = osThreadNew(StartReceiverTask, NULL, &ReceiverTask_attributes);

  /* creation of Sender01 */
  Sender01Handle = osThreadNew(StartSender01, &(xStructsToSend[ 0 ]), &Sender01_attributes);

  /* creation of Sender02 */
  Sender02Handle = osThreadNew(StartSender02, &(xStructsToSend[ 1 ]), &Sender02_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */


  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, gled_Pin|rled_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : gled_Pin rled_Pin */
  GPIO_InitStruct.Pin = gled_Pin|rled_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartReceiverTask */
/**
  * @brief  Function implementing the ReceiverTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartReceiverTask */
void StartReceiverTask(void *argument)
{
	Data_t xReceivedStructure;
	BaseType_t xStatus;
	/* This task is also defined within an infinite loop. */
	for( ;; )
	{

	if( uxQueueMessagesWaiting( xQueue ) == 3 )
	{
	printf( "Queue is full!\r\n" );
	}

	xStatus = xQueueReceive( xQueue, &xReceivedStructure, 100);
	if( xStatus == pdPASS )
	{
	/* Data was successfully received from the queue, print out the received
	value and the source of the value. */
	  if( xReceivedStructure.eDataSource == eSender1 )
	  {
	  printf( "From Sender 1 = %d\n", xReceivedStructure.ucValue );
	  }
	  else
	  {
	  printf( "From Sender 2 = %d\n", xReceivedStructure.ucValue );
	  }
	}
	else
	{
	/* Nothing was received from the queue. This must be an error as this
	task should only run when the queue is full. */
	printf( "Could not receive from the queue.\r\n" );
	}
	vTaskDelay(5);
	}
}

/* USER CODE BEGIN Header_StartSender01 */
/**
* @brief Function implementing the Sender01 thread.
* @param argument: Not used
* @retval None
*/

/* USER CODE END Header_StartSender01 */
void StartSender01(void *argument)
{
 /* USER CODE BEGIN StartSender01 */
 BaseType_t xStatus;
 const TickType_t xTicksToWait = pdMS_TO_TICKS(100);

 /* Infinite loop */
 for(;;)
	{
		xStatus=xQueueSendToBack(xQueue,argument,xTicksToWait);

		if(xStatus != pdPASS)
		{
			printf("could not send to the Queue\n");
		}
		else
		{
			printf("\r Send pass \r\n");
			vTaskDelay(pdMS_TO_TICKS(10));
		}
	}
  /* USER CODE END StartSender01 */
}

/* USER CODE BEGIN Header_StartSender02 */
/**
* @brief Function implementing the Sender02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSender02 */
void StartSender02(void *argument)
{
	/* USER CODE BEGIN StartSender01 */
	BaseType_t xStatus;
	 const TickType_t xTicksToWait =pdMS_TO_TICKS(100);
	  /* Infinite loop */
	for(;;)
	{
		xStatus=xQueueSendToBack(xQueue,argument,xTicksToWait);
		if(xStatus != pdPASS)
		{
			printf("could not send to the Queue\n");

		}
		else
		{
			printf("\r Send pass \r\n");
			vTaskDelay(pdMS_TO_TICKS(10));
		}
	}
}


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
