/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef uint8_t		bool;
typedef uint32_t	blinkInterval;

typedef struct {
	bool			isOn;
	blinkInterval	blinkInt;
} ledStatus_t;

typedef struct {
	uint8_t	 messg[256];
	uint16_t len;
} queueMess_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TX_TIMEOUT   	50
#define RX_TIMEOUT   	500
#define QUEUE_TIMEOUT	50
#define MSSG_LENGTHS 	256

char* name = 		"\n\n\r ___      _______  ______     _______  ___      ___   __    _  ___   _  _______  ______   \n\r"
					"|   |    |       ||      |   |  _    ||   |    |   | |  |  | ||   | | ||       ||    _ |  \n\r"
					"|   |    |    ___||  _    |  | |_|   ||   |    |   | |   |_| ||   |_| ||    ___||   | ||  \n\r"
					"|   |    |   |___ | | |   |  |       ||   |    |   | |       ||      _||   |___ |   |_||_ \n\r"
					"|   |___ |    ___|| |_|   |  |  _   | |   |___ |   | |  _    ||     |_ |    ___||    __  |\n\r"
					"|       ||   |___ |       |  | |_|   ||       ||   | | | |   ||    _  ||   |___ |   |  | |\n\r"
					"|_______||_______||______|   |_______||_______||___| |_|  |__||___| |_||_______||___|  |_|\n\n\n\r";

char* instrucrin ="\n\n\rCommand for communication and control LED ON, LED OFF, LED BLINK <TIME>, STATUS.\n\n\r";



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* Definitions for receiveData */
osThreadId_t receiveDataHandle;
const osThreadAttr_t receiveData_attributes = {
  .name = "receiveData",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for parseData */
osThreadId_t parseDataHandle;
const osThreadAttr_t parseData_attributes = {
  .name = "parseData",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ledControl */
osThreadId_t ledControlHandle;
const osThreadAttr_t ledControl_attributes = {
  .name = "ledControl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for messageQueue */
osMessageQueueId_t messageQueueHandle;
const osMessageQueueAttr_t messageQueue_attributes = {
  .name = "messageQueue"
};
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartreceiveDataTask(void *argument);
void StartParseDataTask(void *argument);
void StartledControl(void *argument);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

ledStatus_t	ledStatus;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	ledStatus.isOn		= 0;
	ledStatus.blinkInt	= 1000;

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
  MX_USART2_UART_Init();
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

  /* Create the queue(s) */
  /* creation of messageQueue */
  messageQueueHandle = osMessageQueueNew (30, sizeof(queueMess_t), &messageQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of receiveData */
  receiveDataHandle = osThreadNew(StartreceiveDataTask, NULL, &receiveData_attributes);

  /* creation of parseData */
  parseDataHandle = osThreadNew(StartParseDataTask, NULL, &parseData_attributes);

  /* creation of ledControl */
  ledControlHandle = osThreadNew(StartledControl, NULL, &ledControl_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  HAL_UART_Transmit(&huart2, (const uint8_t*)name, strlen(name), TX_TIMEOUT);
  HAL_UART_Transmit(&huart2, (const uint8_t*)instrucrin, strlen(instrucrin), TX_TIMEOUT);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartreceiveDataTask */
/**
  * @brief  Function implaementing the receiveData thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartreceiveDataTask */
void StartreceiveDataTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	  HAL_StatusTypeDef	status;
	  uint16_t			buffLen;
	  uint8_t			mssgBuff[256];

	  memset(mssgBuff, '\0', MSSG_LENGTHS);
	  for(;;)
	  {
		if ((status=HAL_UART_Receive(&huart2, mssgBuff, sizeof(uint8_t)*MSSG_LENGTHS, RX_TIMEOUT)) == HAL_OK || status == HAL_TIMEOUT)
		{
			if ((buffLen=strlen((const char*)mssgBuff)))
			{
				queueMess_t mssgCnt;

				mssgCnt.len = buffLen;
				memcpy(mssgCnt.messg, mssgBuff, MSSG_LENGTHS);

				if (!osMessageQueuePut(messageQueueHandle, &mssgCnt, 0, QUEUE_TIMEOUT))
				{
//					char*	str = "UART Reciver task : message put in queue.\n\r";
//					HAL_UART_Transmit(&huart2, (const uint8_t*)str, strlen(str), TX_TIMEOUT);
					memset(mssgBuff, '\0', MSSG_LENGTHS);
				}
			}
		}
		else if (status == HAL_BUSY)
		{
			char*	bussy = "UART Reciver task : UART port was bussy\n\r";
			HAL_UART_Transmit(&huart2, (const uint8_t*)bussy, strlen(bussy), TX_TIMEOUT);
		}
		else if (status == HAL_ERROR)
		{
			char*	bussy = "UART Reciver task : unexpected error while receiving data from UART\n\r";
			HAL_UART_Transmit(&huart2, (const uint8_t*)bussy, strlen(bussy), TX_TIMEOUT);
			Error_Handler();
		}
	    osDelay(50);
	  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartParseDataTask */
/**
* @brief Function implementing the parseData thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartParseDataTask */
void StartParseDataTask(void *argument)
{
  /* USER CODE BEGIN StartParseDataTask */
  /* Infinite loop */
  for(;;)
  {
	  char*	pBuff;
	  queueMess_t mssgCnt;

	  mssgCnt.len = 0;
	  memset(mssgCnt.messg, '\0', MSSG_LENGTHS);

	  if (!osMessageQueueGet(messageQueueHandle, &mssgCnt, 0, QUEUE_TIMEOUT))
	  {

//		  char* func = "Parse task : Item are geted \n\r";
//		  HAL_UART_Transmit(&huart2, mssgCnt.messg, sizeof(uint8_t)*mssgCnt.len, TX_TIMEOUT);
//		  char* str = "\n\r";
//		  HAL_UART_Transmit(&huart2, (const uint8_t*)func, strlen(func), TX_TIMEOUT);

		  pBuff = (char*) mssgCnt.messg;

		  if (!strcmp(pBuff, "LED ON\r"))
		  {
			  char* str = "\n\r-------------------\n\rLed is on now.\n\r-------------------\n\n\r";
		  	  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), 500);
		  	  ledStatus.isOn = 1;
		  }
		  else if (!strcmp(pBuff, "LED OFF\r"))
		  {
			  char* str = "\n\r-------------------\n\rLed is off now.\n\r-------------------\n\n\r";
		  	  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), 500);
			  ledStatus.isOn = 0;
		  }
		  else if (!strncmp(pBuff, "LED BLINK ", 10))
		  {
			  uint16_t blinkRate;

			  blinkRate = atoi(pBuff + 10);
		  	  ledStatus.blinkInt = blinkRate;

			  char* str = "\n\r-------------------\n\rBlink rate : ";
			  char* str1 = "\n\r-------------------\n\n\r";
		  	  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), 500);
		  	  HAL_UART_Transmit(&huart2, (uint8_t *)(pBuff + 10), strlen (pBuff + 10), 500);
		  	  HAL_UART_Transmit(&huart2, (uint8_t *)str1, strlen (str1), 500);
		  }
		  else if (!strcmp(pBuff, "STATUS\r"))
		  {
			  uint8_t numb[15];
		  	  char*		ledIs;
		  	  char* 	str0 = "\n\r-------------------\n\n\r";
		  	  char* 	str1 = "\n\r-------------------\n\rStatus :\n\rled is : on\n\rblink interval : ";
		  	  char* 	str2 = "\n\r-------------------\n\rStatus :\n\rled is : off\n\rblink interval : ";

		  	  memset(numb, '\0', 10);
		  	  ledIs = (ledStatus.isOn) ? str1 : str2;
		  	  itoa((int)ledStatus.blinkInt, (char*)numb, 10);

		  	  HAL_UART_Transmit(&huart2, (uint8_t *)ledIs, strlen (ledIs), 500);
		  	  HAL_UART_Transmit(&huart2, (uint8_t *)numb, strlen ((const char*)numb), 500);
		  	  HAL_UART_Transmit(&huart2, (uint8_t *)str0, strlen (str0), 500);
		  }
		  else
		  {
			  char*	 str = "\n\n\rCommand not correct ::\n\rCheck spaces in word\n\rCorrect command LED ON, LED OFF, LED BLINK <TIME>, STATUS\n\n\r";
		  	  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), 500);
		  }

	  }
    osDelay(1000);
  }
  /* USER CODE END StartParseDataTask */
}

/* USER CODE BEGIN Header_StartledControl */
/**
* @brief Function implementing the ledControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartledControl */
void StartledControl(void *argument)
{
  /* USER CODE BEGIN StartledControl */
  /* Infinite loop */
  uint32_t tick;

  for(;;)
  {
	tick = 1000;
	if (ledStatus.blinkInt > 15 && ledStatus.isOn)
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	tick = ledStatus.blinkInt;
	}
	else
	{
		if (ledStatus.isOn)
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
		else
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
	}
    osDelay(tick);
  }
  /* USER CODE END StartledControl */
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
