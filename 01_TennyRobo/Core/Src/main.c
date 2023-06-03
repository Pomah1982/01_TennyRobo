/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct ServoMotor{
	uint16_t curValue;
	uint16_t trgValue;
	uint8_t increment;
	uint16_t min;
	uint16_t max;
	uint8_t chanel;
} ServoMotor;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// Принимаемая строка, содержащая параметры инфраструктуры
uint8_t receivedStr={0};
//отправляемая строка, содержащая параметры инфраструктуры
char trStr[40]={0};

uint16_t time = 1000;
uint16_t speed_min = 800;//x
uint16_t speed_max = 2300;//y
uint16_t angle_min = 400;//z
uint16_t angle_max = 2400;//u
uint16_t position_min = 400;//w
uint16_t position_max = 2400;//v
uint16_t loader_up = 500;
uint16_t loader_down = 1300;
volatile bool loader_redy = false;
volatile uint8_t timer_interupt_count;
volatile uint16_t period;
volatile int tmpInfValue = 0;

struct ServoMotor MotorTop;
struct ServoMotor MotorBottom;
struct ServoMotor Angle;
struct ServoMotor Position;
struct ServoMotor Loader;	//механизм подачи мячей

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Получить значение инкремента исходя из скорости подачи мячей (времени между выстрелами)
uint8_t getIncrement(struct ServoMotor *srv){
	return (srv->max - srv->min)*20/(time - 100) + 1;
}

//Сделать шаг для инфрастуктуры согласно текущему и целевому значениям и значению инкремента (плавно повернуть серво или сменить скорость двигателя)
void incrementValue(struct ServoMotor *srv){
	if(srv->trgValue == srv->curValue)return;	// Скорость/угол уже установлены
	if(srv->trgValue < srv->min) srv->trgValue = srv->min;	//защита от утановки значения, меньшего, чем минимальнодопустимое для данного серво
	if(srv->trgValue > srv->max) srv->trgValue = srv->max;	//защита от установки значения, большего, чем максимально допустимое для данного серво
	if(srv->curValue < srv->trgValue)	// Необходимо увеличить скорость/угол
	{
		srv->curValue += srv->increment;
		if(srv->curValue > srv->trgValue){
			srv->curValue = srv->trgValue;	//если произошел чрезмерный сдвиг, то устанавливаем текущее значение == целевому
			__HAL_TIM_SetCompare(&htim2, srv->chanel, srv->curValue);
		}

		return;
	}
	//Необходимо уменьшать скорость/угол
	srv->curValue -= srv->increment;
	if(srv->curValue < srv->trgValue) {
		srv->curValue = srv->trgValue;	//если произошел чрезмерный сдвиг, то устанавливаем текущее значение == целевому
		__HAL_TIM_SetCompare(&htim2, srv->chanel, srv->curValue);	//устанавливаем скважность импульсов pwm
	}
}

//Установка основных параметров инфраструктуры
void initServo(struct ServoMotor *srv,uint16_t min,uint16_t max,uint16_t cur,uint8_t chanel){
	srv->curValue = cur;
	srv->trgValue = min;
	srv->min = min;
	srv->max = max;
	srv->increment = getIncrement(srv);
	srv->chanel = chanel;
}

//Установить новое значение инкремента для инфраструктуры
void changeIncrement(){
	MotorTop.increment = getIncrement(&MotorTop);
	MotorBottom.increment = getIncrement(&MotorBottom);
	Angle.increment = getIncrement(&Angle);
	Position.increment = getIncrement(&Position);
}

//Установить первоначальные значения параметров инфраструктуры (min,max,cur)
void initInfrastructure(){
	initServo(&MotorTop,speed_min,speed_max, speed_min+1,TIM_CHANNEL_1);
	initServo(&MotorBottom,speed_min,speed_max, speed_min+1,TIM_CHANNEL_2);
	initServo(&Angle,angle_min,angle_max,(angle_min+angle_max)/2,TIM_CHANNEL_3);
	initServo(&Position,position_min,position_max,(position_min+position_max)/2,TIM_CHANNEL_4);
}

//Поднять загрузчик мячей и взять мяч из лотка
void loaderUp(){
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, loader_up);
}

//Опустить загрузчик мячей для сброса мяча в пушку
void loaderDown(){
	loader_redy = false;
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, loader_down);
}

//Установить значения целевых параметров инфраструктуры согласно полученным с android параметрам
void setInfValue(char endSymbol){
	switch (endSymbol){
	case 'm': MotorTop.trgValue = tmpInfValue; break;	//УСТАНАВЛ�?ВАЕМ СКОРОСТЬ ПЕРВОГО ДВ�?ГАТЕЛЯ
	case 'n': MotorBottom.trgValue = tmpInfValue; break;	//УСТАНАВЛ�?ВАЕМ СКОРОСТЬ ВТОРОГО ДВ�?ГАТЕЛЯ
	case 'a': Angle.trgValue = tmpInfValue; break;	//УСТАНАВЛ�?ВАЕМ УГОЛ SPIN-a
	case 'p': Position.trgValue = tmpInfValue; break;	//УСТАНАВЛ�?ВАЕМ ПОВОРОТ РОБОТА ВЛЕВО/ВПРАВО
	case 'b': loaderDown(); break; //Далее сработает прерывание от датчика выстрела и лопатка поднимется вверх
	default: break;
	}
	tmpInfValue = 0;
}

//Получить отправляемую строку, содеражщую параметры инфраструктуры
void getTransStr(){
	memset(trStr, 0, strlen(trStr));//очищаем строку перед заполнением
	sprintf(trStr,"%s%d%s%d%s%d%s%d%s%d%c",
			"m:",MotorTop.curValue,"|n:",MotorBottom.curValue,
			"|a:",Angle.curValue,"|p:",Position.curValue,"|t:",time,'e');
}

//Настройка минимальной и максимальной частот вращения двигателей
void motorsInitialization(){
	HAL_Delay(1000);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, speed_max);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, speed_max);
	HAL_Delay(2000);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, speed_min);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, speed_min);
	HAL_Delay(1000);
}

void UART1_RxCpltCallback(void){
	uint8_t b;
	b = receivedStr;
	if(b=='g' || b=='m' || b=='n' || b=='a' || b=='p' || b=='b' || b=='t'){
		switch (b) {
			case 'g':
				getTransStr();
				HAL_UART_Transmit(&huart1, (uint8_t*)trStr, strlen(trStr),0x1000);//НУЖНО ОТПРАВЛЯТЬ ПАРАМЕТРЫ �?НФРАСТРУКТУРЫ
				tmpInfValue = 0;
				break;
			case 'b': HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);	//ТУТ НАДО ЗАПУСКАТЬ ВЫСТРЕЛ
				tmpInfValue = 0;
				break;
			case 't': time = tmpInfValue; tmpInfValue = 0; changeIncrement();
			default: setInfValue(b); break;
		}

		HAL_UART_Receive_IT(&huart1,&receivedStr,1);
		return;
	}
	tmpInfValue = 10*tmpInfValue + (b - '0');
	HAL_UART_Receive_IT(&huart1,&receivedStr,1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim2)
    {
    	incrementValue(&MotorTop);
    	incrementValue(&MotorBottom);
    	incrementValue(&Angle);
    	incrementValue(&Position);
    }
    if (htim == &htim3)
    {
    	timer_interupt_count++;
    	if(timer_interupt_count >= period){
    		if(loader_redy){
    			loaderDown();
    		}
    		else timer_interupt_count = period;//Если мяч не упал в загрузчик мячей, то ждем до следующего прерывания таймера (20мС)
    	}
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin== LOAD_SENSOR_Pin) {
		loader_redy = true;
	}
	if(GPIO_Pin== PUSH_SENSOR_Pin) {
		loaderUp();
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &receivedStr,1);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  initInfrastructure();	//Инициализируем инфраструктуру
  motorsInitialization();
  /* USER CODE END 2 */

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

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LOAD_SENSOR_Pin PUSH_SENSOR_Pin */
  GPIO_InitStruct.Pin = LOAD_SENSOR_Pin|PUSH_SENSOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart==&huart1){
		UART1_RxCpltCallback();
	}
}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
//	if(htim==&htim2){
//		//HAL_UART_Transmit(&huart2,(uint8_t*)str2[i],strlen(str2[i]),0x1000);
//
//
//	}
//}
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
