/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"



/* USER CODE BEGIN Includes */

#include "SPI_TFT.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h> 
#include <math.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint16_t i = 0;
uint16_t adc = 0;
uint16_t adc_last = 0;
uint16_t B_rate = 0;
uint16_t B_rate_mean = 0;

uint8_t menu=0;
uint8_t save=0;

uint16_t b_color = COLOR(255, 255, 255);

// значения настроек
uint8_t x_scale = 1;
uint8_t y_scale = 20;
uint8_t rate_filter = 160;
uint8_t rate_min = 25;
uint8_t rate_max = 125;

// флэш
#define MY_FLASH_PAGE_ADDR 0x800FC00



typedef struct
  {
    uint8_t Parameter1;    // 1 byte
    uint8_t Parameter2;    // 1 byte
    uint8_t Parameter3;    // 1 byte
    uint8_t Parameter4;    // 1 byte

  } tpSettings;
 
tpSettings settings;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

	// чтение и запись флэш
	//

	
	void FLASH_ReadSettings(void) {
		//Read settings
    uint32_t *source_addr = (uint32_t *)MY_FLASH_PAGE_ADDR;
    uint32_t *dest_addr = (void *)&settings;
		
		*dest_addr = *(__IO uint32_t*)source_addr;
	}
	
	void FLASH_WriteSettings(void) {
		uint32_t *source_addr = (void *)&settings;
		uint32_t *dest_addr = (uint32_t *) MY_FLASH_PAGE_ADDR;
		HAL_FLASH_Unlock();
		FLASH_PageErase(MY_FLASH_PAGE_ADDR);
		HAL_FLASH_Lock();
		
		HAL_FLASH_Unlock();
		HAL_FLASH_Program_IT(FLASH_TYPEPROGRAM_WORD, (uint32_t)dest_addr, *source_addr);
		HAL_FLASH_Program_IT(FLASH_TYPEPROGRAM_WORD, (uint32_t)dest_addr, *source_addr); // с первого раза не пишет как нужно...
		HAL_FLASH_Lock();

	}
	
	
	
	
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
	
	// значение настроек, сохраненные во флэше
	FLASH_ReadSettings();
	if (settings.Parameter1==255 && settings.Parameter2==255 && settings.Parameter3==255 && settings.Parameter4==255) // если включается первый раз, то записываем стандартные значения в параметры
	{
		settings.Parameter1 = x_scale;
		settings.Parameter2 = y_scale;
		settings.Parameter3 = rate_filter;
		settings.Parameter4 = 0;
		
		FLASH_WriteSettings();
	}
	x_scale = settings.Parameter1;
	y_scale = settings.Parameter2;
	rate_filter = settings.Parameter3;
	
	
	
	HAL_TIM_Base_Start_IT(&htim2);
	
	LCD_Init();
	LCD_setOrientation(ORIENTATION_LANDSCAPE_MIRROR);
	
	main:
	
	LCD_Fill(0x0000);
	
	line(0, 201, 319, 201, 0xFFFF);
	line(0, 0, 0, 199, 0xFFFF);
	STRING_OUT("Пульс", 15, 210, 1, 0xFFFF, 0x0000);
	
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
		if (menu==0) { // основное меню построяения графика
		while(2)
		{
		
		for (i=1; i<319; i++)
		{
			
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100);
		adc = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		adc = adc/y_scale; // (развертка по У)
		if (adc>200) {adc=200;} // чтобы не вылазило за экран графика
			
		//line(i+1, 0, i+1, 199, 0x0000);
		LCD_fillRect(i+1, 0, 2, 200, 0x0000); // закрашиваем отрисованный график перед его перерисовкой новым графиком
		line(i, 200-adc_last, i+1, 200-adc, b_color); // строим график линия между двумя соседними точками по оси Х, высота по У в зависимости от значения АЦП
		//line(0, 200-132, 5, 200-132, 0xFFFF);
		adc_last=adc;
			
		if (B_rate_mean < rate_min || B_rate_mean > rate_max) {b_color = COLOR(255, 0, 0);}  // визуализация цветом отклонения в количестве ударов
		else {b_color = COLOR(255, 255, 255);}
		
		HAL_Delay(x_scale); // (развертка по Х)
		
		STRING_NUM_L(B_rate_mean, 3, 125, 210, b_color, 0x0000);
		
		// детектор электродов
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)<1)
		{
			STRING_OUT("L+", 220, 210, 1, COLOR(0, 255, 0), 0x0000);
		}
		else {STRING_OUT("L+", 220, 210, 1, COLOR(255, 0, 0), 0x0000);}
		
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)<1)
		{
			STRING_OUT("L-", 260, 210, 1, COLOR(0, 255, 0), 0x0000);
		}
		else {STRING_OUT("L-", 260, 210, 1, COLOR(255, 0, 0), 0x0000);}
		//
		
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)<1) // переход в меню настроек
		{
			HAL_Delay(45);
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)<1)
			{
				menu++;
				LCD_Fill(0x0000);
				STRING_OUT("Настройки", 65, 10, 1, 0xFFFF, 0x0000);
				STRING_OUT("X шкала", 		10, 90, 1, 0xFFFF, 0x0000);
				STRING_OUT("Y шкала", 		10, 120, 1, 0xFFFF, 0x0000);
				STRING_OUT("Порог счета", 10, 150, 1, 0xFFFF, 0x0000);
				
				STRING_NUM_L(x_scale, 		3, 230, 90, 0xFFFF, 0x0000);
				STRING_NUM_L(y_scale, 		3, 230, 120, 0xFFFF, 0x0000);
				STRING_NUM_L(rate_filter, 3, 230, 150, 0xFFFF, 0x0000);
				goto settings;
			}
		}		
		
		
		
		}
		
		//STRING_NUM_L(B_rate, 3, 115, 210, 0xFFFF, 0x0000);
	}}
		
	

		
	// меню настроек
		while(2)
		{
			settings:
						
			
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)<1) 
		{
			HAL_Delay(150);
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)<1)
			{
				menu++;
				STRING_NUM_L(x_scale, 		3, 230, 90, 0xFFFF, 0x0000);
				STRING_NUM_L(y_scale, 		3, 230, 120, 0xFFFF, 0x0000);
				STRING_NUM_L(rate_filter, 3, 230, 150, 0xFFFF, 0x0000);
				
				if (menu>4)
				{
					if (save==1)
					{
						settings.Parameter1 = x_scale;
						settings.Parameter2 = y_scale;
						settings.Parameter3 = rate_filter;
						settings.Parameter4 = 0;
						FLASH_WriteSettings();
					}
					menu=0;
					goto main;
				}
			}
		}
		
		if (menu==1)
		{
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)<1)
			{
				HAL_Delay(90);
				if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)<1)
				{
					x_scale++;
				}}
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)<1)
			{
				HAL_Delay(90);
				if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)<1)
				{
					x_scale--;
				}}
			
			STRING_NUM_L(x_scale, 		3, 230, 90, 0x0000, 0xFFFF);
		}
		
		if (menu==2)
		{
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)<1)
			{
				HAL_Delay(90);
				if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)<1)
				{
					y_scale++;
				}}
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)<1)
			{
				HAL_Delay(90);
				if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)<1)
				{
					y_scale--;
				}}
			
			STRING_NUM_L(y_scale, 		3, 230, 120, 0x0000, 0xFFFF);
		}
		
		if (menu==3)
		{
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)<1)
			{
				HAL_Delay(90);
				if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)<1)
				{
					rate_filter++;
				}}
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)<1)
			{
				HAL_Delay(90);
				if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)<1)
				{
					rate_filter--;
				}}
			
			STRING_NUM_L(rate_filter, 3, 230, 150, 0x0000, 0xFFFF);
		}
			
				
		if (menu==4)
		{
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)<1)
			{
				HAL_Delay(90);
				if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)<1)
				{
					save=0;
					
				}}
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)<1)
			{
				HAL_Delay(90);
				if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)<1)
				{
					save=1;
					
				}}
			STRING_OUT("Сохранить?", 10, 180, 1, 0xFFFF, 0x0000);
			if (save==0) {STRING_OUT("Нет", 230, 180, 1, 0x0000, 0xFFFF);}
			if (save==1) {STRING_OUT("Да ", 230, 180, 1, 0x0000, 0xFFFF);}
			HAL_Delay(35);
			
		}
		
		
	}

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
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

    /**Initializes the CPU, AHB and APB busses clocks 
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Enables the Clock Security System 
    */
  HAL_RCC_EnableCSS();

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 4499;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 15;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
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

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif
