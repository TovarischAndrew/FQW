/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"

/* USER CODE BEGIN 0 */

#include "stdbool.h" 
#include "SPI_TFT.h"

extern uint16_t B_rate; // количество ударов в минуту 
extern uint16_t adc; // значение ацп, передаваемое из main сюда в прерывание
extern uint16_t B_rate_mean; // усредненное количество ударов в минуту
extern uint8_t menu;
extern uint8_t rate_filter;

extern uint16_t b_color; // цвет графика

volatile uint16_t P=0; // флаг, 0 - КГ нижний уровень, 1 - КГ высокий уровень
volatile uint32_t sampleCounter = 0; // счетчик миллисекунд (по прерыванию) для подсчета времени между ударами сердца

volatile uint16_t Signal = 0; // в эту переменную принимаем значение ацп из main

// нужно использовать для определения порога низкого и высокого уровня (max - min)*0,6~0,7
// или использовать какое-то фиксированное значение, например 132 = (4095/20)*0,65 = 204*0,65
volatile int min = 0;
volatile int max = 0; 

volatile uint16_t b_buf[5];
volatile uint8_t ii=0;





/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  HAL_RCC_NMI_IRQHandler();
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Prefetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles Flash global interrupt.
*/
void FLASH_IRQHandler(void)
{
  /* USER CODE BEGIN FLASH_IRQn 0 */

  /* USER CODE END FLASH_IRQn 0 */
  HAL_FLASH_IRQHandler();
  /* USER CODE BEGIN FLASH_IRQn 1 */

  /* USER CODE END FLASH_IRQn 1 */
}

/**
* @brief This function handles RCC global interrupt.
*/
void RCC_IRQHandler(void)
{
  /* USER CODE BEGIN RCC_IRQn 0 */

  /* USER CODE END RCC_IRQn 0 */
  /* USER CODE BEGIN RCC_IRQn 1 */

  /* USER CODE END RCC_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel3 global interrupt.
*/
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
* @brief This function handles ADC1 and ADC2 global interrupts.
*/
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
	
	
	// опрос кнопок
//	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)<1) 
//	{
//	
//	}
//		
//	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)<1) 
//	{
//	
//	}
//			
//	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)<1) 
//	{
//	
//	}
//				
//	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)<1) 
//	{
//	
//	}
		
	
	
	
		if (menu==0) // все считается если экран в меню отрисовки графика
		{
			
		Signal = adc;
	
		sampleCounter ++; 
	
		if (Signal > rate_filter && P==0) //(порог отсчета ударов сердца)
		{
			P=1;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			B_rate = 60000/sampleCounter;
			sampleCounter=0;
			
			b_buf[ii] = B_rate;
			ii++;
			if (ii>4) {ii=0;}
			B_rate_mean = (b_buf[0]+b_buf[1]+b_buf[2]+b_buf[3]+b_buf[4])/5;
			
			
			
		}
		
		if (Signal <= rate_filter && P==1) //(порог отсчета ударов сердца)
		{
			P=0;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		}
		
		if (sampleCounter > 5000)
		{
			B_rate = 1;
			sampleCounter=0;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			B_rate_mean=0;
			//STRING_OUT("000", 125, 210, 1, b_color, 0x0000);
		}
		
	}

  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles SPI1 global interrupt.
*/
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */

  /* USER CODE END SPI1_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi1);
  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
