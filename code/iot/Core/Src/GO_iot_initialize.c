/**************************************************************************************
** \file      GO_iot_initialize.c
** \brief     GOcontroll S1 controller platform initialization.
** \internal
***************************************************************************************
** C O P Y R I G H T
***************************************************************************************
** Copyright (c) 2026 by GOcontroll      http://www.GOcontroll.com
** All rights reserved
***************************************************************************************
** L I C E N S E
***************************************************************************************
** Permission is hereby granted, free of charge, to any person obtaining a copy of
** this software and associated documentation files (the "Software"), to deal in the
** Software without restriction, including without limitation the rights to use,
** copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
** Software, and to permit persons to whom the Software is furnished to do so,
** subject to the following conditions:
**
** The above copyright notice and this permission notice shall be included in all
** copies or substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
** AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
** WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
** CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
** \endinternal
****************************************************************************************/

#include "GO_iot_initialize.h"
#include "adc.h"
#include "fdcan.h"
#include "gpio.h"
#include "icache.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "SEGGER_RTT.h"
#include "GO_communication_esp.h"
#include "GO_board.h"


/****************************************************************************************
** \brief  Initialize the GOcontroll S1 controller platform.
****************************************************************************************/
void GO_iot_initialize(void)
{
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	/* Set keep alive pin high as soon as possible */
	HAL_GPIO_WritePin(KL15_CONTROLLER_UCO_GPIO_Port, KL15_CONTROLLER_UCO_Pin,
					  GPIO_PIN_SET);
	MX_ICACHE_Init();
	MX_SPI1_Init();
	MX_USART2_UART_Init();
	MX_ADC1_Init();
	//MX_I2C1_SMBUS_Init();
	//MX_I2C3_Init();
	MX_TIM3_Init();
	MX_FDCAN1_Init();
	MX_FDCAN2_Init();
	HAL_PWR_EnableBkUpAccess();
	if (READ_REG(TAMP->BKP0R) != 0xBEEFU) {
		MX_RTC_Init();
		WRITE_REG(TAMP->BKP0R, 0xBEEFU);
	}

	SEGGER_RTT_Init();
	GO_communication_esp_init(&huart2);

	/* Initialize the FreeRTOS kernel */
	osKernelInitialize();

	GO_board_controller_info_task_start();
}


/****************************************************************************************
** \brief  Configure the system clock (250 MHz from 12 MHz HSE via PLL1).
****************************************************************************************/
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

	/** Initializes the RCC Oscillators according to the specified parameters
	 *  in the RCC_OscInitTypeDef structure. */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
	RCC_OscInitStruct.LSIState       = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource  = RCC_PLL1_SOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM       = 6;
	RCC_OscInitStruct.PLL.PLLN       = 250;
	RCC_OscInitStruct.PLL.PLLP       = 2;
	RCC_OscInitStruct.PLL.PLLQ       = 2;
	RCC_OscInitStruct.PLL.PLLR       = 2;
	RCC_OscInitStruct.PLL.PLLRGE     = RCC_PLL1_VCIRANGE_1;
	RCC_OscInitStruct.PLL.PLLVCOSEL  = RCC_PLL1_VCORANGE_WIDE;
	RCC_OscInitStruct.PLL.PLLFRACN   = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks */
	RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK
	                                 | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
	                                 | RCC_CLOCKTYPE_PCLK3;
	RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}

	/** Configure the programming delay */
	__HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
}


/**
 * @brief  Period elapsed callback in non blocking mode.
 * @note   Called when TIM17 interrupt fires inside HAL_TIM_IRQHandler().
 *         Makes a direct call to HAL_IncTick() to increment the application time base.
 * @param  htim TIM handle
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM17) {
		HAL_IncTick();
	}
}


/**
 * @brief  This function is executed in case of error occurrence.
 */
void Error_Handler(void)
{
	__disable_irq();
	while (1) {}
}


#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
