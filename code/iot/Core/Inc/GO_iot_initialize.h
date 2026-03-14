/**************************************************************************************
** \file      GO_iot_initialize.h
** \brief     GOcontroll IoT controller platform initialization.
**            Contains all hardware and middleware initialization required to run the
**            GOcontroll IoT controller. Call GO_iot_initialize() once at startup,
**            before any model or application code is executed.
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

#ifndef GO_IOT_INITIALIZE_H
#define GO_IOT_INITIALIZE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "cmsis_os2.h"

/****************************************************************************************
** \brief  Initialize the GOcontroll IoT controller platform.
**         Performs HAL init, clock configuration, all peripheral inits (GPIO, SPI,
**         UART, ADC, TIM, FDCAN, RTC), SEGGER RTT, ESP communication, FreeRTOS
**         kernel, and the controller info task.
**         Call this once at the start of main(), before any application or model code.
****************************************************************************************/
void GO_iot_initialize(void);

/****************************************************************************************
** \brief  Configure the system clock (250 MHz from 12 MHz HSE via PLL1).
****************************************************************************************/
void SystemClock_Config(void);

#ifdef __cplusplus
}
#endif

#endif /* GO_IOT_INITIALIZE_H */
