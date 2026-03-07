/**************************************************************************************
 * \file   GO_controller_info.h
 * \brief  Controller runtime information interface for GOcontroll targets.
 *         Covers model/XCP task stack high-water marks, free heap and CPU load.
 *         Compile with -DGOCONTROLL_IOT for STM32-based IoT targets.
 *         On Linux, all functions return 0.
 * \internal
 *----------------------------------------------------------------------------------------
 *                          C O P Y R I G H T
 *----------------------------------------------------------------------------------------
 * Copyright 2025 (c)  by GOcontroll http://www.gocontroll.com All rights reserved
 *
 *----------------------------------------------------------------------------------------
 *                            L I C E N S E
 *----------------------------------------------------------------------------------------
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * \endinternal
 ****************************************************************************************/
#pragma once

#include <stdint.h>

/****************************************************************************************
 * Module information
 ****************************************************************************************/

typedef struct {
	uint32_t article_number; /* e.g. 20100105 reconstructed from 4 bytes */
	uint8_t  sw_major;
	uint8_t  sw_minor;
	uint8_t  sw_patch;
} _moduleInfo;

/**************************************************************************************
** \brief     Get hardware and software version of a registered module.
** \param     slot  Module slot index (0-based).
** \param     info  Pointer to _moduleInfo struct to fill.
** \return    0 on success, -1 if slot is out of range or not registered.
***************************************************************************************/
int GO_controller_info_get_module_info(uint8_t slot, _moduleInfo *info);

/****************************************************************************************
 * Model version
 ****************************************************************************************/

typedef struct {
	uint8_t major;
	uint8_t minor;
	uint8_t patch;
} _modelVersion;

/**************************************************************************************
** \brief     Store the model software version (called from TLC-generated Start code).
** \param     major  Major version number.
** \param     minor  Minor version number.
** \param     patch  Patch version number.
***************************************************************************************/
void GO_controller_info_set_model_version(uint8_t major, uint8_t minor, uint8_t patch);

/**************************************************************************************
** \brief     Retrieve the model software version.
** \param     ver  Pointer to _modelVersion struct to fill.
** \return    Always 0.
***************************************************************************************/
int GO_controller_info_get_model_version(_modelVersion *ver);

/****************************************************************************************
 * RTC time
 ****************************************************************************************/

typedef struct {
	uint16_t year;
	uint8_t  month;
	uint8_t  day;
	uint8_t  hour;
	uint8_t  minute;
	uint8_t  second;
} _rtcTime;

/**************************************************************************************
** \brief     Read the current RTC date and time.
** \param     time  Pointer to _rtcTime struct to fill.
***************************************************************************************/
void GO_controller_info_get_rtc_time(_rtcTime *time);

/**************************************************************************************
** \brief     Return the minimum free stack ever recorded for the model step task
**            (high-water mark). Call at low frequency — scans the full stack.
** \return    Remaining stack in bytes (IoT only; returns 0 on Linux).
***************************************************************************************/
uint32_t GO_controller_info_get_model_stack(void);

/**************************************************************************************
** \brief     Return the minimum free stack ever recorded for the XCP thread
**            (high-water mark). Call at low frequency — scans the full stack.
** \return    Remaining stack in bytes (IoT only; returns 0 on Linux).
***************************************************************************************/
uint32_t GO_controller_info_get_xcp_stack(void);

/**************************************************************************************
** \brief     Return the current FreeRTOS free heap size.
**            Updated every ~1 s by ControllerInfoTask in GO_board.c.
** \return    Free heap in bytes (IoT only; returns 0 on Linux).
***************************************************************************************/
uint32_t GO_controller_info_get_free_heap(void);

/**************************************************************************************
** \brief     Return the CPU load as a percentage (0-100).
**            Computed as 100% minus the FreeRTOS IDLE task share over the last ~1 s.
**            Updated every ~1 s by ControllerInfoTask in GO_board.c.
** \return    CPU load percentage (IoT only; returns 0 on Linux).
***************************************************************************************/
uint8_t GO_controller_info_get_cpu_load(void);
