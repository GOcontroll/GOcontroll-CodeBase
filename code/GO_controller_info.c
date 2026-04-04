/**************************************************************************************
 * \file   GO_controller_info.c
 * \brief  Controller runtime information for GOcontroll targets.
 *         Provides model/XCP task stack high-water marks, free heap and CPU load
 *         (IoT only; Linux stubs return 0).
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

#include "GO_controller_info.h"
#include "GO_board.h"
#include <stddef.h>
#include <string.h>

extern _hardwareConfig hardwareConfig;

/****************************************************************************************
 * Platform-independent implementation
 ****************************************************************************************/

/**************************************************************************************
** \brief     Get hardware and software version of a registered module.
** \param     slot  Module slot index (0-based).
** \param     info  Pointer to _moduleInfo struct to fill.
** \return    0 on success, -1 if slot is out of range or not registered.
***************************************************************************************/
static _modelVersion model_version = {0, 0, 0};

void GO_controller_info_set_model_version(uint8_t major, uint8_t minor, uint8_t patch) {
	model_version.major = major;
	model_version.minor = minor;
	model_version.patch = patch;
}

int GO_controller_info_get_model_version(_modelVersion *ver) {
	*ver = model_version;
	return 0;
}

/****************************************************************************************/

static _appConfig app_config = {
	.app_id           = "GOCO",
	.signing_enabled  = 0u,
	.public_key       = {0},
	.distribution_url = "https://deploy.gocontroll.com",
};

void GO_controller_info_set_app_config(const char *app_id, uint8_t signing,
                                        const uint8_t *public_key, const char *url) {
	if (app_id != NULL) {
		strncpy(app_config.app_id, app_id, 4u);
		app_config.app_id[4] = '\0';
	}
	app_config.signing_enabled = signing;
	if (public_key != NULL) {
		memcpy(app_config.public_key, public_key, 32u);
	} else {
		memset(app_config.public_key, 0, 32u);
	}
	if (url != NULL) {
		strncpy(app_config.distribution_url, url, 256u);
		app_config.distribution_url[256] = '\0';
	}
}

const _appConfig *GO_controller_info_get_app_config(void) {
	return &app_config;
}

/****************************************************************************************/

int GO_controller_info_get_module_info(uint8_t slot, _moduleInfo *info) {
	if (slot >= 8 || hardwareConfig.moduleOccupancy[slot][0] == 0)
		return -1;
	info->article_number =
		(uint32_t)hardwareConfig.moduleOccupancy[slot][0] * 1000000U +
		(uint32_t)hardwareConfig.moduleOccupancy[slot][1] * 10000U +
		(uint32_t)hardwareConfig.moduleOccupancy[slot][2] * 100U +
		(uint32_t)hardwareConfig.moduleOccupancy[slot][3];
	info->sw_major = hardwareConfig.moduleOccupancy[slot][4];
	info->sw_minor = hardwareConfig.moduleOccupancy[slot][5];
	info->sw_patch = hardwareConfig.moduleOccupancy[slot][6];
	return 0;
}

/****************************************************************************************
 * IoT implementation — reads high-water marks updated by ControllerInfoTask in GO_board.c
 ****************************************************************************************/
#ifdef GOCONTROLL_IOT

#include "rtc.h"
extern RTC_HandleTypeDef hrtc;

void GO_controller_info_get_rtc_time(_rtcTime *t) {
	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN); /* Must follow GetTime (shadow register) */
	t->year   = (uint16_t)(sDate.Year + 2000u);
	t->month  = sDate.Month;
	t->day    = sDate.Date;
	t->hour   = sTime.Hours;
	t->minute = sTime.Minutes;
	t->second = sTime.Seconds;
}

#elif defined(GOCONTROLL_LINUX)

void GO_controller_info_get_rtc_time(_rtcTime *t) {
	t->year = 0; t->month = 0; t->day = 0;
	t->hour = 0; t->minute = 0; t->second = 0;
}

#endif /* GOCONTROLL_IOT / GOCONTROLL_LINUX (RTC) */

#ifdef GOCONTROLL_IOT

extern uint32_t go_board_model_stack_hwm;
extern uint32_t go_board_xcp_stack_hwm;
extern uint32_t go_board_free_heap;
extern uint8_t  go_board_cpu_load;

/**************************************************************************************
** \brief     Return the minimum free stack ever recorded for the model step task
**            (high-water mark). Call at low frequency — scans the full stack.
** \return    Remaining stack in bytes (IoT only; returns 0 on Linux).
***************************************************************************************/
uint32_t GO_controller_info_get_model_stack(void) {
	return go_board_model_stack_hwm;
}

/**************************************************************************************
** \brief     Return the minimum free stack ever recorded for the XCP thread
**            (high-water mark). Call at low frequency — scans the full stack.
** \return    Remaining stack in bytes (IoT only; returns 0 on Linux).
***************************************************************************************/
uint32_t GO_controller_info_get_xcp_stack(void) {
	return go_board_xcp_stack_hwm;
}

/**************************************************************************************
** \brief     Return the current FreeRTOS free heap size.
** \return    Free heap in bytes (IoT only; returns 0 on Linux).
***************************************************************************************/
uint32_t GO_controller_info_get_free_heap(void) {
	return go_board_free_heap;
}

/**************************************************************************************
** \brief     Return the CPU load as a percentage (0-100).
** \return    CPU load percentage (IoT only; returns 0 on Linux).
***************************************************************************************/
uint8_t GO_controller_info_get_cpu_load(void) {
	return go_board_cpu_load;
}

/****************************************************************************************
 * Linux implementation — not available on Linux; return 0
 ****************************************************************************************/
#elif defined(GOCONTROLL_LINUX)

/**************************************************************************************
** \brief     Return the minimum free stack ever recorded for the model step task.
** \return    Remaining stack in bytes (IoT only; returns 0 on Linux).
***************************************************************************************/
uint32_t GO_controller_info_get_model_stack(void) { return 0; }

/**************************************************************************************
** \brief     Return the minimum free stack ever recorded for the XCP thread.
** \return    Remaining stack in bytes (IoT only; returns 0 on Linux).
***************************************************************************************/
uint32_t GO_controller_info_get_xcp_stack(void)   { return 0; }

/**************************************************************************************
** \brief     Return the current free heap size.
** \return    Free heap in bytes (IoT only; returns 0 on Linux).
***************************************************************************************/
uint32_t GO_controller_info_get_free_heap(void)   { return 0; }

/**************************************************************************************
** \brief     Return the CPU load percentage.
** \return    CPU load percentage (IoT only; returns 0 on Linux).
***************************************************************************************/
uint8_t GO_controller_info_get_cpu_load(void)     { return 0; }

#endif /* GOCONTROLL_IOT / GOCONTROLL_LINUX */
