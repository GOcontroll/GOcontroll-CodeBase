/**************************************************************************************
 * \file   GO_gps.h
 * \brief  Platform-agnostic GPS interface for GOcontroll targets.
 *         Select the target implementation at compile time via the
 *         GOCONTROLL_LINUX or GOCONTROLL_IOT preprocessor macro.
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

#ifdef __cplusplus
extern "C" {
#endif

struct gps_data {
	float    longitude;
	float    latitude;
	float    altitude;
	float    velocity;
	uint16_t year;
	uint8_t  month;
	uint8_t  day;
	uint8_t  hour;
	uint8_t  minute;
	uint8_t  second;
	uint8_t  valid;  /**< 1 = valid GPS fix, 0 = no fix */
};

/**************************************************************************************
** \brief     Initialize the GPS subsystem (open UART, create mutex/thread).
** \return    none
***************************************************************************************/
void GO_gps_initialize(void);

/**************************************************************************************
** \brief     Read the latest GPS fix into the caller-supplied structure.
** \param     out  pointer to a gps_data structure to populate
** \return    none
***************************************************************************************/
void GO_gps_read(struct gps_data *out);

/**************************************************************************************
** \brief     Terminate the GPS subsystem and release all resources.
** \return    none
***************************************************************************************/
void GO_gps_terminate(void);

#ifdef __cplusplus
}
#endif

