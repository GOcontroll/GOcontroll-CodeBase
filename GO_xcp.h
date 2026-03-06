/**************************************************************************************
 * \file   GO_xcp.h
 * \brief  XCP target-specific interface for GOcontroll hardware.
 *         Interfaces the XCP stack with the underlying transport (CAN, TCP, UDP).
 *
 *         Platform selection via preprocessor define:
 *           GOCONTROLL_IOT  →  STM32H5 (Moduline IOT): FDCAN via HAL + FreeRTOS
 *           (default)       →  Linux/IMX8 (Moduline IV / Moduline Mini): sockets
 *
 * \internal
 *----------------------------------------------------------------------------------------
 *                          C O P Y R I G H T
 *----------------------------------------------------------------------------------------
 * Copyright (c) 2019 by HAN Automotive http://www.han.nl All rights reserved
 * Copyright (c) 2025 by GOcontroll    http://www.gocontroll.com All rights reserved
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

/****************************************************************************************
 * General information about the XCP stack functionality
 ****************************************************************************************
 * This is a basic target-independent implementation of the XCP protocol.
 * It only takes care of the parameter upload and variable download functions.
 *
 * Required files:
 *   - XcpStack.c / XcpStack.h
 *   - GO_xcp.c / GO_xcp.h  (this file, replaces XcpTargetSpecific)
 *
 * Data layout:
 *   Incoming (master → slave):
 *     dataReceived[0]     = XCP command
 *     dataReceived[1..n]  = XCP data
 *
 *   Outgoing (slave → master):
 *     dataToSend[0]       = message length (excludes length and checksum byte)
 *     dataToSend[1]       = positive (0xFF) or negative (0xFE) reply
 *     dataToSend[2..n]    = XCP data
 *     dataToSend[n+1]     = checksum (if enabled)
 ****************************************************************************************/

#ifndef GO_XCP_H
#define GO_XCP_H

/****************************************************************************************
 * Include files
 ****************************************************************************************/
#include <stdint.h>

#include "XcpStack.h"

#ifdef GOCONTROLL_IOT
#include "GO_communication_can.h"
#include "cmsis_os2.h"
#include "stm32h5xx_hal.h"
#else
#include <stdio.h>
#endif

/****************************************************************************************
 * XCP memory allocation strategy
 ****************************************************************************************/

/** \brief Static XCP memory reservation (used when dynamic allocation is disabled) */
#define XCPSTATICMEMORY (2048)

#ifdef GOCONTROLL_IOT
/** \brief STM32H5: use FreeRTOS pvPortMalloc/pvPortFree */
#define DYNAMICMEMORYALLOCATION         (0)
#define DYNAMICMEMORYALLOCATIONFREERTOS (0)
#else
/** \brief Linux: use standard malloc/free */
#define DYNAMICMEMORYALLOCATION         (1)
#define DYNAMICMEMORYALLOCATIONFREERTOS (0)
#endif

#if DYNAMICMEMORYALLOCATION == 1 && DYNAMICMEMORYALLOCATIONFREERTOS == 1
#error Both DYNAMICMEMORYALLOCATION and DYNAMICMEMORYALLOCATIONFREERTOS are defined. Please define only one option.
#endif

/****************************************************************************************
 * Type definitions — platform-independent
 ****************************************************************************************/
typedef struct {
	char channel[7];
} _eventChannel;

/****************************************************************************************
 * Type definitions — Linux specific
 ****************************************************************************************/
#ifndef GOCONTROLL_IOT

typedef struct {
	char     *can_channel;
	uint32_t  xcp_send_id;
	uint32_t  xcp_receive_id;
} _XCP_CAN_Args;

#endif /* !GOCONTROLL_IOT */

/****************************************************************************************
 * Function prototypes — platform-independent
 ****************************************************************************************/

/**************************************************************************************
** \brief     Send the data from the XCP stack to the active transport bus.
** \param     data  pointer to the data array to transmit
** \return    0 on success, 1 on failure
***************************************************************************************/
uint8_t GOxcp_sendData(uint8_t *data);

/**************************************************************************************
** \brief     Read data from a memory location into the XCP data buffer.
** \param     data      destination buffer
** \param     elements  number of bytes to read (1, 2, 4 or 8)
** \param     location  source memory address
** \return    none
***************************************************************************************/
void GOxcp_readData(void *data, uint8_t elements, void *location);

/**************************************************************************************
** \brief     Write data from the XCP data buffer to a memory location.
** \param     data      source buffer
** \param     elements  number of bytes to write (1, 2, 4 or 8)
** \param     location  destination memory address
** \return    none
***************************************************************************************/
void GOxcp_writeData(void *data, uint8_t elements, void *location);

/**************************************************************************************
** \brief     Stop the active XCP connection and release resources.
** \return    none
***************************************************************************************/
void GOxcp_stopConnection(void);

/**************************************************************************************
** \brief     Handle a user-defined XCP command (transport-specific behaviour).
** \param     dataReceived  pointer to the received XCP command bytes
** \return    0 on success, -1 on unrecognised command
***************************************************************************************/
uint8_t GOxcp_userCmd(uint8_t *dataReceived);

/****************************************************************************************
 * Function prototypes — STM32H5 (GOCONTROLL_IOT) specific
 ****************************************************************************************/
#ifdef GOCONTROLL_IOT

/**************************************************************************************
** \brief     Configure the FDCAN peripheral for XCP and set up the receive filter.
** \param     can_channel             FDCAN handle to use
** \param     xcp_send_id             CAN ID for outgoing XCP frames
** \param     xcp_receive_id          CAN ID for incoming XCP frames
** \param     xcp_send_id_extended    1 = 29-bit extended ID, 0 = 11-bit standard
** \param     xcp_receive_id_extended 1 = 29-bit extended ID, 0 = 11-bit standard
** \return    none
***************************************************************************************/
void GOxcp_initCan(FDCAN_HandleTypeDef *can_channel,
				   uint32_t xcp_send_id, uint32_t xcp_receive_id,
				   uint8_t xcp_send_id_extended,
				   uint8_t xcp_receive_id_extended);

/**************************************************************************************
** \brief     FreeRTOS task that processes incoming XCP CAN frames.
** \param     args  unused task argument
** \return    none (infinite loop)
***************************************************************************************/
void GOxcp_threadCan(void *args);

/****************************************************************************************
 * Function prototypes — Linux specific
 ****************************************************************************************/
#else

/**************************************************************************************
** \brief     Start the XCP slave over TCP/IP (blocking — runs in a thread).
** \param     aArgument  unused
** \return    none
***************************************************************************************/
void *GOxcp_initializeTcp(void *aArgument);

/**************************************************************************************
** \brief     Start the XCP slave over UDP/IP (blocking — runs in a thread).
** \param     aArgument  unused
** \return    none
***************************************************************************************/
void *GOxcp_initializeUdp(void *aArgument);

/**************************************************************************************
** \brief     Start the XCP slave over CAN (blocking — runs in a thread).
** \param     aArgument  pointer to _XCP_CAN_Args
** \return    none
***************************************************************************************/
void *GOxcp_initializeCan(void *aArgument);

#endif /* GOCONTROLL_IOT */

#endif /* GO_XCP_H */

/* end of GO_xcp.h */
