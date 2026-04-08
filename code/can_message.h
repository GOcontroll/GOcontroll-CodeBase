/**************************************************************************************
 * \file   can_message.h
 * \brief  Platform-independent CAN message type used by the GOcontroll blockset.
 *         Matches the Simulink CAN_DATATYPE convention so generated code from the
 *         CAN send/receive blocks can target both Linux (SocketCAN) and IoT (FDCAN).
 *
 * \internal
 *----------------------------------------------------------------------------------------
 *                          C O P Y R I G H T
 *----------------------------------------------------------------------------------------
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

#ifndef CAN_MESSAGE_H
#define CAN_MESSAGE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************
 * \brief CAN message as exchanged between Simulink blocks and the underlying driver.
 *
 * Fields:
 *   ID       — 11-bit (standard) or 29-bit (extended) CAN identifier
 *   Extended — 1 = 29-bit extended ID, 0 = 11-bit standard ID
 *   Remote   — 1 = Remote Transmission Request frame, 0 = data frame
 *   Length   — number of data bytes (0–8)
 *   Data     — payload bytes (unused bytes beyond Length are ignored)
 **************************************************************************************/
typedef struct {
	uint32_t ID;
	uint8_t  Extended;
	uint8_t  Remote;
	uint8_t  Length;
	uint8_t  Data[8];
} CAN_DATATYPE;

#ifdef __cplusplus
}
#endif

#endif /* CAN_MESSAGE_H */

/* end of can_message.h */
