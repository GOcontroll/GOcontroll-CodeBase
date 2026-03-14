/************************************************************************************//**
* \file         GO_communication_can.h
* \brief        Unified CAN communication driver for GOcontroll platforms.
*               Linux: SocketCAN (PF_CAN / SOCK_RAW).
*               IoT  : STM32 FDCAN via STM32 HAL.
* \internal
*----------------------------------------------------------------------------------------
*                          C O P Y R I G H T
*----------------------------------------------------------------------------------------
*  Copyright 2024 (c) by GOcontroll      http://www.gocontroll.com   All rights reserved
*
*----------------------------------------------------------------------------------------
*                            L I C E N S E
*----------------------------------------------------------------------------------------
* Permission is hereby granted, free of charge, to any person obtaining a copy of this
* software and associated documentation files (the "Software"), to deal in the Software
* without restriction, including without limitation the rights to use, copy, modify, merge,
* publish, distribute, sublicense, and/or sell copies of the Software, and to permit
* persons to whom the Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all copies or
* substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
* OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
* DEALINGS IN THE SOFTWARE.
* \endinternal
****************************************************************************************/

#ifndef GO_COMMUNICATION_CAN_H
#define GO_COMMUNICATION_CAN_H

/****************************************************************************************
* IoT platform (STM32 FDCAN via HAL)
****************************************************************************************/
#ifdef GOCONTROLL_IOT

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "stm32h5xx_hal.h"
#include "print.h"

/* Baudrate index passed to GO_communication_can_initialize() */
#define CAN125KBPS 0
#define CAN250KBPS 1
#define CAN500KBPS 2
#define CAN1MBPS   3

/* Bit masks used to pack DLC, ExtId and RTR into can_frame.flags */
#define CAN_PACKED_DLC   0b001111
#define CAN_PACKED_EXTID 0b010000
#define CAN_PACKED_RTR   0b100000

/** Compact CAN frame representation used internally by the IoT driver */
struct can_frame {
	uint8_t  data[8];
	uint32_t id;
	uint8_t  flags;  /* DLC (bits 0-3), ExtId (bit 4), RTR (bit 5) */
};

/**************************************************************************************
** \brief     Initialise an FDCAN peripheral (HAL init + global filter). Does NOT
**            start the peripheral; call GO_communication_can_start() afterwards once
**            all callbacks are registered.
** \param     hfdcan   Pointer to the FDCAN handle (Instance must be set by caller).
** \param     baudrate Baudrate index (CAN125KBPS ... CAN1MBPS).
** \param     autort   ENABLE / DISABLE automatic retransmission.
** \return    0 on success, -1 on error.
***************************************************************************************/
int GO_communication_can_initialize(FDCAN_HandleTypeDef *hfdcan, uint32_t baudrate,
                            FunctionalState autort);

/**************************************************************************************
** \brief     Start the FDCAN peripheral and take the transceiver out of silent mode.
**            Call this AFTER all HAL callbacks are registered.
** \param     hfdcan   Pointer to the FDCAN handle (must be initialized with
**                     GO_communication_can_initialize).
** \return    0 on success, -1 on error.
***************************************************************************************/
int GO_communication_can_start(FDCAN_HandleTypeDef *hfdcan);

/**************************************************************************************
** \brief     Pack an FDCAN RX header into the compact can_frame representation.
** \param     frame   Output frame.
** \param     header  FDCAN RX header from HAL_FDCAN_GetRxMessage().
** \return    none
***************************************************************************************/
void GO_communication_can_pack_header(struct can_frame *frame,
                                   FDCAN_RxHeaderTypeDef *header);

/**************************************************************************************
** \brief     Extract the DLC byte count from a packed can_frame.
** \param     frame  Pointer to the packed CAN frame.
** \return    DLC byte count (0-8).
***************************************************************************************/
uint8_t GO_communication_can_packed_dlc(struct can_frame *frame);

/**************************************************************************************
** \brief     Return true when the packed frame carries a 29-bit extended identifier.
** \param     frame  Pointer to the packed CAN frame.
** \return    true if the frame uses an extended (29-bit) identifier, false otherwise.
***************************************************************************************/
bool GO_communication_can_packed_is_ext_id(struct can_frame *frame);

/**************************************************************************************
** \brief     Return true when the packed frame is a Remote Transmission Request.
** \param     frame  Pointer to the packed CAN frame.
** \return    true if the frame is an RTR frame, false otherwise.
***************************************************************************************/
bool GO_communication_can_packed_is_rtr(struct can_frame *frame);

/**************************************************************************************
** \brief     Handle FDCAN error-status interrupt: on bus-off, clears CCCR.INIT to
**            trigger the automatic 128 x 11 recessive-bit recovery sequence.
**            Safe to call from the registered HAL ErrorStatusCallback (ISR context).
** \param     hfdcan         FDCAN handle that raised the error interrupt.
** \param     ErrorStatusITs Error status interrupt flags passed by the HAL callback.
** \return    none
***************************************************************************************/
void GO_communication_can_bus_off_recovery(FDCAN_HandleTypeDef *hfdcan,
                                       uint32_t ErrorStatusITs);

/*==============================================================================================
** Short-name public API — used by generated Simulink code.
** These wrap the GO_communication_can_* functions above and add bitrate bookkeeping.
==============================================================================================*/

int     init_can(FDCAN_HandleTypeDef *hfdcan, uint32_t baudrate, FunctionalState autort);
int     start_can(FDCAN_HandleTypeDef *hfdcan);
void    can_pack_header(struct can_frame *frame, FDCAN_RxHeaderTypeDef *header);
uint8_t can_packed_dlc(struct can_frame *frame);
bool    can_packed_is_ExtId(struct can_frame *frame);
bool    can_packed_is_RTR(struct can_frame *frame);
void    can_bus_off_recovery(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs);

/**************************************************************************************
** \brief     Return the protocol bitrate code for the given CAN channel.
**            Stored by init_can() when the channel is initialised.
** \param     channel  1 = CAN1, 2 = CAN2.
** \return    1=125kbps, 2=250kbps, 3=500kbps, 4=1Mbps, 0=channel not initialised.
***************************************************************************************/
uint8_t can_get_esp_bitrate(uint8_t channel);

#endif /* GOCONTROLL_IOT */


/****************************************************************************************
* Linux platform (SocketCAN)
****************************************************************************************/
#ifdef GOCONTROLL_LINUX

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <netinet/in.h>
#include <netdb.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <fcntl.h>
#include <errno.h>

#ifndef CANBUFSIZE
#define CANBUFSIZE 10
#endif

/** Struct to hold a buffered CAN message */
struct CANbuffer_t {
	canid_t can_id;
	bool newFlag;
	struct can_frame frame;
};

typedef struct {
	uint8_t  socketCreated[4];
	uint16_t initErrorCounter[4];
	int      socket[4];
} _canConnection;

/**************************************************************************************
** \brief     Creates a CAN socket to be able to read and write CAN messages from.
** \param     canInterface  Index of the CAN bus (0-3).
** \return    none
***************************************************************************************/
void GO_communication_can_socket(uint8_t canInterface);

/**************************************************************************************
** \brief     Gets a CAN message from the receive buffer.
** \param     can_id        Identifier of the CAN message to retrieve.
** \param     canInterface  CAN bus index.
** \param     frame         Output pointer to the stored can_frame.
** \param     newFlag       Output flag indicating whether the message is new.
** \return    0 on success, -1 on error.
***************************************************************************************/
int GO_communication_can_get_buffer(canid_t can_id, uint8_t canInterface,
                                 struct can_frame **frame, bool *newFlag);

/**************************************************************************************
** \brief     Update the SocketCAN receive filter for a given ID slot.
** \param     oldCANid      Identifier to replace.
** \param     canInterface  CAN bus index.
** \param     newCANid      New identifier.
** \return    0 on success, -1 on error.
***************************************************************************************/
int GO_communication_can_update_receive_filter(canid_t oldCANid, uint8_t canInterface,
                                           canid_t newCANid);

/**************************************************************************************
** \brief     Update the receive buffer slot for a given ID.
** \param     oldCANid      Identifier to replace.
** \param     canInterface  CAN bus index.
** \param     newCANid      New identifier.
** \return    0 on success, -1 on error.
***************************************************************************************/
int GO_communication_can_update_receive_buffer(canid_t oldCANid, uint8_t canInterface,
                                           canid_t newCANid);

/**************************************************************************************
** \brief     Drain all open SocketCAN sockets and copy received frames into the buffer.
** \return    0 on success, -1 on error.
***************************************************************************************/
int GO_communication_can_serve_connection(void);

#endif /* GOCONTROLL_LINUX */

#endif /* GO_COMMUNICATION_CAN_H */
