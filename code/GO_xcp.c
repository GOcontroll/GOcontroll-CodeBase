/**************************************************************************************
 * \file   GO_xcp.c
 * \brief  XCP target-specific implementation for GOcontroll hardware.
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
 * Include files — common
 ****************************************************************************************/
#include "GO_xcp.h"

#include <stdint.h>
#include <string.h>

#include "print.h"

/****************************************************************************************
 * Include files — platform-specific
 ****************************************************************************************/
#ifdef GOCONTROLL_IOT

#include "Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.h"
#include "cmsis_os2.h"
#elif defined(GOCONTROLL_LINUX)

#include <arpa/inet.h>
#include <errno.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/in.h>
#include <net/if.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>



#include "SYS_config.h"

#endif /* GOCONTROLL_IOT / GOCONTROLL_LINUX */

/****************************************************************************************
 * Macro definitions
 ****************************************************************************************/
#define XCPMAXDTOLENGTH 255
#define XCPMAXCTOLENGTH 255
#define TIMEOUT         1

/****************************************************************************************
 * Data declarations — platform-independent
 ****************************************************************************************/

// TODO channels must be assigned during timer creation.
_eventChannel eventChannel[3] = {
	{"EvChnl1"},
	{"EvChnl2"},
	{"EvChnl3"},
};

/****************************************************************************************
 ****************************************************************************************
 * Platform-independent implementations
 ****************************************************************************************
 ****************************************************************************************/

/**************************************************************************************
** \brief     Read data from a memory location into the XCP data buffer.
** \param     data      destination buffer
** \param     elements  number of bytes to read (1, 2, 4 or 8)
** \param     location  source memory address
** \return    none
***************************************************************************************/
void GO_xcp_read_data(void *data, uint8_t elements, void *location) {
	switch (elements) {
		case 1:
			*(uint8_t *)data = *(uint8_t *)location;
			break;
		case 2:
			*(uint16_t *)data = *(uint16_t *)location;
			break;
		case 4:
			*(uint32_t *)data = *(uint32_t *)location;
			break;
		case 8:
			*(uint64_t *)data = *(uint64_t *)location;
			break;
	}
}

/**************************************************************************************
** \brief     Write data from the XCP data buffer to a memory location.
** \param     data      source buffer
** \param     elements  number of bytes to write (1, 2, 4 or 8)
** \param     location  destination memory address
** \return    none
***************************************************************************************/
void GO_xcp_write_data(void *data, uint8_t elements, void *location) {
	// TODO check for write-protected memory areas
	switch (elements) {
		case 1:
			*(uint8_t *)location = *(uint8_t *)data;
			break;
		case 2:
			*(uint16_t *)location = *(uint16_t *)data;
			break;
		case 4:
			*(uint32_t *)location = *(uint32_t *)data;
			break;
		case 8:
			*(uint64_t *)location = *(uint64_t *)data;
			break;
	}
}

/****************************************************************************************
 ****************************************************************************************
 * STM32H5 (GOCONTROLL_IOT) specific implementations
 ****************************************************************************************
 ****************************************************************************************/
#ifdef GOCONTROLL_IOT


/****************************************************************************************
 * Data declarations
 ****************************************************************************************/
static uint8_t				dataToSend[16] = {0};
static uint32_t				xcpDtoId;
static uint8_t				xcpDtoIdExt;
void					   *XcpConnection_fd;
static uint8_t				xcpTransmissionBus = 0;

osMessageQueueId_t xcp_received;

/****************************************************************************************/

static uint8_t XcpCanSend(uint8_t *data);

/****************************************************************************************/
// TODO Channels must be assigned during timer creation.


void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {
	if (RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) {
		FDCAN_RxHeaderTypeDef header;
		struct can_frame message;
		while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &header, message.data) ==
			   HAL_OK) {
			GO_communication_can_pack_header(&message, &header);
			osMessageQueuePut(xcp_received, &message, 0, 0);
		}
	}
}

/****************************************************************************************/

/**************************************************************************************
** \brief     Configure the FDCAN peripheral for XCP and set up the receive filter.
** \param     can_channel             FDCAN handle to use
** \param     xcp_send_id             CAN ID for outgoing XCP frames
** \param     xcp_receive_id          CAN ID for incoming XCP frames
** \param     xcp_send_id_extended    1 = 29-bit extended ID, 0 = 11-bit standard
** \param     xcp_receive_id_extended 1 = 29-bit extended ID, 0 = 11-bit standard
** \return    none
***************************************************************************************/
void GO_xcp_init_can(FDCAN_HandleTypeDef *can_channel,
				   uint32_t xcp_send_id, uint32_t xcp_receive_id,
				   uint8_t xcp_send_id_extended,
				   uint8_t xcp_receive_id_extended) {
	/* General xcp stack settings*/
	xcpTransmissionBus = XCPCAN;
	xcpDtoId = xcp_send_id;
	xcpDtoIdExt = xcp_send_id_extended;
	XcpConnection_fd = can_channel;
	XcpDynamicConfigurator(0, 8, 8);

	/* Configure filter to route XCP receive ID to FIFO1 */
	FDCAN_FilterTypeDef canFilter = {0};
	/* High priority selection ono filer bank */
	canFilter.FilterIndex = 0;
	/* Only one message ID may pass this filter */
	canFilter.FilterType	= FDCAN_FILTER_DUAL;
	/* Port all xcp communictaion to FiFo 1 */
	canFilter.FilterConfig	= FDCAN_FILTER_TO_RXFIFO1;
	/* Select ID type*/
	if (xcp_receive_id_extended) {
		canFilter.IdType = FDCAN_EXTENDED_ID;
	} else {
		canFilter.IdType = FDCAN_STANDARD_ID;
	}
	/* Assign the range where XCP messages are filtered */
	canFilter.FilterID1 = xcp_receive_id;
	canFilter.FilterID2 = xcp_receive_id;

	if (HAL_FDCAN_ConfigFilter(can_channel, &canFilter) != HAL_OK)
		err("Could not config filter: 0x%x\n", can_channel->ErrorCode);

	xcp_received = osMessageQueueNew(10, sizeof(struct can_frame), NULL);

	if (HAL_FDCAN_RegisterRxFifo1Callback(can_channel, HAL_FDCAN_RxFifo1Callback) != HAL_OK)
		err("Could not register FIFO1 callback: 0x%x\n", can_channel->ErrorCode);
	if (HAL_FDCAN_ActivateNotification(can_channel,
									   FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
		err("Could not activate notification: 0x%x\n", can_channel->ErrorCode);
}

/****************************************************************************************/

/**************************************************************************************
** \brief     FreeRTOS task that processes incoming XCP CAN frames.
** \param     args  unused task argument
** \return    none (infinite loop)
***************************************************************************************/
void GO_xcp_thread_can(void *args) {
	struct can_frame message;

	while (1) {
		if (!osMessageQueueGet(xcp_received, &message, 0, osWaitForever)) {
			dbg("received CAN message, dlc: %d, id: %x\ndata: [",
				GO_communication_can_packed_dlc(&message), message.id);
			for (int i = 0; i < GO_communication_can_packed_dlc(&message); i++) {
				dbg("%02x,", message.data[i]);
			}
			dbg("]\n");
			dbg("xcp stack free: %d\n", osThreadGetStackSpace(osThreadGetId()));

			XcpCommunicationHandling(message.data, GO_communication_can_packed_dlc(&message),
									 dataToSend);
		}
	}
}

/****************************************************************************************/

/**************************************************************************************
** \brief     Send the data from the XCP stack to the active transport bus.
** \param     data  pointer to the data array to transmit
** \return    0 on success, 1 on failure
***************************************************************************************/
uint8_t GO_xcp_send_data(uint8_t *data) {
	switch (xcpTransmissionBus) {
		case XCPCAN:
			return XcpCanSend(data);
		default:
			return 1;
	}
}

/****************************************************************************************/

static uint8_t XcpCanSend(uint8_t *data) {
	if (data[0] == 0 || data[0] > 8) {
		err("Could not send message, incorrect size: %d\n", data[0]);
		return 1;
	}
	osDelay(1);


	FDCAN_TxHeaderTypeDef header = {0};
	header.DataLength          = data[0];
	header.Identifier          = xcpDtoId;
	header.IdType              = xcpDtoIdExt ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
	header.TxFrameType         = FDCAN_DATA_FRAME;
	header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	header.BitRateSwitch       = FDCAN_BRS_OFF;
	header.FDFormat            = FDCAN_CLASSIC_CAN;
	header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
	header.MessageMarker       = 0;
	if (HAL_FDCAN_AddMessageToTxFifoQ((FDCAN_HandleTypeDef*)XcpConnection_fd, &header, &data[1]) != HAL_OK)
		return 1;
	return 0;
}



/****************************************************************************************/

/**************************************************************************************
** \brief     Stop the active XCP connection and release resources.
** \return    none
***************************************************************************************/
void GO_xcp_stop_connection(void) { return; }

/**************************************************************************************
** \brief     Handle a user-defined XCP command (transport-specific behaviour).
** \param     dataReceived  pointer to the received XCP command bytes
** \return    0 on success, -1 on unrecognised command
***************************************************************************************/
uint8_t GO_xcp_user_cmd(uint8_t *dataReceived) { return 0; }

/****************************************************************************************
 ****************************************************************************************
 * Linux specific implementations
 ****************************************************************************************
 ****************************************************************************************/
#elif defined(GOCONTROLL_LINUX)

/****************************************************************************************
 * Type definitions
 ****************************************************************************************/

/** \brief XCP DTO packet type with TCP/IP header */
typedef union {
	struct {
		uint16_t len;
		uint16_t counter;
		uint8_t  data[XCPMAXDTOLENGTH];
	} s;
	uint8_t raw[4 + XCPMAXDTOLENGTH];
} __attribute__((packed)) tMacNetXcpDtoPacket;

/** \brief XCP CTO packet type with TCP/IP header */
typedef union {
	struct {
		uint16_t len;
		uint16_t counter;
		uint8_t  data[XCPMAXCTOLENGTH];
	} s;
	uint8_t raw[4 + XCPMAXCTOLENGTH];
} __attribute__((packed)) tMacNetXcpCtoPacket;

/****************************************************************************************
 * Data declarations
 ****************************************************************************************/
int  xcpSocketResult, XcpSocket;
static struct sockaddr_in addr_HT;
static int		  slen		       = sizeof(addr_HT);
static uint8_t		  dataToSend[120]      = {0};
static uint8_t		  xcpTransmissionBus   = 0;
static int		  timeout_sec	       = 0;
bool			  timeout_active       = false;

uint32_t uniqueIdLengthAdress;
uint32_t uniqueIdStringAdress;
int	 XcpConnection_fd;

/** \brief ECU identifier variables */
uint32_t uniqueIdLength = (uint32_t)kXcpStationIdLength;
char	 uniqueIdString[] = kXcpStationIdString;

timer_t		      countDownTimer = 0;
struct t_eventData {
	int myData;
};
static struct t_eventData eventData = {.myData = 0};
static struct sigevent	  sev	    = {0};
struct itimerspec	  its;

typedef struct {
	uint32_t xcpDtoId;
	uint8_t  xcpMacCanChannel;
} _xcpCanParameters;

static _xcpCanParameters xcpCanParameters;

/****************************************************************************************
 * Internal function prototypes
 ****************************************************************************************/
static int    ServeEthXcpConnection(void);
static int    ServeCANXcpConnection(void);
static uint8_t XcpEthSend(uint8_t *data);
static uint8_t XcpCanSend(uint8_t *data);
static void   expired(union sigval timer_data);

/****************************************************************************************/

static void expired(union sigval timer_data) {
	(void)timer_data;
	char data[3];
	err("Timer expired, killing XCP\n");
	data[0] = 0xdd; /* START_STOP_DAQ_LIST */
	data[1] = 0x00; /* Mode 00 = stop DAQ list */
	XcpCommunicationHandling((uint8_t *)&data, (uint32_t)3,
							 (uint8_t *)&dataToSend);
	its.it_value.tv_sec	 = 0;
	its.it_value.tv_nsec	 = 0;
	its.it_interval.tv_sec	 = 0;
	its.it_interval.tv_nsec	 = 0;
	int res = timer_settime(countDownTimer, 0, &its, NULL);
	if (res != 0) {
		err("Error timer_settime: %s\n", strerror(errno));
	}
}

/****************************************************************************************/

/**************************************************************************************
** \brief     Start the XCP slave over TCP/IP (blocking — runs in a thread).
** \param     aArgument  unused
** \return    none
***************************************************************************************/
void *GO_xcp_initialize_tcp(void *aArgument) {
	(void)aArgument;
	xcpTransmissionBus = XCPETH;
	struct sockaddr_in XcpSocketAddr, XCPclientAddr;
	socklen_t	   XCPclientAddrLen;
	char		   XcpClientAddrStr[INET_ADDRSTRLEN];
	XCPclientAddrLen = sizeof(struct sockaddr_in);

#if defined(XCP_ENABLE_DAQ_TIMESTAMP)
	xcpTimestampValue = 0;
#endif

	XcpDynamicConfigurator(0, 255, 255);

	XcpSocket = socket(AF_INET, SOCK_STREAM, 0);
	if (XcpSocket == -1)
		err("Error in XCPinit; create socket: %s\n", strerror(errno));

	dbg("Successfully created socket\n");

	if (setsockopt(XcpSocket, SOL_SOCKET, SO_REUSEADDR, &(int){1},
				   sizeof(int)) < 0)
		err("setsockopt(SO_REUSEADDR) failed");

	memset(&XcpSocketAddr, 0, sizeof(struct sockaddr_in));
	XcpSocketAddr.sin_family      = AF_INET;
	XcpSocketAddr.sin_addr.s_addr = INADDR_ANY;
	XcpSocketAddr.sin_port	      = htons(XCP_PORT_NUM);

	xcpSocketResult = bind(XcpSocket, (struct sockaddr *)&XcpSocketAddr,
						   sizeof(XcpSocketAddr));
	if (xcpSocketResult == -1)
		err("Error in XCPinit; bind socket: %s\n", strerror(errno));

	dbg("Successfully bound socket\n");

	xcpSocketResult = listen(XcpSocket, SOMAXCONN);
	if (xcpSocketResult == -1)
		err("Error in XCPinit; listen to socket: %s\n", strerror(errno));

	dbg("Successfully listening to socket\n");

	unsigned int timeout = 5000;
	if (setsockopt(XcpSocket, SOL_TCP, TCP_USER_TIMEOUT, &timeout,
				   sizeof(timeout)) < 0)
		err("setsockopt(TCP_USER_TIMEOUT) failed\n");

	for (;;) {
		XcpConnection_fd = accept(XcpSocket, (struct sockaddr *)&XCPclientAddr,
								  &XCPclientAddrLen);
		if (XcpConnection_fd == -1) {
			err("Error in XCPinit; accepting incoming connection %s\n",
				strerror(errno));
			if (close(XcpConnection_fd) == -1)
				err("XcpInitialize: error while trying to close the "
					"connection: %s\n", strerror(errno));
			continue;
		}
		dbg("Successfully accepted incoming connection from client\n");

		if (inet_ntop(AF_INET, &XCPclientAddr.sin_addr.s_addr,
					  XcpClientAddrStr, sizeof(XcpClientAddrStr)) != NULL)
			info("Incoming connection from client: %s\n",
				 inet_ntoa(XCPclientAddr.sin_addr));
		else
			err("Unable to get client IP address: %s\n", strerror(errno));

		while (ServeEthXcpConnection() != -1) {
		}

		dbg("ServeXcpConnection() returned an error\n");
		if (close(XcpConnection_fd) == -1)
			err("XcpInitialize: error while trying to close the "
				"connection: %s\n", strerror(errno));
	}

	return 0;
}

/****************************************************************************************/

/**************************************************************************************
** \brief     Start the XCP slave over UDP/IP (blocking — runs in a thread).
** \param     aArgument  unused
** \return    none
***************************************************************************************/
void *GO_xcp_initialize_udp(void *aArgument) {
	(void)aArgument;
	xcpTransmissionBus = XCPETH;
	struct sockaddr_in XcpSocketAddr;
	int		   result;
	timeout_active = true;

#if defined(XCP_ENABLE_DAQ_TIMESTAMP)
	xcpTimestampValue = 0;
#endif

	XcpDynamicConfigurator(0, 255, 255);

	XcpConnection_fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (XcpConnection_fd == -1)
		err("Error in XCPinit; create socket: %s\n", strerror(errno));

	dbg("Successfully created socket\n");

	memset(&XcpSocketAddr, 0, sizeof(struct sockaddr_in));
	XcpSocketAddr.sin_family      = AF_INET;
	XcpSocketAddr.sin_addr.s_addr = INADDR_ANY;
	XcpSocketAddr.sin_port	      = htons(XCP_PORT_NUM);

	result = bind(XcpConnection_fd, (struct sockaddr *)&XcpSocketAddr,
				  sizeof(XcpSocketAddr));
	if (result == -1)
		err("Error in XCPinit; bind socket: %s\n", strerror(errno));

	dbg("Successfully bound socket\n");

	sev.sigev_notify	      = SIGEV_THREAD;
	sev.sigev_notify_function     = &expired;
	sev.sigev_value.sival_ptr     = &eventData;
	int res = timer_create(CLOCK_REALTIME, &sev, &countDownTimer);
	if (res != 0) {
		err("Error timer_create: %s\n", strerror(errno));
	}

	for (;;) {
		while (ServeEthXcpConnection() != -1) {
		}
		dbg("ServeXcpConnection() returned an error\n");
	}

	return 0;
}

/****************************************************************************************/

/**************************************************************************************
** \brief     Start the XCP slave over CAN (blocking — runs in a thread).
** \param     aArgument  pointer to _XCP_CAN_Args
** \return    none
***************************************************************************************/
void *GO_xcp_initialize_can(void *aArgument) {
	xcpTransmissionBus = XCPCAN;
	_XCP_CAN_Args	   *args = (_XCP_CAN_Args *)aArgument;
	xcpCanParameters.xcpDtoId = args->xcp_send_id;
	struct sockaddr_can addr;
	struct ifreq	    ifr;
	int		    result;

#if defined(XCP_ENABLE_DAQ_TIMESTAMP)
	xcpTimestampValue = 0;
#endif

	XcpDynamicConfigurator(0, 8, 8);
	XcpConnection_fd = socket(AF_CAN, SOCK_RAW, CAN_RAW);
	if (XcpConnection_fd == -1)
		err("Error in XCPinit; create socket: %s\n", strerror(errno));

	dbg("Successfully created socket\n");

	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	strncpy(ifr.ifr_name, args->can_channel, sizeof(ifr.ifr_name) - 1);
	if (ioctl(XcpConnection_fd, SIOCGIFINDEX, &ifr) < 0)
		err("CAN socket ioctl SIOCGIFINDEX error.\n");
	addr.can_ifindex = ifr.ifr_ifindex;

	result = bind(XcpConnection_fd, (struct sockaddr *)(void *)&addr,
				  sizeof(addr));
	if (result == -1)
		err("Error in XCPinit; bind socket: %s\n", strerror(errno));

	dbg("Successfully bound socket\n");

	struct can_filter filter;
	filter.can_id   = args->xcp_receive_id;
	filter.can_mask = CAN_EFF_MASK | CAN_EFF_FLAG;
	result = setsockopt(XcpConnection_fd, SOL_CAN_RAW, CAN_RAW_FILTER,
						&filter, sizeof(filter));
	if (result == -1)
		err("Error in XCPinit; setsockopt: %s\n", strerror(errno));

	for (;;) {
		while (ServeCANXcpConnection() != -1) {
		}
		dbg("ServeXcpConnection() returned an error\n");
	}

	return 0;
}

/****************************************************************************************/

static int ServeEthXcpConnection(void) {
	tMacNetXcpCtoPacket ctoPacket;
	ssize_t		    readCnt;

	do {
		readCnt = recvfrom(
			XcpConnection_fd, (void *)&ctoPacket, 4, MSG_PEEK,
			(struct sockaddr *restrict)&addr_HT,
			(socklen_t *restrict)&slen);
		dbg("recv1: %li\n", readCnt);
		if (readCnt != 4) {
			if (readCnt == -1) {
				err("ServeXcpConnection failed to read first 4 bytes: %s\n",
					strerror(errno));
				return -1;
			} else if (readCnt == 0) {
				if (TIMEOUT == 1) err("ServeXcpConnection: Connection closed\n");
				return -1;
			}
		} else {
			readCnt = recvfrom(
				XcpConnection_fd, (uint8_t *)&ctoPacket,
				(size_t)ctoPacket.s.len + 4, 0,
				(struct sockaddr *restrict)&addr_HT,
				(socklen_t *restrict)&slen);
			dbg("recv2: %li\n", readCnt);
			if (readCnt == 0) {
				err("Connection lost, closing socket...\n");
				return -1;
			} else if (readCnt != ctoPacket.s.len + 4) {
				err("ServeXcpConnection failed to read rest of message: %s\n",
					strerror(errno));
			} else {
				dbg("Message from HANtune: %d\n", ctoPacket.s.len);
				for (int i = 0; i < ctoPacket.s.len; i++) {
					dbg("%02x ", ctoPacket.s.data[i]);
				}
				dbg("\n");
				if (timeout_active) {
					if (ctoPacket.s.data[0] == 0xFE)
						timeout_sec = 0;

					its.it_value.tv_sec	= timeout_sec;
					its.it_value.tv_nsec	= 0;
					its.it_interval.tv_sec	= timeout_sec;
					its.it_interval.tv_nsec	= 0;

					int res = timer_settime(countDownTimer, 0, &its, NULL);
					if (res != 0)
						err("Error timer_settime: %s\n", strerror(errno));
				}
				XcpCommunicationHandling(
					(uint8_t *)&ctoPacket.s.data,
					(uint32_t)ctoPacket.s.len,
					(uint8_t *)&dataToSend);
			}
		}
	} while (readCnt > 0);
	return 0;
}

/****************************************************************************************/

static int ServeCANXcpConnection(void) {
	struct can_frame sc_frame;
	int		 ret;

	while ((ret = recv(XcpConnection_fd, &sc_frame, sizeof(sc_frame), 0)) > 0) {
		if (ret < (int)sizeof(sc_frame))
			continue;

		dbg("received CAN message, dlc: %d, id: %x\ndata: [",
			sc_frame.can_dlc, sc_frame.can_id);
		for (int i = 0; i < sc_frame.can_dlc; i++) {
			dbg("%02x,", sc_frame.data[i]);
		}
		dbg("]\n");

		XcpCommunicationHandling(sc_frame.data, sc_frame.can_dlc, dataToSend);
	}

	return ret;
}

/****************************************************************************************/

/**************************************************************************************
** \brief     Send the data from the XCP stack to the active transport bus.
** \param     data  pointer to the data array to transmit
** \return    0 on success, 1 on failure
***************************************************************************************/
uint8_t GO_xcp_send_data(uint8_t *data) {
	switch (xcpTransmissionBus) {
		case XCPETH:
			return XcpEthSend(data);
		case XCPCAN:
			return XcpCanSend(data);
		default:
			return 1;
	}
}

/****************************************************************************************/

static uint8_t XcpEthSend(uint8_t *data) {
	static uint16_t	    CTR = 0;
	tMacNetXcpDtoPacket dtoPacket;
	ssize_t		    res;

	dtoPacket.s.len	    = (uint16_t)data[0];
	dtoPacket.s.counter = CTR++;
	memcpy(dtoPacket.s.data, data + 1, dtoPacket.s.len);

	addr_HT.sin_port = htons(50000); // TODO: port should be a Simulink setting
	res = sendto(XcpConnection_fd, (void *)&dtoPacket.raw[0],
				 (size_t)dtoPacket.s.len + 4, MSG_DONTWAIT,
				 (struct sockaddr *)&addr_HT, slen);
	if (res != (dtoPacket.s.len + 4)) {
		dbg("sent %ld bytes, expected %d\n", res, dtoPacket.s.len + 4);
		dbg("XcpSendData: Error in sending: %s\n", strerror(errno));
		if (close(XcpConnection_fd) == -1)
			err("XcpSendData: error while trying to close the connection: "
				"%s\n", strerror(errno));
		return 1;
	}
	return 0;
}

/****************************************************************************************/

static uint8_t XcpCanSend(uint8_t *data) {
	struct can_frame sc_frame;
	int		 res;
	if (data[0] != 0 && data[0] <= 8) {
		sc_frame.can_dlc = data[0];
		sc_frame.can_id	 = xcpCanParameters.xcpDtoId;
		memcpy(sc_frame.data, &data[1], data[0]);
		res = send(XcpConnection_fd, &sc_frame, sizeof(sc_frame), MSG_DONTWAIT);
		if (res >= 0) {
			return 0;
		}
	} else {
		dbg("Could not send message, incorrect size: %d\n", data[0]);
	}
	return 1;
}

/****************************************************************************************/

/**************************************************************************************
** \brief     Stop the active XCP connection and release resources.
** \return    none
***************************************************************************************/
void GO_xcp_stop_connection(void) {
	close(XcpSocket);
	dbg("XCP socket closed\n");
}

/****************************************************************************************/

/**************************************************************************************
** \brief     Handle a user-defined XCP command (transport-specific behaviour).
** \param     dataReceived  pointer to the received XCP command bytes
** \return    0 on success, -1 on unrecognised command
***************************************************************************************/
uint8_t GO_xcp_user_cmd(uint8_t *dataReceived) {
	switch (dataReceived[1]) {
		case 0x10:
			timeout_sec = (int)(dataReceived[2] << 8) + dataReceived[3];
			/* Make target timeout longer than HANtune timeout */
			timeout_sec = 2 * timeout_sec;

			its.it_value.tv_sec	= timeout_sec;
			its.it_interval.tv_sec	= timeout_sec;

			int res = timer_settime(countDownTimer, 0, &its, NULL);
			if (res != 0)
				err("Error timer_settime: %s\n", strerror(errno));
			return 0;
		case 0x11:
			return 0;
		default:
			return -1;
	}
}

#endif /* GOCONTROLL_IOT / GOCONTROLL_LINUX */

/* end of GO_xcp.c */
