/************************************************************************************//**
* \file         GO_communication_can.c
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

#include "GO_communication_can.h"

/****************************************************************************************
* IoT platform (STM32 FDCAN via HAL)
****************************************************************************************/
#ifdef GOCONTROLL_IOT

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "main.h"
#include "print.h"


struct fdcan_timing {
	uint32_t prescaler;
	uint32_t seg1;
	uint32_t seg2;
};

static const struct fdcan_timing timing_table[4] = {
	{2,  15, 32},   /* CAN125KBPS: 12 MHz / (8 * 12) = 125 kbps, SP=75% */
	{1,  15, 32},   /* CAN250KBPS: 12 MHz / (4 * 12) = 250 kbps, SP=75% */
	{1,  10, 13},   /* CAN500KBPS: 12 MHz / (2 * 12) = 500 kbps, SP=75% */
	{1,  5, 6},   /* CAN1MBPS:   12 MHz / (1 * 12) = 1 Mbps,   SP=75% */
};

/**************************************************************************************
** \brief     Pack an FDCAN RX header into the compact can_frame representation.
** \param     frame   Output frame.
** \param     header  FDCAN RX header from HAL_FDCAN_GetRxMessage().
** \return    none
***************************************************************************************/
void GO_communication_can_pack_header(struct can_frame *frame,
                                   FDCAN_RxHeaderTypeDef *header) {
	frame->id = header->Identifier;
	/* DLC: FDCAN uses FDCAN_DLC_BYTES_x macros; lower nibble holds byte count */
	frame->flags  = (uint8_t)(header->DataLength) & CAN_PACKED_DLC;
	frame->flags |= (header->IdType == FDCAN_EXTENDED_ID)   ? CAN_PACKED_EXTID : 0;
	frame->flags |= (header->RxFrameType == FDCAN_REMOTE_FRAME) ? CAN_PACKED_RTR : 0;
}

/**************************************************************************************
** \brief     Extract the DLC byte count from a packed can_frame.
** \param     frame  Pointer to the packed CAN frame.
** \return    DLC byte count (0-8).
***************************************************************************************/
uint8_t GO_communication_can_packed_dlc(struct can_frame *frame) {
	return frame->flags & CAN_PACKED_DLC;
}

/**************************************************************************************
** \brief     Return true when the packed frame carries a 29-bit extended identifier.
** \param     frame  Pointer to the packed CAN frame.
** \return    true if the frame uses an extended (29-bit) identifier, false otherwise.
***************************************************************************************/
bool GO_communication_can_packed_is_ext_id(struct can_frame *frame) {
	return (frame->flags & CAN_PACKED_EXTID) > 0;
}

/**************************************************************************************
** \brief     Return true when the packed frame is a Remote Transmission Request.
** \param     frame  Pointer to the packed CAN frame.
** \return    true if the frame is an RTR frame, false otherwise.
***************************************************************************************/
bool GO_communication_can_packed_is_rtr(struct can_frame *frame) {
	return (frame->flags & CAN_PACKED_RTR) > 0;
}

/**************************************************************************************
** \brief     Handle FDCAN error-status interrupt: on bus-off, clears CCCR.INIT to
**            trigger the automatic 128 x 11 recessive-bit recovery sequence.
**            Safe to call from the registered HAL ErrorStatusCallback (ISR context).
** \param     hfdcan         FDCAN handle that raised the error interrupt.
** \param     ErrorStatusITs Error status interrupt flags passed by the HAL callback.
** \return    none
***************************************************************************************/
void GO_communication_can_bus_off_recovery(FDCAN_HandleTypeDef *hfdcan,
                                       uint32_t ErrorStatusITs) {
	if (ErrorStatusITs & FDCAN_IT_BUS_OFF) {
		/* Clear CCCR.INIT to start the automatic recovery sequence.
		 * The M_CAN hardware monitors the bus for 128 × 11 consecutive recessive
		 * bits and then rejoins without further software intervention. This is
		 * non-blocking and safe to call from ISR context. */
		CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
		err("CAN bus-off: recovery triggered\n");
	}
}

/* Per-channel bitrate bookkeeping — set by GO_communication_can_initialize(),
 * read by can_get_esp_bitrate() / can_get_busload() for the ESP protocol info message. */
static uint8_t s_can1_esp_bitrate = 0u;
static uint8_t s_can2_esp_bitrate = 0u;

/* Per-channel bus load accumulators.
 * bits_accumulated: total bit-time-equivalent bits seen since last can_get_busload() call.
 * last_cyccnt:      DWT->CYCCNT snapshot taken at the end of the last can_get_busload() call. */
struct can_busload_state {
    volatile uint32_t bits_accumulated;
    uint32_t          last_cyccnt;
};
static struct can_busload_state s_busload[2];

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
                            FunctionalState autort) {
	if (baudrate >= 4) {
		err("Invalid baudrate index %d\n", baudrate);
		return -1;
	}

	/* Enable DWT cycle counter for bus load timing (safe to call multiple times). */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;

	const struct fdcan_timing *t = &timing_table[baudrate];

	hfdcan->Init.ClockDivider         = FDCAN_CLOCK_DIV1;
	hfdcan->Init.FrameFormat          = FDCAN_FRAME_CLASSIC;
	hfdcan->Init.Mode                 = FDCAN_MODE_NORMAL;
	hfdcan->Init.AutoRetransmission   = autort;
	hfdcan->Init.TransmitPause        = DISABLE;
	hfdcan->Init.ProtocolException    = DISABLE;
	hfdcan->Init.NominalPrescaler     = t->prescaler;
	hfdcan->Init.NominalSyncJumpWidth = 1;
	hfdcan->Init.NominalTimeSeg1      = t->seg1;
	hfdcan->Init.NominalTimeSeg2      = t->seg2;
	hfdcan->Init.DataPrescaler        = 1;
	hfdcan->Init.DataSyncJumpWidth    = 1;
	hfdcan->Init.DataTimeSeg1         = 1;
	hfdcan->Init.DataTimeSeg2         = 1;
	hfdcan->Init.StdFiltersNbr        = 2;
	hfdcan->Init.ExtFiltersNbr        = 2;
	hfdcan->Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;

	if (HAL_FDCAN_Init(hfdcan) != HAL_OK) {
		err("Could not init FDCAN\n");
		return -1;
	}

	/* Route all non-matching frames to FIFO0 by default.
	 * Any block that needs a specific FIFO (e.g. XCP → FIFO1) adds a
	 * higher-priority named filter afterwards. */
	if (HAL_FDCAN_ConfigGlobalFilter(hfdcan,
			FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
			FDCAN_FILTER_REMOTE,      FDCAN_FILTER_REMOTE) != HAL_OK) {
		err("Could not config global filter\n");
		return -1;
	}

	dbg("FDCAN init OK, prescaler=%d\n", t->prescaler);

	uint8_t ch_idx;
	if (hfdcan->Instance == FDCAN1) {
		s_can1_esp_bitrate = (uint8_t)(baudrate + 1u);
		ch_idx = 0u;
	} else {
		s_can2_esp_bitrate = (uint8_t)(baudrate + 1u);
		ch_idx = 1u;
	}
	s_busload[ch_idx].bits_accumulated = 0u;
	s_busload[ch_idx].last_cyccnt      = DWT->CYCCNT;

	return 0;
}

/**************************************************************************************
** \brief     Start the FDCAN peripheral and take the transceiver out of silent mode.
**            Call this AFTER all HAL callbacks are registered.
** \param     hfdcan   Pointer to the FDCAN handle (must be initialized with
**                     GO_communication_can_initialize).
** \return    0 on success, -1 on error.
***************************************************************************************/
int GO_communication_can_start(FDCAN_HandleTypeDef *hfdcan) {
	if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {
		err("Could not start FDCAN\n");
		return -1;
	}

	if (hfdcan->Instance == FDCAN1)
		HAL_GPIO_WritePin(CAN1_SILENT_UCO_GPIO_Port, CAN1_SILENT_UCO_Pin,
		                  GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(CAN2_SILENT_UCO_GPIO_Port, CAN2_SILENT_UCO_Pin,
		                  GPIO_PIN_RESET);
	return 0;
}

int init_can(FDCAN_HandleTypeDef *hfdcan, uint32_t baudrate, FunctionalState autort)
{
    return GO_communication_can_initialize(hfdcan, baudrate, autort);
}

int start_can(FDCAN_HandleTypeDef *hfdcan)
{
    return GO_communication_can_start(hfdcan);
}

void can_pack_header(struct can_frame *frame, FDCAN_RxHeaderTypeDef *header)
{
    GO_communication_can_pack_header(frame, header);
}

uint8_t can_packed_dlc(struct can_frame *frame)
{
    return GO_communication_can_packed_dlc(frame);
}

bool can_packed_is_ExtId(struct can_frame *frame)
{
    return GO_communication_can_packed_is_ext_id(frame);
}

bool can_packed_is_RTR(struct can_frame *frame)
{
    return GO_communication_can_packed_is_rtr(frame);
}

void can_bus_off_recovery(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    GO_communication_can_bus_off_recovery(hfdcan, ErrorStatusITs);
}

uint8_t can_get_esp_bitrate(uint8_t channel)
{
    if (channel == 1u) return s_can1_esp_bitrate;
    if (channel == 2u) return s_can2_esp_bitrate;
    return 0u;
}

/**************************************************************************************
** \brief     Accumulate the bit-time cost of one CAN frame on a channel.
**            Called from both TX path (task context) and RX FIFO0 callback (ISR context).
**            Uses a Classic CAN frame bit count with a ×1.1 stuffing factor (half worst-case):
**              Standard: (47 + dlc×8) × 11/10 bits
**              Extended: (67 + dlc×8) × 11/10 bits
** \param     channel   1 = CAN1, 2 = CAN2.
** \param     dlc       Data length code in bytes (0–8).
** \param     extended  true for 29-bit extended identifier, false for 11-bit standard.
** \return    none
***************************************************************************************/
void can_busload_count_frame(uint8_t channel, uint8_t dlc, bool extended)
{
    if (channel < 1u || channel > 2u) { return; }
    uint32_t overhead  = extended ? 67u : 47u;
    uint32_t bits      = (overhead + (uint32_t)dlc * 8u) * 11u / 10u;
    s_busload[channel - 1u].bits_accumulated += bits;
}

/**************************************************************************************
** \brief     Return the CAN bus load for the given channel and reset the accumulator.
**            Call periodically (e.g. every 200 ms from GO_communication_esp_send_cyclic_info).
**            Uses DWT->CYCCNT for elapsed-time measurement.
** \param     channel  1 = CAN1, 2 = CAN2.
** \return    Bus load in percent (0–100), or 0 if the channel is not initialised.
***************************************************************************************/
uint8_t can_get_busload(uint8_t channel)
{
    static const uint32_t bitrate_bps[5] = {0u, 125000u, 250000u, 500000u, 1000000u};

    if (channel < 1u || channel > 2u) { return 0u; }

    uint8_t  bitrate_idx = (channel == 1u) ? s_can1_esp_bitrate : s_can2_esp_bitrate;
    if (bitrate_idx == 0u) { return 0u; }  /* channel not initialised */

    struct can_busload_state *s = &s_busload[channel - 1u];

    uint32_t now     = DWT->CYCCNT;
    uint32_t elapsed = now - s->last_cyccnt;  /* correct even when CYCCNT wraps */
    s->last_cyccnt   = now;

    taskENTER_CRITICAL();
    uint32_t bits = s->bits_accumulated;
    s->bits_accumulated = 0u;
    taskEXIT_CRITICAL();

    if (elapsed == 0u) { return 0u; }

    /* load = bits_on_bus / (bitrate × elapsed_seconds) × 100
     *      = bits × SystemCoreClock × 100 / (bitrate × elapsed_cycles)   */
    uint64_t load = (uint64_t)bits * (uint64_t)SystemCoreClock * 100ULL
                    / ((uint64_t)bitrate_bps[bitrate_idx] * (uint64_t)elapsed);

    return (load > 100u) ? 100u : (uint8_t)load;
}

/*==============================================================================================
** Shared per-bus receive buffer — legacy CAN receive block (dynamic CAN ID).
==============================================================================================*/

static struct can_rx_slot_t s_rx_buf[2][CAN_RX_BUF_SIZE];

/**************************************************************************************
** \brief     Push a received frame into the shared per-bus buffer.
**            Call from the FDCAN FIFO0 RX callback (ISR context).
***************************************************************************************/
void GO_communication_can_rx_push(uint8_t canInterface, struct can_frame *frame)
{
    if (canInterface > 1u) { return; }
    for (uint8_t i = 0u; i < CAN_RX_BUF_SIZE; i++) {
        if (s_rx_buf[canInterface][i].id == frame->id) {
            memcpy(&s_rx_buf[canInterface][i].frame, frame, sizeof(struct can_frame));
            s_rx_buf[canInterface][i].new_flag = true;
            return;
        }
    }
    /* No matching slot registered — frame discarded */
}

/**************************************************************************************
** \brief     Retrieve a buffered frame by CAN ID (call from model step, task context).
***************************************************************************************/
int GO_communication_can_rx_get(uint8_t canInterface, uint32_t can_id,
                                 struct can_frame *frame_out, bool *new_flag)
{
    if (canInterface > 1u) { return -1; }
    for (uint8_t i = 0u; i < CAN_RX_BUF_SIZE; i++) {
        if (s_rx_buf[canInterface][i].id == can_id) {
            taskENTER_CRITICAL();
            memcpy(frame_out, &s_rx_buf[canInterface][i].frame, sizeof(struct can_frame));
            *new_flag = s_rx_buf[canInterface][i].new_flag;
            s_rx_buf[canInterface][i].new_flag = false;
            taskEXIT_CRITICAL();
            return 0;
        }
    }
    return -1;
}

/**************************************************************************************
** \brief     Update the CAN ID of an existing buffer slot (dynamic ID change).
***************************************************************************************/
int GO_communication_can_rx_update_id(uint8_t canInterface, uint32_t old_id,
                                       uint32_t new_id)
{
    if (canInterface > 1u) { return -1; }
    for (uint8_t i = 0u; i < CAN_RX_BUF_SIZE; i++) {
        if (s_rx_buf[canInterface][i].id == old_id) {
            s_rx_buf[canInterface][i].id       = new_id;
            s_rx_buf[canInterface][i].new_flag = false;
            return 0;
        }
    }
    return -1;
}

#endif /* GOCONTROLL_IOT */


/****************************************************************************************
* Linux platform (SocketCAN)
****************************************************************************************/
#ifdef GOCONTROLL_LINUX

#define NOTNEEDED        0
#define INITIALIZEFAILED 1
#define INITIALIZED      2
#define ERRORSTATE       3

_canConnection canConnection;

int can_socket; /* File descriptor for CAN socket */
struct can_filter CANfilter[4][CANBUFSIZE];
struct CANbuffer_t CANbuffer[4][CANBUFSIZE];

/**************************************************************************************
** \brief     Creates a CAN socket to be able to read and write CAN messages from.
** \param     canInterface  Index of the CAN bus (0-3).
** \return    none
***************************************************************************************/
void GO_communication_can_socket(uint8_t canInterface) {
	struct sockaddr_can addr;
	struct ifreq ifr;

	/* Check parameter plausibility */
	if (canInterface > 3) { return; }

	fprintf(stderr, "About to enter GO_communication_can_socket()\n");
	/* First check if the CAN socket has already been created */
	if (canConnection.socketCreated[canInterface] == NOTNEEDED ||
	    canConnection.socketCreated[canInterface] == INITIALIZEFAILED) {
		fprintf(stderr, "Entered GO_communication_can_socket()\n");
		/* Create the CAN socket */
		canConnection.socket[canInterface] = socket(PF_CAN, SOCK_RAW, CAN_RAW);
		if (canConnection.socket[canInterface] == -1) {
			perror("GO_communication_can_socket");
		} else {
			if (canInterface == 0) { strcpy(ifr.ifr_name, "can0"); }
			if (canInterface == 1) { strcpy(ifr.ifr_name, "can1"); }
			if (canInterface == 2) { strcpy(ifr.ifr_name, "can2"); }
			if (canInterface == 3) { strcpy(ifr.ifr_name, "can3"); }

			if (ioctl(canConnection.socket[canInterface], SIOCGIFINDEX, &ifr) < 0) {
				canConnection.socketCreated[canInterface] = INITIALIZEFAILED;
				return;
			}

			addr.can_family  = AF_CAN;
			addr.can_ifindex = ifr.ifr_ifindex;

			if (bind(canConnection.socket[canInterface],
			         (struct sockaddr *)&addr, sizeof(addr)) < 0) {
				canConnection.socketCreated[canInterface] = INITIALIZEFAILED;
				return;
			} else {
				canConnection.socketCreated[canInterface] = INITIALIZED;
			}
		}
	}
}

/**************************************************************************************
** \brief     Drain all open SocketCAN sockets and copy received frames into the buffer.
** \return    0 on success, -1 on error.
***************************************************************************************/
int GO_communication_can_serve_connection(void) {
	struct can_frame frame;
	uint16_t i;

	for (uint8_t canInterface = 0; canInterface < 4; canInterface++) {
		if (canConnection.socketCreated[canInterface] == INITIALIZED) {
			while (recv(canConnection.socket[canInterface], &frame,
			            sizeof(struct can_frame), MSG_DONTWAIT) > 0) {
				for (i = 0; i < CANBUFSIZE; i++) {
					if (frame.can_id == CANbuffer[canInterface][i].can_id) {
						memcpy(&CANbuffer[canInterface][i].frame, &frame,
						       sizeof(struct can_frame));
						CANbuffer[canInterface][i].newFlag = true;
						break;
					}
				}
			}
		} else if (canConnection.socketCreated[canInterface] == INITIALIZEFAILED) {
			if ((canConnection.initErrorCounter[canInterface]++) > 15) {
				canConnection.socketCreated[canInterface] = ERRORSTATE;
				printf("Interface: CAN%d not found!\n", canInterface);
			}
			GO_communication_can_socket(canInterface);
		}
	}
	return 0;
}

/**************************************************************************************
** \brief     Gets a CAN message from the receive buffer.
** \param     can_id        Identifier of the CAN message to retrieve.
** \param     canInterface  CAN bus index.
** \param     frame         Output pointer to the stored can_frame.
** \param     newFlag       Output flag indicating whether the message is new.
** \return    0 on success, -1 on error.
***************************************************************************************/
int GO_communication_can_get_buffer(canid_t can_id, uint8_t canInterface,
                                 struct can_frame **frame, bool *newFlag) {
	if (canInterface > 3) { return -1; }

	for (uint16_t i = 0; i < CANBUFSIZE; i++) {
		if (CANbuffer[canInterface][i].can_id == can_id) {
			*frame   = &CANbuffer[canInterface][i].frame;
			*newFlag = CANbuffer[canInterface][i].newFlag;
			CANbuffer[canInterface][i].newFlag = false;
			return 0;
		}
	}
	return -1;
}

/**************************************************************************************
** \brief     Update the SocketCAN receive filter for a given ID slot.
** \param     oldCANid      Identifier to replace.
** \param     canInterface  CAN bus index.
** \param     newCANid      New identifier.
** \return    0 on success, -1 on error.
***************************************************************************************/
int GO_communication_can_update_receive_filter(canid_t oldCANid, uint8_t canInterface,
                                           canid_t newCANid) {
	if (canInterface > 3) { return -1; }

	uint16_t i;
	int returnValue;

	for (i = 0; i <= CANBUFSIZE; i++) {
		if (CANfilter[canInterface][i].can_id == oldCANid) {
			CANfilter[canInterface][i].can_id   = newCANid;
			CANfilter[canInterface][i].can_mask = CAN_EFF_MASK;
			break;
		}
	}
	if (i >= CANBUFSIZE) {
		returnValue = -1;
	} else {
		returnValue = setsockopt(can_socket, SOL_CAN_RAW, CAN_RAW_FILTER,
		                         &CANfilter[canInterface],
		                         sizeof(CANfilter[canInterface]));
	}
	return returnValue;
}

/**************************************************************************************
** \brief     Update the receive buffer slot for a given ID.
** \param     oldCANid      Identifier to replace.
** \param     canInterface  CAN bus index.
** \param     newCANid      New identifier.
** \return    0 on success, -1 on error.
***************************************************************************************/
int GO_communication_can_update_receive_buffer(canid_t oldCANid, uint8_t canInterface,
                                           canid_t newCANid) {
	if (canInterface > 3) { return -1; }

	uint16_t i;
	int returnValue;

	for (i = 0; i <= CANBUFSIZE; i++) {
		fprintf(stdout, "CANid of buffer %x and CAN id of block 0x%x\n",
		        CANbuffer[canInterface][i].can_id, oldCANid);
		if (CANbuffer[canInterface][i].can_id == oldCANid) {
			CANbuffer[canInterface][i].can_id = newCANid;
			fprintf(stdout, "Just added the newCANid to the buffer: 0x%x\n",
			        CANbuffer[canInterface][i].can_id);
			break;
		}
	}
	if (i >= CANBUFSIZE) {
		returnValue = -1;
	} else {
		returnValue = 0;
	}
	return returnValue;
}

#endif /* GOCONTROLL_LINUX */
