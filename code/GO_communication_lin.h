/**************************************************************************************
 * \file   GO_communication_lin.h
 * \brief  LIN bus communication interface for GOcontroll hardware.
 *         Handles LIN master send/receive, message scheduling and interface
 *         initialisation.
 *
 *         Platform selection via preprocessor define:
 *           GOCONTROLL_IOT  →  STM32H5 (Moduline IOT): TODO
 *           (default)       →  Linux/IMX8 (Moduline IV / Moduline Mini)
 *
 *         This code is heavily inspired by slLIN prototype code developed by:
 *           Pavel Pisa <pisa@cmp.felk.cvut.cz>
 *           Rostislav Lisovy <lisovy@kormus.cz>
 *         Source: https://github.com/lin-bus/linux-lin/
 *         Released under MIT licence with their permission.
 * \internal
 *----------------------------------------------------------------------------------------
 *                          C O P Y R I G H T
 *----------------------------------------------------------------------------------------
 * Copyright 2024 (c) by GOcontroll http://www.gocontroll.com All rights reserved
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

#ifndef GO_COMMUNICATION_LIN_H
#define GO_COMMUNICATION_LIN_H

/****************************************************************************************
 * Include files
 ****************************************************************************************/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************************
 * Macro definitions
 ****************************************************************************************/

/* Maximum number of data bytes in a LIN message */
#define SLLIN_DATA_MAX	8
/* Total LIN frame buffer size: break + sync + ID + data + checksum */
#define SLLIN_BUFF_LEN	(1 + 1 + 1 + SLLIN_DATA_MAX + 1)

/****************************************************************************************
 * Data declarations — Linux specific
 ****************************************************************************************/
#ifdef GOCONTROLL_LINUX

#include <asm/termbits.h>

struct sllin_tty {
	int             tty_fd;
	struct termios2 tattr_orig;
	struct termios2 tattr;
};

struct sllin {
	struct sllin_tty *tty;           /* ptr to TTY structure          */
	unsigned char    rx_buff[SLLIN_BUFF_LEN]; /* LIN Rx buffer        */
	unsigned char    tx_buff[SLLIN_BUFF_LEN]; /* LIN Tx buffer        */
	int              rx_expect;      /* expected number of Rx chars   */
	int              rx_lim;         /* maximum Rx chars for ID       */
	int              rx_cnt;         /* message buffer Rx fill level  */
	int              tx_lim;         /* actual limit of bytes to Tx   */
	int              tx_cnt;         /* number of already Tx bytes    */
	char             lin_master;     /* node is a master node         */
	int              lin_baud;       /* LIN baudrate                  */
	int              lin_break_baud; /* baudrate used for break send  */
	int              lin_state;      /* state                         */
	int              id_to_send;     /* there is ID to be sent        */
	unsigned long    flags;          /* flag values / mode etc        */
};

/****************************************************************************************
 * Data declarations — STM32H5 (GOCONTROLL_IOT) specific
 ****************************************************************************************/
#elif defined(GOCONTROLL_IOT)

/*
 * TODO: add STM32H5 LIN data types here.
 *
 * Suggested candidates:
 *   - LIN handle wrapping the UART peripheral (e.g. UART_HandleTypeDef *)
 *   - LIN state machine struct for break/sync/ID/data sequencing
 *   - DMA or interrupt-driven rx/tx buffer descriptors
 *
 * Required HAL includes (add as needed):
 *   #include "stm32h5xx_hal.h"
 *   #include "usart.h"
 */

#endif /* GOCONTROLL_LINUX / GOCONTROLL_IOT */

/****************************************************************************************
 * Function prototypes — platform-independent API
 ****************************************************************************************/

/**************************************************************************************
** \brief     Open and configure the LIN serial interface.
**            Linux:  opens /dev/ttymxc3 at 19200 baud.
**            STM32:  TODO — configure UART peripheral for LIN break detection.
** \return    0 on success, -1 on failure
***************************************************************************************/
int GO_communication_lin_initialize_interface(void);

/**************************************************************************************
** \brief     Close the LIN serial interface and restore terminal settings.
**            Linux:  restores termios settings and closes file descriptor.
**            STM32:  TODO — disable UART peripheral.
** \return    0 on success, -1 on failure
***************************************************************************************/
int GO_communication_lin_de_initialize_interface(void);

/**************************************************************************************
** \brief     Round-robin LIN message scheduler.
**            action 1 = register an ID during initialisation.
**            action 2 = query whether this ID may transmit now.
** \param     id      LIN frame identifier (0-63)
** \param     action  1 = register, 2 = query
** \return    1 if the message is allowed to transmit, 0 otherwise
***************************************************************************************/
uint8_t GO_communication_lin_message_scheduler(uint8_t id, uint8_t action);

/**************************************************************************************
** \brief     Send a LIN header and read the slave response.
** \param     id          LIN frame identifier (0-63)
** \param     dataLength  data length code (1 = 2 bytes, 2 = 4 bytes, 3 = 8 bytes)
** \param     data        buffer to store received data bytes
** \param     checksum    1 = classic checksum, 2 = enhanced checksum (ID included)
** \return    0 on success, -1 on failure
***************************************************************************************/
int GO_communication_lin_master_retrieve_data(uint8_t id, uint8_t dataLength,
                                          uint8_t data[], uint8_t checksum);

/**************************************************************************************
** \brief     Send a complete LIN frame (header + data + checksum).
** \param     id          LIN frame identifier (0-63)
** \param     dataLength  data length code (1 = 2 bytes, 2 = 4 bytes, 3 = 8 bytes)
** \param     data        data bytes to transmit
** \param     checksum    1 = classic checksum, 2 = enhanced checksum (ID included)
** \return    0 on success, -1 on failure
***************************************************************************************/
int GO_communication_lin_master_send_data(uint8_t id, uint8_t dataLength,
                                      uint8_t data[], uint8_t checksum);

#ifdef __cplusplus
}
#endif

#endif /* GO_COMMUNICATION_LIN_H */

/* end of GO_communication_lin.h */
