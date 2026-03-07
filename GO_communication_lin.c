/**************************************************************************************
 * \file   GO_communication_lin.c
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

/****************************************************************************************
 * Include files — common
 ****************************************************************************************/
#include "GO_communication_lin.h"

/****************************************************************************************
 ****************************************************************************************
 * STM32H5 (GOCONTROLL_IOT) specific implementations
 ****************************************************************************************
 ****************************************************************************************/
#ifdef GOCONTROLL_IOT

/*
 * TODO: Implement LIN communication for STM32H5.
 *
 * Suggested approach:
 *   - GO_communication_lin_initialize_interface: configure UART peripheral in LIN mode
 *     using HAL_LIN_Init(), set baud rate to 19200, enable break detection.
 *   - GO_communication_lin_de_initialize_interface: call HAL_UART_DeInit() and disable
 *     the peripheral clock.
 *   - GO_communication_lin_message_scheduler: port the existing round-robin scheduler
 *     directly — logic is platform-independent.
 *   - GO_communication_lin_master_retrieve_data: use HAL_LIN_SendBreak() to generate the
 *     LIN break field, then transmit sync + PID via HAL_UART_Transmit(), and
 *     receive the slave response with HAL_UART_Receive() with a suitable timeout.
 *   - GO_communication_lin_master_send_data: same break/sync/PID sequence followed by
 *     HAL_UART_Transmit() for data bytes and checksum.
 *
 * Required HAL includes (add as needed):
 *   #include "stm32h5xx_hal.h"
 *   #include "usart.h"
 *
 * Functions to implement:
 *   int     GO_communication_lin_initialize_interface(void)
 *   int     GO_communication_lin_de_initialize_interface(void)
 *   uint8_t GO_communication_lin_message_scheduler(uint8_t id, uint8_t action)
 *   int     GO_communication_lin_master_retrieve_data(uint8_t id, uint8_t dataLength,
 *               uint8_t data[], uint8_t checksum)
 *   int     GO_communication_lin_master_send_data(uint8_t id, uint8_t dataLength,
 *               uint8_t data[], uint8_t checksum)
 */

/****************************************************************************************
 ****************************************************************************************
 * Linux specific implementations
 ****************************************************************************************
 ****************************************************************************************/
#else

#include <fcntl.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include <asm/ioctls.h>
#include <asm/termbits.h>

#include "pthread.h"

/****************************************************************************************
 * Macro definitions
 ****************************************************************************************/
#define LIN_HDR_SIZE 2

/****************************************************************************************
 * Local data declarations
 ****************************************************************************************/
static struct sllin_tty sllin_tty_data;

static struct sllin sllin_data = {
	.tty = &sllin_tty_data,
};

static struct sllin *sl;

static int lin_descriptor = 0;

/****************************************************************************************
 * Local data definitions
 ****************************************************************************************/
static const unsigned char sllin_id_parity_table[64] = {
 /*0*/  0x80,0xc0,0x40,0x00,0xc0,0x80,0x00,0x40, /*7*/
        0x00,0x40,0xc0,0x80,0x40,0x00,0x80,0xc0, /*15*/
        0x40,0x00,0x80,0xc0,0x00,0x40,0xc0,0x80, /*23*/
        0xc0,0x80,0x00,0x40,0x80,0xc0,0x40,0x00,
        0x00,0x40,0xc0,0x80,0x40,0x00,0x80,0xc0,
        0x80,0xc0,0x40,0x00,0xc0,0x80,0x00,0x40,
        0xc0,0x80,0x00,0x40,0x80,0xc0,0x40,0x00,
        0x40,0x00,0x80,0xc0,0x00,0x40,0xc0,0x80
};

static int tty_set_baudrate(struct sllin_tty *tty, int baudrate);
static int tty_set_mode(struct sllin_tty *tty, int baudrate);

/**************************************************************************************
** \brief     Open and configure the LIN serial interface.
**            Linux:  opens /dev/ttymxc3 at 19200 baud.
**            STM32:  TODO — configure UART peripheral for LIN break detection.
** \return    0 on success, -1 on failure
***************************************************************************************/

int GO_communication_lin_initialize_interface(void)
{
	static uint8_t firsttime = 1;

	if (firsttime == 1) {
		sl = &sllin_data;
		sl->lin_baud = 19200;
		sl->lin_break_baud = (sl->lin_baud * 2) / 3;

		int fd = open("/dev/ttymxc3", O_RDWR);
		if (fd < 0) {
			perror("open()");
			return -1;
		}

		sl->tty->tty_fd = fd;
		tty_set_mode(sl->tty, sl->lin_baud);
		firsttime = 0;
	}
	return 0;
}

/**************************************************************************************
** \brief     Close the LIN serial interface and restore terminal settings.
**            Linux:  restores termios settings and closes file descriptor.
**            STM32:  TODO — disable UART peripheral.
** \return    0 on success, -1 on failure
***************************************************************************************/

int GO_communication_lin_de_initialize_interface(void)
{
	ioctl(sl->tty->tty_fd, TCSETS2, sl->tty->tattr_orig);
	close(sl->tty->tty_fd);
	return 0;
}

/**************************************************************************************
** \brief     Round-robin LIN message scheduler.
**            action 1 = register an ID during initialisation.
**            action 2 = query whether this ID may transmit now.
** \param     id      LIN frame identifier (0-63)
** \param     action  1 = register, 2 = query
** \return    1 if the message is allowed to transmit, 0 otherwise
***************************************************************************************/

uint8_t GO_communication_lin_message_scheduler(uint8_t id, uint8_t action)
{
	static uint8_t storedIds    = 0;
	static uint8_t storedId[10] = {0};
	static uint8_t cnt          = 0;

	if (action == 2) {
		if (id == storedId[0])
			cnt++;
		if (cnt >= storedIds)
			cnt = 0;
		if (storedId[cnt] == id)
			return 1;
	} else if (action == 1) {
		storedId[storedIds++] = id;
	}

	return 0;
}

/**************************************************************************************
** \brief     Send a LIN header and read the slave response.
** \param     id          LIN frame identifier (0-63)
** \param     dataLength  data length code (1 = 2 bytes, 2 = 4 bytes, 3 = 8 bytes)
** \param     data        buffer to store received data bytes
** \param     checksum    1 = classic checksum, 2 = enhanced checksum (ID included)
** \return    0 on success, -1 on failure
***************************************************************************************/

int GO_communication_lin_master_retrieve_data(uint8_t id, uint8_t dataLength,
                                          uint8_t data[], uint8_t checksum)
{
	uint8_t buff[6];

	buff[0] = 0x00; /* Fake break */
	buff[1] = 0x55; /* Sync byte */

	id &= 0x3f;
	id |= sllin_id_parity_table[id];
	buff[2] = id;

	ioctl(sl->tty->tty_fd, TCFLSH, TCIOFLUSH);

	tty_set_baudrate(sl->tty, sl->lin_break_baud);
	write(sl->tty->tty_fd, &buff[0], 1);
	{
		struct timespec sleep_time;
		sleep_time.tv_sec  = 0;
		sleep_time.tv_nsec = ((1000000000ll * 11) / sl->lin_break_baud);
		clock_nanosleep(CLOCK_MONOTONIC, 0, &sleep_time, NULL);
	}

	tty_set_baudrate(sl->tty, sl->lin_baud);
	write(sl->tty->tty_fd, &buff[1], 1);
	write(sl->tty->tty_fd, &buff[2], 1);

	uint8_t dataBytes = 0;
	if      (dataLength == 3) dataBytes = 8;
	else if (dataLength == 2) dataBytes = 4;
	else if (dataLength == 1) dataBytes = 2;

	fcntl(sl->tty->tty_fd, F_SETFL, fcntl(sl->tty->tty_fd, F_GETFL) | O_NONBLOCK);

	/* TODO: replace busy-wait with proper slave-response timeout */
	usleep(5000);

	read(sl->tty->tty_fd, &buff[0], 1); /* Break frame */
	read(sl->tty->tty_fd, &buff[1], 1); /* Sync byte   */
	read(sl->tty->tty_fd, &buff[2], 1); /* Identifier  */

	uint8_t calculatedChecksum = 0;
	if (checksum == 2)
		calculatedChecksum = buff[2];

	for (uint8_t i = 0; i < dataBytes; i++) {
		read(sl->tty->tty_fd, &data[i], 1);
		calculatedChecksum += data[i];
		if (calculatedChecksum > 255)
			calculatedChecksum -= 255;
	}
	return 0;
}

/**************************************************************************************
** \brief     Send a complete LIN frame (header + data + checksum).
** \param     id          LIN frame identifier (0-63)
** \param     dataLength  data length code (1 = 2 bytes, 2 = 4 bytes, 3 = 8 bytes)
** \param     data        data bytes to transmit
** \param     checksum    1 = classic checksum, 2 = enhanced checksum (ID included)
** \return    0 on success, -1 on failure
***************************************************************************************/

int GO_communication_lin_master_send_data(uint8_t id, uint8_t dataLength,
                                      uint8_t data[], uint8_t checksum)
{
	uint8_t dataBytes = 0;
	if      (dataLength == 3) dataBytes = 8;
	else if (dataLength == 2) dataBytes = 4;
	else if (dataLength == 1) dataBytes = 2;

	uint8_t buff[6];
	buff[0] = 0x00; /* Fake break */
	buff[1] = 0x55; /* Sync byte  */

	id &= 0x3f;
	id |= sllin_id_parity_table[id];
	buff[2] = id;

	ioctl(sl->tty->tty_fd, TCFLSH, TCIOFLUSH);

	tty_set_baudrate(sl->tty, sl->lin_break_baud);
	write(sl->tty->tty_fd, &buff[0], 1);
	{
		struct timespec sleep_time;
		sleep_time.tv_sec  = 0;
		sleep_time.tv_nsec = ((1000000000ll * 11) / sl->lin_break_baud);
		clock_nanosleep(CLOCK_MONOTONIC, 0, &sleep_time, NULL);
	}

	tty_set_baudrate(sl->tty, sl->lin_baud);
	write(sl->tty->tty_fd, &buff[1], 1);
	write(sl->tty->tty_fd, &buff[2], 1);

	uint16_t calculatedChecksum = 0;
	if (checksum == 2)
		calculatedChecksum = buff[2];

	for (uint8_t i = 0; i < dataBytes; i++) {
		write(sl->tty->tty_fd, &data[i], 1);
		calculatedChecksum += data[i];
		if (calculatedChecksum > 255)
			calculatedChecksum -= 255;
	}

	calculatedChecksum = (uint8_t)~calculatedChecksum;
	write(sl->tty->tty_fd, &calculatedChecksum, 1);
	return 0;
}

/**************************************************************************************
** \brief     Set the baud rate on the TTY file descriptor using TCSETS2/BOTHER.
** \param     tty       Pointer to the sllin_tty structure holding the fd and termios.
** \param     baudrate  Desired baud rate in bits per second.
** \return    0 on success, -1 on error.
***************************************************************************************/

static int tty_set_baudrate(struct sllin_tty *tty, int baudrate)
{
	tty->tattr.c_ospeed = baudrate;
	tty->tattr.c_ispeed = baudrate;
	tty->tattr.c_cflag &= ~CBAUD;
	tty->tattr.c_cflag |= BOTHER;

	if (ioctl(tty->tty_fd, TCSETS2, &tty->tattr)) {
		perror("ioctl TIOCSSERIAL");
		return -1;
	}
	return 0;
}

/**************************************************************************************
** \brief     Configure the TTY into raw mode suitable for LIN communication and
**            apply the initial baud rate.
** \param     tty       Pointer to the sllin_tty structure holding the fd and termios.
** \param     baudrate  Desired baud rate in bits per second.
** \return    0 on success, -1 on error.
***************************************************************************************/

static int tty_set_mode(struct sllin_tty *tty, int baudrate)
{
	if (!isatty(tty->tty_fd)) {
		fprintf(stderr, "Not a terminal.\n");
		return -1;
	}

	if (ioctl(sl->tty->tty_fd, TCFLSH, TCIOFLUSH) != 0) {
		perror("tcflush");
		return -1;
	}

	if (ioctl(tty->tty_fd, TCGETS2, &tty->tattr_orig) < 0) {
		perror("ioctl TCGETS2");
		return -1;
	}

	if (ioctl(tty->tty_fd, TCGETS2, &tty->tattr) < 0) {
		perror("ioctl TCGETS2");
		return -1;
	}

	tty->tattr.c_cflag = CS8 | CREAD | CLOCAL;
	tty->tattr.c_iflag = 0;
	tty->tattr.c_oflag = NL0 | CR0 | TAB0 | BS0 | VT0 | FF0;
	tty->tattr.c_lflag = 0;

	tty->tattr.c_cc[VINTR]    = '\0';
	tty->tattr.c_cc[VQUIT]    = '\0';
	tty->tattr.c_cc[VERASE]   = '\0';
	tty->tattr.c_cc[VKILL]    = '\0';
	tty->tattr.c_cc[VEOF]     = '\0';
	tty->tattr.c_cc[VTIME]    = '\0';
	tty->tattr.c_cc[VMIN]     = 1;
	tty->tattr.c_cc[VSWTC]    = '\0';
	tty->tattr.c_cc[VSTART]   = '\0';
	tty->tattr.c_cc[VSTOP]    = '\0';
	tty->tattr.c_cc[VSUSP]    = '\0';
	tty->tattr.c_cc[VEOL]     = '\0';
	tty->tattr.c_cc[VREPRINT] = '\0';
	tty->tattr.c_cc[VDISCARD] = '\0';
	tty->tattr.c_cc[VWERASE]  = '\0';
	tty->tattr.c_cc[VLNEXT]   = '\0';
	tty->tattr.c_cc[VEOL2]    = '\0';

	if (ioctl(tty->tty_fd, TCSETS2, &tty->tattr)) {
		perror("ioctl TIOCSSERIAL");
		return -1;
	}

	tty_set_baudrate(tty, baudrate);
	return 0;
}

#endif /* GOCONTROLL_IOT */

/* end of GO_communication_lin.c */
