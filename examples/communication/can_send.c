/**************************************************************************************
 * \file   can_send.c
 * \brief  Send a CAN frame on can0 once per second.
 *
 *         Frame:  ID 0x601 | Standard (11-bit) | DLC 8
 *                 Data:  0x01 0x02 0x03 0x04 0x05 0x06 0x07 0x08
 *
 *         The data byte 0 is incremented on every transmission so you can
 *         verify reception on the other end (e.g. with candump or can_receive).
 *
 *         This example demonstrates:
 *           - Opening a raw SocketCAN socket on can0
 *           - Building and transmitting a CAN frame
 *           - Clean shutdown on SIGTERM / SIGINT (Ctrl+C)
 *
 *         Prerequisites:
 *           - can0 must be up and configured: ip link set can0 up type can bitrate 250000
 **************************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "GO_board.h"
#include "print.h"

/* CAN interface and message parameters */
#define CAN_INTERFACE  "can0"
#define CAN_TX_ID      0x601
#define CAN_DLC        8

/* Send interval: 100 cycles × 10 ms = 1 s */
#define SEND_INTERVAL_CYCLES 100

static int can_fd = -1;
static volatile bool running = true;

/* -------------------------------------------------------------------------
 * Shutdown callback — registered via GO_board_exit_program().
 * ------------------------------------------------------------------------- */
static void app_terminate(void)
{
	running = false;
	info("Shutting down — closing CAN socket\n");
	if (can_fd >= 0) {
		close(can_fd);
		can_fd = -1;
	}
}

/* -------------------------------------------------------------------------
 * Open a raw SocketCAN socket bound to CAN_INTERFACE.
 * Returns the file descriptor on success, -1 on failure.
 * ------------------------------------------------------------------------- */
static int can_open(const char *iface)
{
	int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (fd < 0) {
		perror("socket");
		return -1;
	}

	struct ifreq ifr;
	strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);
	ifr.ifr_name[IFNAMSIZ - 1] = '\0';
	if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
		perror("ioctl SIOCGIFINDEX");
		close(fd);
		return -1;
	}

	struct sockaddr_can addr;
	memset(&addr, 0, sizeof(addr));
	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind");
		close(fd);
		return -1;
	}

	return fd;
}

/* -------------------------------------------------------------------------
 * main
 * ------------------------------------------------------------------------- */
int main(void)
{
	info("=== CAN send example ===\n");
	info("Interface : %s\n", CAN_INTERFACE);
	info("TX ID     : 0x%03X | DLC %d\n", CAN_TX_ID, CAN_DLC);
	info("Interval  : 1 s    Press Ctrl+C to stop.\n\n");

	GO_board_get_hardware_version();
	GO_board_exit_program(app_terminate);

	can_fd = can_open(CAN_INTERFACE);
	if (can_fd < 0) {
		err("Failed to open %s — is the interface up?\n", CAN_INTERFACE);
		return 1;
	}
	info("CAN socket opened on %s\n\n", CAN_INTERFACE);

	/* Build the frame once; only data[0] changes each cycle */
	struct can_frame frame;
	memset(&frame, 0, sizeof(frame));
	frame.can_id  = CAN_TX_ID;
	frame.can_dlc = CAN_DLC;
	frame.data[0] = 0x00; /* counter, incremented each transmission */
	frame.data[1] = 0x02;
	frame.data[2] = 0x03;
	frame.data[3] = 0x04;
	frame.data[4] = 0x05;
	frame.data[5] = 0x06;
	frame.data[6] = 0x07;
	frame.data[7] = 0x08;

	int    cycle  = 0;
	uint32_t sent = 0;

	while (running) {
		if (++cycle >= SEND_INTERVAL_CYCLES) {
			cycle = 0;
			frame.data[0]++;

			ssize_t written = write(can_fd, &frame, sizeof(frame));
			if (written != sizeof(frame)) {
				err("Write error on %s\n", CAN_INTERFACE);
			} else {
				sent++;
				info("[%4u] TX  0x%03X  [%d]  "
				     "%02X %02X %02X %02X %02X %02X %02X %02X\n",
				     sent, frame.can_id, frame.can_dlc,
				     frame.data[0], frame.data[1], frame.data[2], frame.data[3],
				     frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
			}
		}

		usleep(10000); /* 10 ms cycle */
	}

	return 0;
}
