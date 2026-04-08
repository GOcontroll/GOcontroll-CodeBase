/**************************************************************************************
 * \file   can_receive.c
 * \brief  Receive CAN frames on can0 and print them to the console.
 *
 *         By default a kernel filter is installed that passes only ID 0x601.
 *         Set RX_ID to 0 and RX_MASK to 0 to receive all frames.
 *
 *         This example demonstrates:
 *           - Opening a raw SocketCAN socket on can0
 *           - Installing a receive filter for a specific CAN ID
 *           - Non-blocking frame reception in the application loop
 *           - Clean shutdown on SIGTERM / SIGINT (Ctrl+C)
 *
 *         Prerequisites:
 *           - can0 must be up: ip link set can0 up type can bitrate 250000
 *           - To test together with can_send: run can_send on the same bus
 **************************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "GO_board.h"
#include "print.h"

/* CAN interface and filter */
#define CAN_INTERFACE  "can0"
#define RX_ID          0x601          /* CAN ID to listen for                   */
#define RX_MASK        CAN_SFF_MASK   /* Match all 11-bit standard frames exactly */

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
 * Open a raw SocketCAN socket and install a receive filter.
 * Returns the file descriptor on success, -1 on failure.
 * ------------------------------------------------------------------------- */
static int can_open(const char *iface, canid_t id, canid_t mask)
{
	int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (fd < 0) {
		perror("socket");
		return -1;
	}

	/* Bind to interface */
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

	/* Install receive filter — only matching frames are delivered to this socket.
	 * Pass id=0, mask=0 to receive all frames. */
	if (mask != 0) {
		struct can_filter filter = { .can_id = id, .can_mask = mask };
		if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0) {
			perror("setsockopt CAN_RAW_FILTER");
			close(fd);
			return -1;
		}
		info("Filter: ID 0x%03X  mask 0x%03X\n", id, mask);
	} else {
		info("Filter: none (receiving all frames)\n");
	}

	return fd;
}

/* -------------------------------------------------------------------------
 * main
 * ------------------------------------------------------------------------- */
int main(void)
{
	info("=== CAN receive example ===\n");
	info("Interface : %s\n", CAN_INTERFACE);
	info("RX ID     : 0x%03X  Press Ctrl+C to stop.\n\n", RX_ID);

	GO_board_get_hardware_version();
	GO_board_exit_program(app_terminate);

	can_fd = can_open(CAN_INTERFACE, RX_ID, RX_MASK);
	if (can_fd < 0) {
		err("Failed to open %s — is the interface up?\n", CAN_INTERFACE);
		return 1;
	}
	info("CAN socket opened on %s  — waiting for frames...\n\n", CAN_INTERFACE);

	uint32_t received = 0;

	while (running) {
		struct can_frame frame;
		ssize_t nbytes = recv(can_fd, &frame, sizeof(frame), MSG_DONTWAIT);

		if (nbytes == sizeof(frame)) {
			received++;
			/* Distinguish standard (11-bit) from extended (29-bit) IDs */
			if (frame.can_id & CAN_EFF_FLAG) {
				info("[%4u] RX  0x%08X (EXT)  [%d]  ", received,
				     frame.can_id & CAN_EFF_MASK, frame.can_dlc);
			} else {
				info("[%4u] RX  0x%03X         [%d]  ", received,
				     frame.can_id & CAN_SFF_MASK, frame.can_dlc);
			}
			for (int i = 0; i < frame.can_dlc; i++) {
				info("%02X ", frame.data[i]);
			}
			info("\n");
		} else if (nbytes < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
			if (running) {
				err("recv error: %s\n", strerror(errno));
			}
		}

		usleep(1000); /* 1 ms poll — short to minimise latency */
	}

	info("Received %u frame(s) total.\n", received);
	return 0;
}
