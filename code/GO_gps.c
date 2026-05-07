/**************************************************************************************
 * \file   GO_gps.c
 * \brief  Platform-agnostic GPS implementation for GOcontroll targets.
 *         Compile with -DGOCONTROLL_LINUX for Linux targets or
 *         -DGOCONTROLL_IOT for STM32-based S1 targets.
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

#include "GO_gps.h"

/****************************************************************************************
 * S1 implementation — GPS data is received from the ESP32 via the UART frame protocol.
 * The ESP sends ESPIF_MSG_GPS_DATA (0x40) frames; the ESP driver calls
 * GO_communication_esp_on_gps_data() from UART ISR context to store the data.
 * GO_gps_read() runs in task context and uses a brief interrupt disable to copy
 * the struct atomically — no RTOS mutex may be used in ISR context.
 ****************************************************************************************/
#ifdef GOCONTROLL_IOT

#include "GO_communication_esp.h"
#include "SEGGER_RTT.h"
/* volatile: written from ISR, read from task */
static volatile struct gps_data s_gps_data = {0};

/**************************************************************************************
** \brief     Initialize the GPS subsystem: enable GPS on the ESP so it starts
**            sending ESPIF_MSG_GPS_DATA (0x40) frames.
** \return    none
***************************************************************************************/
void GO_gps_initialize(void) {
	GO_communication_esp_enable_gps(true);
}

/**************************************************************************************
** \brief     Read the latest GPS fix into the caller-supplied structure.
**            Uses a brief interrupt-disable critical section to guarantee a
**            consistent copy — safe to call from task context only.
** \param     out  pointer to a gps_data structure to populate
** \return    none
***************************************************************************************/
void GO_gps_read(struct gps_data *out) {
	__disable_irq();
	*out = s_gps_data;
	__enable_irq();
}

/**************************************************************************************
** \brief     Terminate the GPS subsystem: disable GPS on the ESP.
** \return    none
***************************************************************************************/
void GO_gps_terminate(void) {
	GO_communication_esp_enable_gps(false);
}

/**************************************************************************************
** \brief     Called by the ESP driver when a GPS_DATA frame (0x40) is received.
**            Runs in UART ISR context — no RTOS calls allowed.
**            Converts HHMMSS/DDMMYY integers and stores all fields in s_gps_data.
** \param     gps  Pointer to the decoded GPS frame (valid for this call only).
** \return    none
***************************************************************************************/
void GO_communication_esp_on_gps_data(const EspInterface_GpsData_t *gps) {
	uint32_t t = gps->utc_time;  /* HHMMSS, e.g. 143022 = 14:30:22 */
	uint32_t d = gps->utc_date;  /* DDMMYY, e.g. 230226 = 23-02-2026 */

	s_gps_data.latitude  = gps->latitude;
	s_gps_data.longitude = gps->longitude;
	s_gps_data.altitude  = gps->altitude;
	s_gps_data.velocity  = gps->speed;
	s_gps_data.hour      = (uint8_t)(t / 10000u);
	s_gps_data.minute    = (uint8_t)((t % 10000u) / 100u);
	s_gps_data.second    = (uint8_t)(t % 100u);
	s_gps_data.day       = (uint8_t)(d / 10000u);
	s_gps_data.month     = (uint8_t)((d % 10000u) / 100u);
	s_gps_data.year      = (uint16_t)((d % 100u) + 2000u);
	s_gps_data.valid     = gps->valid;
}

/****************************************************************************************
 * Linux implementation
 ****************************************************************************************/
#elif defined(GOCONTROLL_LINUX)

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

struct gps_thread_args {
	struct gps_data *gps_data;
	pthread_mutex_t *gps_data_lock;
	uint8_t          thread_run;
};

static struct gps_data        s_gps_data;
static pthread_mutex_t        s_gps_mutex;
static pthread_t              s_gps_thread;
static struct gps_thread_args s_gps_args;

static void parse_gps(char *buff, struct gps_data *gps_data,
					  pthread_mutex_t *gps_data_lock) {
	char *first_split = NULL;
	char *newline_split = NULL;
	char *comma_split = NULL;
	char *data_end, *data_comma, *token;
	float longitude, latitude, altitude, velocity;
	int temp_int;
	uint16_t year;
	uint8_t month, day, hour, minute, second;

	data_end = strtok_r(buff, ": ", &first_split);
	if (data_end == NULL) goto no_msg;
	data_end = strtok_r(NULL, ": ", &first_split);
	if (data_end == NULL) goto no_msg;

	data_comma = strtok_r(data_end, "\r", &newline_split);
	if (data_comma == NULL) goto no_msg;

	token = strtok_r(data_comma, ",", &comma_split);
	if (token == NULL) goto no_msg;
	if (strnlen(token, 11) >= 4) {
		latitude = strtof(token + 2, NULL) / 60;
		latitude = latitude + (float)((atoi(token) / 100));
	} else {
		goto no_msg;
	}

	token = strtok_r(NULL, ",", &comma_split);
	if (token == NULL) goto no_msg;
	if (!strcmp(token, "S")) latitude = -latitude;

	token = strtok_r(NULL, ",", &comma_split);
	if (token == NULL) goto no_msg;
	if (strnlen(token, 12) >= 5) {
		longitude = strtof(token + 3, NULL) / 60;
		longitude = longitude + (float)((atoi(token) / 100));
	}

	token = strtok_r(NULL, ",", &comma_split);
	if (token == NULL) goto no_msg;
	if (!strcmp(token, "W")) longitude = -longitude;

	token = strtok_r(NULL, ",", &comma_split);
	if (token == NULL) goto no_msg;
	if (strnlen(token, 6) == 6) {
		temp_int = atoi(token);
		year  = (temp_int % 100) + 2000;
		month = (temp_int % 10000) / 100;
		day   = temp_int / 10000;
	}

	token = strtok_r(NULL, ",", &comma_split);
	if (token == NULL) goto no_msg;
	if (strnlen(token, 6) == 6) {
		temp_int = atoi(token);
		second = (temp_int % 100);
		minute = (temp_int % 10000) / 100;
		hour   = temp_int / 10000;
	}

	token = strtok_r(NULL, ",", &comma_split);
	if (token == NULL) goto no_msg;
	altitude = strtof(token, NULL);

	token = strtok_r(NULL, ",", &comma_split);
	if (token == NULL) goto no_msg;
	velocity = strtof(token, NULL) * 1.852;

	pthread_mutex_lock(gps_data_lock);
	gps_data->longitude = longitude;
	gps_data->latitude  = latitude;
	gps_data->altitude  = altitude;
	gps_data->velocity  = velocity;
	gps_data->year      = year;
	gps_data->month     = month;
	gps_data->day       = day;
	gps_data->hour      = hour;
	gps_data->minute    = minute;
	gps_data->second    = second;
	pthread_mutex_unlock(gps_data_lock);
	return;

no_msg:
	pthread_mutex_lock(gps_data_lock);
	gps_data->longitude = 0;
	gps_data->latitude  = 0;
	gps_data->altitude  = 0;
	gps_data->velocity  = 0;
	gps_data->year      = 0;
	gps_data->month     = 0;
	gps_data->day       = 0;
	gps_data->hour      = 0;
	gps_data->minute    = 0;
	gps_data->second    = 0;
	pthread_mutex_unlock(gps_data_lock);
}

static void __attribute__((unused)) printarr(char *arr, ssize_t count) {
	printf("%s", arr);
	printf("[");
	for (int i = 0; i < count; i++) {
		printf("0x%x,", arr[i]);
	}
	printf("]\n");
}

static int at_command(int fd, char *command, ssize_t command_len, char *buff) {
	ssize_t num_bytes;
	num_bytes = write(fd, command, command_len);
	usleep(1000000);
	num_bytes = read(fd, buff, 256);
	if (num_bytes >= 256) {
		return -ENOMEM;
	}
	buff[num_bytes] = 0;

#if DEBUG == 1
	printarr(buff, num_bytes + 1);
#endif

	return strstr(buff, "OK\r") == NULL;
}

static void *readGpsThread(void *args) {
	struct gps_thread_args *gps_thread_args = (struct gps_thread_args *)args;
	pthread_mutex_t *gps_data_lock = gps_thread_args->gps_data_lock;
	struct gps_data *gps_data      = gps_thread_args->gps_data;
	int tty_fd, ret;
	char buff[256];
	struct termios tty;

	tty_fd = open("/dev/ttymxc1", O_RDWR);
	if (tty_fd < 0) {
		fprintf(stderr, "Could not open tty: %s\n", strerror(tty_fd));
		return 0;
	}

	tcgetattr(tty_fd, &tty);
	tty.c_cflag &= ~PARENB;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;
	tty.c_cflag &= ~CRTSCTS;
	tty.c_cflag |= CREAD | CLOCAL;
	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO;
	tty.c_lflag &= ~ECHOE;
	tty.c_lflag &= ~ECHONL;
	tty.c_lflag &= ~ISIG;
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
	tty.c_oflag &= ~OPOST;
	tty.c_oflag &= ~ONLCR;
	tty.c_cc[VTIME] = 10;
	tty.c_cc[VMIN]  = 2;

	cfsetspeed(&tty, B115200);
	tcsetattr(tty_fd, TCSANOW, &tty);

	do {
		ret = at_command(tty_fd, "AT\r", 4, buff);
		if (ret) fprintf(stderr, "test command failed\n");
	} while (ret);

	do {
		ret = at_command(tty_fd, "AT+CGPS=0\r", 11, buff);
		if (ret) fprintf(stderr, "could not disable gps\n");
	} while (ret);

	do {
		ret = at_command(tty_fd, "AT+CGPS=1\r", 11, buff);
		if (ret) fprintf(stderr, "could not enable gps\n");
	} while (ret);

	while (gps_thread_args->thread_run) {
		if (at_command(tty_fd, "AT+CGPSINFO\r", 13, buff)) {
			continue;
		}
		parse_gps(buff, gps_data, gps_data_lock);
	}

	close(tty_fd);
	return 0;
}

/**************************************************************************************
** \brief     Initialize the GPS subsystem (open UART, create mutex/thread).
** \return    none
***************************************************************************************/
void GO_gps_initialize(void) {
	pthread_mutex_init(&s_gps_mutex, NULL);
	s_gps_args.gps_data      = &s_gps_data;
	s_gps_args.gps_data_lock = &s_gps_mutex;
	s_gps_args.thread_run    = 1;
	pthread_create(&s_gps_thread, NULL, readGpsThread, &s_gps_args);
}

/**************************************************************************************
** \brief     Read the latest GPS fix into the caller-supplied structure.
** \param     out  pointer to a gps_data structure to populate
** \return    none
***************************************************************************************/
void GO_gps_read(struct gps_data *out) {
	pthread_mutex_lock(&s_gps_mutex);
	*out = s_gps_data;
	pthread_mutex_unlock(&s_gps_mutex);
}

/**************************************************************************************
** \brief     Terminate the GPS subsystem and release all resources.
** \return    none
***************************************************************************************/
void GO_gps_terminate(void) {
	s_gps_args.thread_run = 0;
	pthread_join(s_gps_thread, NULL);
	pthread_mutex_destroy(&s_gps_mutex);
}

#endif
