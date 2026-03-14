/**************************************************************************************
 * \file   GO_communication_modules.c
 * \brief  Module communication implementation for GOcontroll hardware.
 *         Handles SPI send/receive, chip selects, module reset and bootloader escape.
 *
 *         Platform selection via preprocessor define:
 *           GOCONTROLL_IOT  →  STM32H5 (Moduline IOT)
 *           (default)       →  Linux/IMX8 (Moduline IV / Moduline Mini)
 * \internal
 *----------------------------------------------------------------------------------------
 *                          C O P Y R I G H T
 *----------------------------------------------------------------------------------------
 * Copyright 2025 (c) by GOcontroll http://www.gocontroll.com All rights reserved
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
#include "GO_communication_modules.h"

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include "print.h"

/****************************************************************************************
 * Include files — platform-specific
 ****************************************************************************************/
#ifdef GOCONTROLL_IOT

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gpio.h"
#include "spi.h"
#include "stm32h5xx_hal.h"
#include "SEGGER_RTT.h"

#elif defined(GOCONTROLL_LINUX)

#define _DEFAULT_SOURCE		/* For usleep() */
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>

#endif /* GOCONTROLL_IOT / GOCONTROLL_LINUX */

/****************************************************************************************
 * Macro definitions
 ****************************************************************************************/
#define LOW		0
#define HIGH	1

/****************************************************************************************
 * Data declarations
 ****************************************************************************************/
_hardwareConfig hardwareConfig;

/****************************************************************************************
 * Linux-specific internal helpers
 ****************************************************************************************/
#ifdef GOCONTROLL_LINUX

typedef struct {
	char *channel;
} _moduleSpi;

static _moduleSpi moduleSpi[8] = {
	{"/dev/spidev1.0"}, {"/dev/spidev1.1"}, {"/dev/spidev2.0"},
	{"/dev/spidev2.1"}, {"/dev/spidev2.2"}, {"/dev/spidev2.3"},
	{"/dev/spidev0.0"}, {"/dev/spidev0.1"},
};

static uint8_t	spi_mode	= 0;
static uint8_t	spi_bits	= 8;
static uint32_t	spi_speed	= 2000000;

/**************************************************************************************
** \brief     Open (lazily) and return the spidev file descriptor for a module slot.
**            Configures SPI mode, bits-per-word, and speed on first open.
** \param     moduleSlot  Module slot index (0-7).
** \return    File descriptor for the SPI device, or -1 on error.
***************************************************************************************/
static int GO_communication_modules_spi_device(uint8_t moduleSlot) {
	static int spiDevice[8] = {0};

	if (spiDevice[moduleSlot] == 0) {
		spiDevice[moduleSlot] = open(moduleSpi[moduleSlot].channel, O_RDWR);

		ioctl(spiDevice[moduleSlot], SPI_IOC_WR_MODE, &spi_mode);
		ioctl(spiDevice[moduleSlot], SPI_IOC_WR_BITS_PER_WORD, &spi_bits);
		ioctl(spiDevice[moduleSlot], SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
	}

	return spiDevice[moduleSlot];
}

/**************************************************************************************
** \brief     Open (lazily) and return the file descriptor for the reset LED sysfs
**            entry of the given module slot.
** \param     moduleSlot  Module slot index (0-7).
** \return    File descriptor for the reset GPIO sysfs entry, or -1 on error.
***************************************************************************************/
static int GO_communication_modules_module_reset(uint8_t moduleSlot) {
	static int moduleReset[8] = {0};

	if (moduleReset[moduleSlot] == 0) {
		char path[40];

		snprintf(path, 40, "/sys/class/leds/ResetM-%d/brightness",
				 moduleSlot + 1);

		moduleReset[moduleSlot] = open(path, O_WRONLY);

		if (-1 == moduleReset[moduleSlot]) {
			fprintf(stderr, "Error GPIO write module reset!\n");
			return (-1);
		}
	}

	return moduleReset[moduleSlot];
}

#endif /* GOCONTROLL_LINUX */

/****************************************************************************************
 ****************************************************************************************
 * Platform-independent implementations
 ****************************************************************************************
 ****************************************************************************************/

/**************************************************************************************
** \brief     Calculate the checksum of an SPI message.
** \param     array   Buffer containing the SPI message bytes.
** \param     length  Number of bytes to sum.
** \return    The checksum byte.
***************************************************************************************/
uint8_t GO_communication_modules_checksum_calculator(uint8_t *array,
													uint8_t length) {
	uint8_t checkSum = 0;
	for (uint8_t pointer = 0; pointer < length; pointer++) {
		checkSum += array[pointer];
	}
	return checkSum;
}

/****************************************************************************************/

/**************************************************************************************
** \brief     Register a detected module in the hardware config table.
** \param     slot  Module slot index (0-7).
** \param     rx    Bootloader RX buffer containing firmware info.
** \return    none
***************************************************************************************/
void GO_communication_modules_register_module(uint8_t slot, uint8_t *rx) {
	memcpy(hardwareConfig.moduleOccupancy[slot], &rx[6], 7);
	info("module %d registered, firmware: [ ", slot + 1);
	for (uint8_t i = 0; i < 7; i++) {
		info("%d, ", hardwareConfig.moduleOccupancy[slot][i]);
	}
	info("]\n");
}

/****************************************************************************************/

/**************************************************************************************
** \brief     Initialize a module in a specific slot.
** \param     moduleslot  Slot index (0-based).
** \return    0 on success, -ENODEV on failure.
***************************************************************************************/
int GO_communication_modules_initialize(uint8_t moduleslot) {
	int res;
	if (moduleslot >= hardwareConfig.moduleNumber) {
		return -ENODEV;
	}

	for (uint8_t i = 0; i < 5; i++) {	
		GO_communication_modules_reset_state_module(moduleslot, 1);
		GO_communication_modules_delay_1ms(2);
		GO_communication_modules_reset_state_module(moduleslot, 0);
		GO_communication_modules_delay_1ms(2);
		
		uint8_t dataTxBoot[BOOTMESSAGELENGTHCHECK] = {0};
		uint8_t dataRxBoot[BOOTMESSAGELENGTHCHECK] = {0};

		res = GO_communication_modules_escape_from_bootloader(
			moduleslot, dataTxBoot, dataRxBoot);

		dbg("bootloader:\n[");
		for (uint8_t j = 0; j < BOOTMESSAGELENGTH; j++) {
			dbg("%d, ", dataRxBoot[j]);
		}
		dbg("]\n");

		/* checksum faulty, but a module seems to be there — retry */
		if (res &&
			(dataRxBoot[0] == 9 || dataRxBoot[1] == BOOTMESSAGELENGTH - 1 ||
			 dataRxBoot[2] == 9)) {
			dbg("checksum error\n");
			continue;
		}
		/* checksum correct but message doesn't come from the bootloader */
		if (!res &&
			(dataRxBoot[0] != 9 || dataRxBoot[1] != BOOTMESSAGELENGTH - 1 ||
			 dataRxBoot[2] != 9)) {
			dbg("message incorrect\n");
			continue;
		}
		/* no module present — don't loop multiple times */
		if (dataRxBoot[0] == 255) {
			dbg("no module present\n");
			break;
		}
		uint8_t dataTxFirm[BOOTMESSAGELENGTHCHECK] = {0};
		uint8_t dataRxFirm[BOOTMESSAGELENGTHCHECK] = {0};
		GO_communication_modules_delay_1ms(2);
		res = GO_communication_modules_escape_from_bootloader(
			moduleslot, dataTxFirm, dataRxFirm);
		dbg("firmware:\n[");
		for (uint8_t j = 0; j <= dataRxFirm[1]; j++) {
			dbg("%d, ", dataRxFirm[j]);
		}
		dbg("]\n");
		if (!res && dataRxFirm[0] != 9 && dataRxFirm[2] != 9 &&
			dataRxFirm[1] != 0) {
			GO_communication_modules_register_module(moduleslot, dataRxBoot);
			GO_communication_modules_delay_1ms(4);
			return 0;
		}
	}
	return -ENODEV;
}

/****************************************************************************************
 ****************************************************************************************
 * Platform-specific implementations
 ****************************************************************************************
 ****************************************************************************************/

/****************************************************************************************/

/**************************************************************************************
** \brief     Delay execution for a number of milliseconds.
** \param     times  Number of milliseconds to sleep.
** \return    none
***************************************************************************************/
void GO_communication_modules_delay_1ms(uint32_t times) {
#ifdef GOCONTROLL_IOT
	if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
		osDelay(times);
	} else {
		/* DWT cycle-counter busy-wait — no interrupt dependency */
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
		DWT->CTRL       |= DWT_CTRL_CYCCNTENA_Msk;
		uint32_t clk_per_ms = SystemCoreClock / 1000U;
		while (times--) {
			uint32_t t0 = DWT->CYCCNT;
			while ((DWT->CYCCNT - t0) < clk_per_ms);
		}
	}
#elif defined(GOCONTROLL_LINUX)
	usleep(times * 1000);
#endif
}

/****************************************************************************************/

/**************************************************************************************
** \brief     Set the state of the reset pin of a module.
** \param     module  Slot index (0-7).
** \param     state   Desired pin state: 1 = assert reset, 0 = release.
** \return    0 on success, -1 on failure.
***************************************************************************************/
int8_t GO_communication_modules_reset_state_module(uint8_t module, uint8_t state) {
#ifdef GOCONTROLL_IOT
	if (module == 0) {
		HAL_GPIO_WritePin(MOD1_RESET_GPIO_Port, MOD1_RESET_Pin, !state);
	} else if (module == 1) {
		HAL_GPIO_WritePin(MOD2_RESET_GPIO_Port, MOD2_RESET_Pin, !state);
	}
	return 0;
#elif defined(GOCONTROLL_LINUX)
	static const char s_values_str[] = "01";

	if (1 != write(GO_communication_modules_module_reset(module),
				   &s_values_str[LOW == state ? 0 : 1], 1)) {
		fprintf(stderr, "Reset pin fail for module %d\n", module);
		return (-1);
	}
	return (0);
#endif
}

/****************************************************************************************/

/**************************************************************************************
** \brief     Get a module out of its bootloader state.
** \param     module  Slot index (0-7).
** \param     dataTx  Transmit buffer.
** \param     dataRx  Receive buffer.
** \return    0 on success, -1 on failure.
***************************************************************************************/
int GO_communication_modules_escape_from_bootloader(uint8_t module,
												  uint8_t *dataTx,
												  uint8_t *dataRx) {
	/* Platform-independent: build bootloader escape message */
	dataTx[0] = 19;
	dataTx[1] = BOOTMESSAGELENGTH - 1;
	*(uint16_t *)&dataTx[2] = 19;
	dataTx[BOOTMESSAGELENGTH - 1] = GO_communication_modules_checksum_calculator(
		&dataTx[0], BOOTMESSAGELENGTH - 1);

	/* Platform-specific: transmit and receive */
#ifdef GOCONTROLL_IOT
	if (module == 0) {
		HAL_GPIO_WritePin(SPI_MOD1_CS_GPIO_Port, SPI_MOD1_CS_Pin,
						  GPIO_PIN_RESET);
	} else if (module == 1) {
		HAL_GPIO_WritePin(SPI_MOD2_CS_GPIO_Port, SPI_MOD2_CS_Pin,
						  GPIO_PIN_RESET);
	}

	HAL_SPI_TransmitReceive(&hspi1, &dataTx[0], &dataRx[0],
							BOOTMESSAGELENGTHCHECK, 500);

	if (module == 0) {
		HAL_GPIO_WritePin(SPI_MOD1_CS_GPIO_Port, SPI_MOD1_CS_Pin, GPIO_PIN_SET);
	} else if (module == 1) {
		HAL_GPIO_WritePin(SPI_MOD2_CS_GPIO_Port, SPI_MOD2_CS_Pin, GPIO_PIN_SET);
	}

	if (dataRx[1] <= BOOTMESSAGELENGTHCHECK) {
#elif defined(GOCONTROLL_LINUX)
	struct spi_ioc_transfer tr = {
		.tx_buf		= (long int)&dataTx[0],
		.rx_buf		= (long int)&dataRx[0],
		.len		= BOOTMESSAGELENGTHCHECK,
		.delay_usecs	= 10,
		.speed_hz	= 0,
		.bits_per_word	= 0,
	};

	ioctl(GO_communication_modules_spi_device(module), SPI_IOC_MESSAGE(1), &tr);

	if (dataRx[1] < BOOTMESSAGELENGTHCHECK) {
#endif
		if (GO_communication_modules_checksum_calculator(
				&dataRx[0], dataRx[1]) == dataRx[dataRx[1]]) {
			return 0;
		}
	}

	return -1;
}

/****************************************************************************************/

/**************************************************************************************
** \brief     Send data over SPI to a module.
** \param     command     SPI command byte.
** \param     dataLength  Total message length in bytes.
** \param     id1         Identifier byte 1.
** \param     id2         Identifier byte 2.
** \param     id3         Identifier byte 3.
** \param     id4         Identifier byte 4.
** \param     module      Target module slot (0-7).
** \param     dataTx      Transmit buffer.
** \param     delay       Pre-transmission delay in microseconds.
** \return    0 on success, -1 on failure.
***************************************************************************************/
int GO_communication_modules_send_spi(uint8_t command, uint8_t dataLength,
									 uint8_t id1, uint8_t id2, uint8_t id3,
									 uint8_t id4, uint8_t module,
									 uint8_t *dataTx, uint32_t delay) {
	/* Platform-independent: build message header and checksum */
	dataTx[0] = command;
	dataTx[1] = dataLength - 1;
	dataTx[2] = id1;
	dataTx[3] = id2;
	dataTx[4] = id3;
	dataTx[5] = id4;
	dataTx[dataLength - 1] =
		GO_communication_modules_checksum_calculator(&dataTx[0], dataLength - 1);

	/* Platform-specific: transmit */
#ifdef GOCONTROLL_IOT
	if (module == 0) {
		HAL_GPIO_WritePin(SPI_MOD1_CS_GPIO_Port, SPI_MOD1_CS_Pin,
						  GPIO_PIN_RESET);
	} else if (module == 1) {
		HAL_GPIO_WritePin(SPI_MOD2_CS_GPIO_Port, SPI_MOD2_CS_Pin,
						  GPIO_PIN_RESET);
	}

	/* Round delay in µs up to ms for HAL */
	GO_communication_modules_delay_1ms(delay / 1000 + (delay % 1000 != 0));
	HAL_SPI_Transmit(&hspi1, &dataTx[0], dataLength + MESSAGEOVERLENGTH, 500);

	if (module == 0) {
		HAL_GPIO_WritePin(SPI_MOD1_CS_GPIO_Port, SPI_MOD1_CS_Pin, GPIO_PIN_SET);
	} else if (module == 1) {
		HAL_GPIO_WritePin(SPI_MOD2_CS_GPIO_Port, SPI_MOD2_CS_Pin, GPIO_PIN_SET);
	}
#elif defined(GOCONTROLL_LINUX)
	usleep((uint32_t)delay);
	write(GO_communication_modules_spi_device(module), &dataTx[0],
		  dataLength + MESSAGEOVERLENGTH);
#endif

	return 0;
}

/****************************************************************************************/

/**************************************************************************************
** \brief     Send an SPI message to a module and receive the response.
** \param     command     SPI command byte.
** \param     dataLength  Total message length in bytes.
** \param     id1         Identifier byte 1.
** \param     id2         Identifier byte 2.
** \param     id3         Identifier byte 3.
** \param     id4         Identifier byte 4.
** \param     module      Target module slot (0-7).
** \param     dataTx      Transmit buffer.
** \param     dataRx      Receive buffer.
** \return    0 if checksum valid, -1 otherwise.
***************************************************************************************/
int GO_communication_modules_send_receive_spi(uint8_t command, uint8_t dataLength,
											uint8_t id1, uint8_t id2,
											uint8_t id3, uint8_t id4,
											uint8_t module, uint8_t *dataTx,
											uint8_t *dataRx) {
	/* Platform-independent: build message header and checksum */
	dataTx[0] = command;
	dataTx[1] = dataLength - 1;
	dataTx[2] = id1;
	dataTx[3] = id2;
	dataTx[4] = id3;
	dataTx[5] = id4;
	dataTx[dataLength - 1] =
		GO_communication_modules_checksum_calculator(&dataTx[0], dataLength - 1);

	/* Platform-specific: transmit and receive */
#ifdef GOCONTROLL_IOT
	if (module == 0) {
		HAL_GPIO_WritePin(SPI_MOD1_CS_GPIO_Port, SPI_MOD1_CS_Pin,
						  GPIO_PIN_RESET);
	} else if (module == 1) {
		HAL_GPIO_WritePin(SPI_MOD2_CS_GPIO_Port, SPI_MOD2_CS_Pin,
						  GPIO_PIN_RESET);
	}

	HAL_SPI_TransmitReceive(&hspi1, &dataTx[0], &dataRx[0],
							dataLength + MESSAGEOVERLENGTH, 500);

	if (module == 0) {
		HAL_GPIO_WritePin(SPI_MOD1_CS_GPIO_Port, SPI_MOD1_CS_Pin, GPIO_PIN_SET);
	} else if (module == 1) {
		HAL_GPIO_WritePin(SPI_MOD2_CS_GPIO_Port, SPI_MOD2_CS_Pin, GPIO_PIN_SET);
	}
#elif defined(GOCONTROLL_LINUX)
	/* Reset essential values to erase any stale message */
	dataRx[0] = 0;
	dataRx[dataLength - 1] = 0;

	struct spi_ioc_transfer tr = {
		.tx_buf		= (long int)&dataTx[0],
		.rx_buf		= (long int)&dataRx[0],
		.len		= dataLength + MESSAGEOVERLENGTH,
		.delay_usecs	= 10,
		.speed_hz	= 0,
		.bits_per_word	= 0,
	};

	ioctl(GO_communication_modules_spi_device(module), SPI_IOC_MESSAGE(1), &tr);
#endif

	/* Platform-independent: verify response checksum */
	if (dataRx[1] == dataLength - 1) {
		if (GO_communication_modules_checksum_calculator(
				&dataRx[0], dataLength - 1) == dataRx[dataLength - 1]) {
			return 0;
		}
	}
	return -1;
}

/****************************************************************************************
 ****************************************************************************************
 * STM32H5 (GOCONTROLL_IOT) specific implementations
 ****************************************************************************************
 ****************************************************************************************/
#ifdef GOCONTROLL_IOT

/**************************************************************************************
** \brief     Delay execution using the RTOS scheduler (non-blocking for other tasks).
** \param     times  Number of milliseconds to delay.
** \return    none
***************************************************************************************/
void GO_communication_modules_delay_1ms_os(uint32_t times) { HAL_Delay(times); }

#endif /* GOCONTROLL_IOT */

/* end of GO_communication_modules.c */
