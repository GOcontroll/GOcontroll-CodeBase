/**************************************************************************************
 * \file   GO_communication_modules.c
 * \brief  Module communication implementation for GOcontroll hardware.
 *         Handles SPI send/receive, chip selects, module reset and bootloader escape.
 *
 *         Platform selection via preprocessor define:
 *           GOCONTROLL_IOT  →  STM32H5 (Moduline S1)
 *           (default)       →  Linux/IMX8 (Moduline L4 / Moduline M1)
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

/* Timing for the bootloader-escape handshake in GO_communication_modules_initialize().
 * Two INDEPENDENT constraints (see that function for the mechanism):
 *   - MODULE_BOOT_SETTLE_MS: settle after reset-release before the first escape. Must be
 *     long enough for the module's SPI-DMA re-arm (runs off a 1 ms poll task), yet short
 *     enough that the escape still lands inside the bootloader's open window after reset —
 *     miss it and the module boots into its application and answers with a non-9,45,9 frame.
 *   - MODULE_ESCAPE_GAP_MS: rest between the two back-to-back escapes so the slave can
 *     re-arm its DMA before being clocked again (larger matters for the heavily-loaded
 *     output module).
 * Starting point: 1 ms each — tune against the module firmware's real window / re-arm time. */
/* 100 ms: the output module needs ~100 ms after reset-release to reach its bootloader
 * and drive MISO — clocking it earlier reads a not-ready slave (all-0x01 bytes). Measured
 * on the logic analyzer: a reset-release -> escape gap of ~110 ms reliably read 9,45,9. */
#define MODULE_BOOT_SETTLE_MS	100u
#define MODULE_ESCAPE_GAP_MS	10u

/* Settle after releasing reset before the pad level is sampled / the bus is touched.
 * (MODULE_RESET_ASSERT_MS lives in the header — callers outside this file need it too.) */
#define MODULE_RESET_SETTLE_MS	2u

/****************************************************************************************
 * Data declarations
 ****************************************************************************************/
_hardwareConfig hardwareConfig;

#ifdef GOCONTROLL_IOT
static osSemaphoreId_t s_spi_done = NULL;
/* Result of the last DMA transfer, written by the HAL completion/error callbacks
 * and read back by GO_communication_modules_spi_wait(): 0 = completed OK,
 * -1 = the peripheral reported an error (OVR/MODF/FRE/DMA). */
static volatile int s_spi_status = 0;

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	(void)hspi;
	s_spi_status = 0;
	osSemaphoreRelease(s_spi_done);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	(void)hspi;
	s_spi_status = 0;
	osSemaphoreRelease(s_spi_done);
}

/* Without this the HAL calls the empty __weak error callback, the completion
 * semaphore is never released, and every module transfer burns the full 500 ms
 * timeout. Flag the error and wake the waiting task so it can abort the bus. */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
	(void)hspi;
	s_spi_status = -1;
	osSemaphoreRelease(s_spi_done);
}

/* Drain any stale completion token left by a previously timed-out transfer, so
 * the next acquire waits for THIS transfer rather than returning immediately
 * (which would drop CS mid-frame). */
static void GO_communication_modules_spi_drain(void) {
	while (osSemaphoreAcquire(s_spi_done, 0) == osOK) {
	}
	/* Also flush any stale byte left in the SPI hardware RX FIFO. A residual RX byte at
	 * the start of the next transfer makes the DMA read [stale, frame...]: the whole frame
	 * lands one byte late (observed: leading 0x01, e.g. 1,43,2,22,4,1 read as 1,1,43,2,22,4,1)
	 * and the module checksum fails. This is the warm output-module "byte shift" — the DMA
	 * completes but the data is offset by one. */
	while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_RXP)) {
		(void)(*(__IO uint8_t *)&hspi1.Instance->RXDR);
	}
}

/* Block until the running DMA transfer signals completion. On timeout or a
 * reported bus error, abort the SPI so the peripheral leaves its BUSY/ERROR
 * state and returns to READY — otherwise the next HAL_SPI_*_DMA() call returns
 * HAL_BUSY forever and the module bus stays dead. Returns 0 on success, -1
 * otherwise. */
/* A module SPI frame is < 1 ms on the wire (≤ 61 bytes @ ≥ 500 kHz) plus the
 * module's ~150 us turnaround, so a healthy transfer completes in well under a
 * millisecond. The old 500 ms guard meant a NON-responding module (bus held / no
 * clocking) stalled every single transfer for half a second while slot_1 kept the
 * bus mutex held — starving slot_2 out of the loop. A short guard fails fast so one
 * dead module can no longer monopolise the shared bus. */
#define SPI_XFER_TIMEOUT_MS  25u
static int GO_communication_modules_spi_wait(void) {
	if (osSemaphoreAcquire(s_spi_done, SPI_XFER_TIMEOUT_MS) != osOK) {
		err("SPI transfer timed out — aborting module bus\n");
		HAL_SPI_Abort(&hspi1);
		return -1;
	}
	if (s_spi_status != 0) {
		err("SPI transfer error — aborting module bus\n");
		HAL_SPI_Abort(&hspi1);
		return -1;
	}
	return 0;
}

/* Prime a module's SPI slave (ports the Linux GocontrollProcessorboard DummySpiSend()): one
 * throwaway CS-asserted transfer so the first REAL transfer to the module is not the one that
 * lands a byte late (the STM32H5 output-module "byte shift"). Learned on hardware:
 *   - CS must be ASSERTED (a CS-high peripheral-only prime does NOT re-phase the slave).
 *   - It only re-phases when it is the FIRST transfer to a RUNNING (app) slave — i.e. right
 *     after a reset+boot. Sending it into a module still in its BOOTLOADER (cold start) corrupts
 *     detection, so this MUST NOT run on the cold reset+detect path — only in the warm adopt
 *     path, after a fresh reset that reboots the app. Transmit-only; response irrelevant. */
void GO_communication_modules_dummy_spi(uint8_t module) {
	uint8_t dummy[5] = {1, 2, 3, 4, 5};
	if (module == 0) {
		HAL_GPIO_WritePin(SPI_MOD1_CS_GPIO_Port, SPI_MOD1_CS_Pin, GPIO_PIN_RESET);
		(void)HAL_SPI_Transmit(&hspi1, &dummy[0], 5, 100);
		HAL_GPIO_WritePin(SPI_MOD1_CS_GPIO_Port, SPI_MOD1_CS_Pin, GPIO_PIN_SET);
	} else if (module == 1) {
		HAL_GPIO_WritePin(SPI_MOD2_CS_GPIO_Port, SPI_MOD2_CS_Pin, GPIO_PIN_RESET);
		(void)HAL_SPI_Transmit(&hspi1, &dummy[0], 5, 100);
		HAL_GPIO_WritePin(SPI_MOD2_CS_GPIO_Port, SPI_MOD2_CS_Pin, GPIO_PIN_SET);
	}
}
#endif /* GOCONTROLL_IOT */

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

#ifdef GOCONTROLL_IOT
	if (s_spi_done == NULL) {
		s_spi_done = osSemaphoreNew(1, 0, NULL);
	}
#endif

	/* Reset the module EXACTLY ONCE — not on every retry iteration. The module's
	 * bootloader opens a short window after a single reset and leaves it only on
	 * a *clean* escape command; a corrupt/failed escape keeps the module in the
	 * bootloader, so the handshake below can simply be retried. Re-asserting reset
	 * on every iteration instead throws the module back into the bootloader and
	 * fights its own boot timing — observed to intermittently block detection of
	 * the slot-1 module while slot 2 (which succeeded on the first try) worked.
	 * Retry only the bootloader-escape handshake below; never the reset. */
	/* Single shared reset implementation — see MODULE_RESET_ASSERT_MS for why the pulse
	 * is as wide as it is, and GO_communication_modules_reset_module() for the pad-level
	 * verification. A failing return means the reset line never moved; detection below is
	 * then guaranteed to read live application frames, so say so instead of silently
	 * spending five retries on it. */
	if (GO_communication_modules_reset_module(moduleslot, MODULE_RESET_ASSERT_MS) != 0) {
		err("module %d: reset line fault — bootloader detection will not work\n",
			moduleslot + 1);
	}

	for (uint8_t i = 0; i < 5; i++) {
		uint8_t dataTxBoot[BOOTMESSAGELENGTHCHECK] = {0};
		uint8_t dataRxBoot[BOOTMESSAGELENGTHCHECK] = {0};

		/* Let the slave finish re-arming its SPI-DMA from the reset / previous escape
		 * before clocking the next transfer. The module defers its DMA re-arm to a
		 * 1 ms polling task; under heavy on-module interrupt load (e.g. the output
		 * module's 10 us control + supply-ramp timers) that re-arm lags, and clocking
		 * a not-yet-re-armed slave makes it shift out mis-aligned data (a 1-bit slip,
		 * e.g. module id 20,20,2 read back as 40,40,4). Spacing the escapes out keeps
		 * the slave in step. */
		GO_communication_modules_delay_1ms(MODULE_BOOT_SETTLE_MS);

		res = GO_communication_modules_escape_from_bootloader(
			moduleslot, dataTxBoot, dataRxBoot);

		dbg("slot %d bootloader:\n[", moduleslot + 1);
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
		/* Same re-arm settle as above, between the two back-to-back escapes. */
		GO_communication_modules_delay_1ms(MODULE_ESCAPE_GAP_MS);
		res = GO_communication_modules_escape_from_bootloader(
			moduleslot, dataTxFirm, dataRxFirm);
		dbg("slot %d firmware:\n[", moduleslot + 1);
		for (uint8_t j = 0; j <= dataRxFirm[1]; j++) {
			dbg("%d, ", dataRxFirm[j]);
		}
		dbg("]\n");
		if (!res && dataRxFirm[0] != 9 && dataRxFirm[2] != 9 &&
			dataRxFirm[1] != 0) {
			GO_communication_modules_register_module(moduleslot, dataRxBoot);
			GO_communication_modules_delay_1ms(3);
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

#ifdef GOCONTROLL_IOT
/* Read the ACTUAL pad level of a module's reset line. The pin is an open-drain output,
 * so reading IDR reports what the pad is really doing rather than what was written to
 * ODR — which is exactly what is needed to prove the line moves at all. */
static GPIO_PinState GO_communication_modules_reset_pin_level(uint8_t module) {
	if (module == 0) {
		return HAL_GPIO_ReadPin(MOD1_RESET_GPIO_Port, MOD1_RESET_Pin);
	} else if (module == 1) {
		return HAL_GPIO_ReadPin(MOD2_RESET_GPIO_Port, MOD2_RESET_Pin);
	}
	return GPIO_PIN_SET;
}

/* Raw GPIO register dump for a module's reset pin. HAL_GPIO_ReadPin() reports IDR, which
 * reads 0 both when the pad is genuinely pulled low AND when the pin sits in analog mode
 * (schmitt trigger disabled) — so IDR alone cannot distinguish "driving low" from "not
 * driving at all". MODER/OTYPER/ODR settle that question. */
static void GO_communication_modules_reset_pin_dump(uint8_t module, const char *when) {
	GPIO_TypeDef *port;
	uint16_t      mask;

	if (module == 0) {
		port = MOD1_RESET_GPIO_Port;
		mask = MOD1_RESET_Pin;
	} else if (module == 1) {
		port = MOD2_RESET_GPIO_Port;
		mask = MOD2_RESET_Pin;
	} else {
		return;
	}

	uint32_t bit = 0u;
	for (uint16_t m = mask; m > 1u; m >>= 1) {
		bit++;
	}

	/* mode: 0=input 1=output 2=alternate 3=analog | otype: 0=push-pull 1=open-drain
	 * pupd: 0=none 1=pull-up 2=pull-down | odr = what we wrote | idr = what the pad reads */
	dbg("module %d RESET pin (%s): bit=%lu mode=%lu otype=%lu pupd=%lu odr=%lu idr=%lu\n",
		 module + 1, when, (unsigned long)bit,
		 (unsigned long)((port->MODER  >> (bit * 2u)) & 3u),
		 (unsigned long)((port->OTYPER >>  bit)       & 1u),
		 (unsigned long)((port->PUPDR  >> (bit * 2u)) & 3u),
		 (unsigned long)((port->ODR    >>  bit)       & 1u),
		 (unsigned long)((port->IDR    >>  bit)       & 1u));
}
#endif

/**************************************************************************************
** \brief     Assert the module's reset line for assert_ms, then release it.
**
**            Single implementation of the hardware reset, shared by the bootloader
**            detection path and by any application-level recovery/adopt path, so the
**            two can never drift apart in pulse width or ordering.
**
**            On IOT the pad level is sampled back while reset is asserted and again
**            after release. The reset pin is open-drain with a pull-up, so a failure to
**            read LOW while asserted means the pad is not being driven at all — on the
**            STM32H5 MOD1_RESET is PB4, which is also NJTRST, so an attached debugger
**            holding the JTAG pins will produce exactly that.
**
** \param     moduleslot  Slot index (0-based).
** \param     assert_ms   Reset-assert pulse width in milliseconds.
** \return    0 when the pad was observed low while asserted and high after release,
**            -1 otherwise (the reset line is not doing what was asked of it).
***************************************************************************************/
int8_t GO_communication_modules_reset_module(uint8_t moduleslot, uint32_t assert_ms) {
	int8_t rc = 0;

	GO_communication_modules_reset_state_module(moduleslot, 1);   /* assert = drive low */
	GO_communication_modules_delay_1ms(1);

#ifdef GOCONTROLL_IOT
	/* Eenmalig per slot de rauwe registerstand, zodat "de pin gaat niet laag" op de
	 * analyzer eenduidig te scheiden is van een MCU die hem wel degelijk laag trekt. */
	static uint8_t s_dumped[8] = {0};
	if (moduleslot < 8u && s_dumped[moduleslot] == 0u) {
		s_dumped[moduleslot] = 1u;
		GO_communication_modules_reset_pin_dump(moduleslot, "asserted");
	}

	if (GO_communication_modules_reset_pin_level(moduleslot) != GPIO_PIN_RESET) {
		err("module %d: RESET line did not go LOW while asserted — pad is not driven "
			"(MOD1_RESET is PB4 = NJTRST; check for an attached debugger)\n",
			moduleslot + 1);
		rc = -1;
	}
#endif

	if (assert_ms > 1u) {
		GO_communication_modules_delay_1ms(assert_ms - 1u);
	}

	GO_communication_modules_reset_state_module(moduleslot, 0);   /* release = pulled up */
	GO_communication_modules_delay_1ms(MODULE_RESET_SETTLE_MS);

#ifdef GOCONTROLL_IOT
	if (GO_communication_modules_reset_pin_level(moduleslot) != GPIO_PIN_SET) {
		err("module %d: RESET line stuck LOW after release — module is held in reset\n",
			moduleslot + 1);
		rc = -1;
	}
#endif

	return rc;
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

	/* Invalidate the length/checksum bytes so a stale response from a previous
	 * transfer in this static buffer cannot masquerade as a valid reply if this
	 * transfer fails. */
	dataRx[1] = 0;
	if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
		GO_communication_modules_spi_drain();
		if (HAL_SPI_TransmitReceive_DMA(&hspi1, &dataTx[0], &dataRx[0],
										BOOTMESSAGELENGTHCHECK) == HAL_OK) {
			GO_communication_modules_spi_wait();
		} else {
			HAL_SPI_Abort(&hspi1);
		}
	} else {
		HAL_SPI_TransmitReceive(&hspi1, &dataTx[0], &dataRx[0], BOOTMESSAGELENGTHCHECK, 100);
	}

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
	int rc = 0;
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
	/* Pre-transmission delay — BEFORE asserting CS, not after.
	 *
	 * Callers use this as an INTER-MESSAGE settle: GO_module_output_configuration() passes
	 * 500 us for the second initialization message "because the module needs to handle the
	 * first message". Previously the delay sat between the CS assert and the transmit, so
	 * CS was held low for the whole wait — and since 500 us rounds up to delay_1ms(1),
	 * which is a full osDelay(1) under the scheduler, the slave saw CS go active and then
	 * no clock for at least an entire RTOS tick. A slave that arms its SPI reception on the
	 * CS falling edge cannot be expected to survive that, and the message is lost silently
	 * because send_spi() never reads a response.
	 *
	 * Waiting first and only then framing the transfer gives the module the intended gap
	 * while keeping the CS window tight around the actual clocking. */
	GO_communication_modules_delay_1ms(delay / 1000 + (delay % 1000 != 0));

	if (module == 0) {
		HAL_GPIO_WritePin(SPI_MOD1_CS_GPIO_Port, SPI_MOD1_CS_Pin,
						  GPIO_PIN_RESET);
	} else if (module == 1) {
		HAL_GPIO_WritePin(SPI_MOD2_CS_GPIO_Port, SPI_MOD2_CS_Pin,
						  GPIO_PIN_RESET);
	}

	if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
		GO_communication_modules_spi_drain();
		if (HAL_SPI_Transmit_DMA(&hspi1, &dataTx[0],
								 dataLength + MESSAGEOVERLENGTH) == HAL_OK) {
			rc = GO_communication_modules_spi_wait();
		} else {
			HAL_SPI_Abort(&hspi1);
			rc = -1;
		}
	} else {
		if (HAL_SPI_Transmit(&hspi1, &dataTx[0], dataLength + MESSAGEOVERLENGTH,
							 100) != HAL_OK) {
			rc = -1;
		}
	}

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

	return rc;
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

	/* Invalidate the length byte so a stale frame left in this static RX buffer
	 * by a previous cycle cannot pass the checksum check below if this transfer
	 * fails (e.g. after a timeout/abort). */
	dataRx[1] = 0;
	if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
		GO_communication_modules_spi_drain();
		if (HAL_SPI_TransmitReceive_DMA(&hspi1, &dataTx[0], &dataRx[0],
										dataLength + MESSAGEOVERLENGTH) == HAL_OK) {
			GO_communication_modules_spi_wait();
		} else {
			HAL_SPI_Abort(&hspi1);
		}
	} else {
		HAL_SPI_TransmitReceive(&hspi1, &dataTx[0], &dataRx[0], dataLength + MESSAGEOVERLENGTH, 100);
	}

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
