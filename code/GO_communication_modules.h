/**************************************************************************************
 * \file   GO_communication_modules.h
 * \brief  Module communication interface for GOcontroll hardware.
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

#ifndef GO_COMMUNICATION_MODULES_H
#define GO_COMMUNICATION_MODULES_H

/****************************************************************************************
 * Include files
 ****************************************************************************************/
#include <stdint.h>
#include "GO_board.h"

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************************
 * Macro definitions (Message lengths)
 ****************************************************************************************/
#define BOOTMESSAGELENGTH				46
#define BOOTMESSAGELENGTHCHECK			61

#define INPUTMODULE6CHMESSAGELENGTH		55
#define INPUTMODULE10CHMESSAGELENGTH	50

#define INPUTMODULE420MAMESSAGELENGTH	33

#define OUTPUTMODULE6CHMESSAGELENGTH	44
#define OUTPUTMODULE10CHMESSAGELENGTH	49

#define BRIDGEMODULEMESSAGELENGTH		44

#define IRMODULEMESSAGELENGTH			31
#define RTCMODULEMESSAGELENGTH			44

#define LOADCELLMODULEMESSAGELENGTH		50

#define MESSAGEOVERLENGTH				1

/****************************************************************************************
 * Macro definitions (Module reset)
 ****************************************************************************************/
/* Reset-assert pulse width used by GO_communication_modules_reset_module().
 *
 * A 2 ms pulse resets the input module fine, but is too short to force an already-RUNNING
 * output module back into its bootloader: the module keeps running its application and
 * detection then reads live app frames ("message incorrect") instead of 9,45,9.
 *
 * History worth keeping: this was briefly raised to 3000 ms (the ~2.79 s a cold boot
 * happens to hold reset, measured on the logic analyzer) while the reset line itself was
 * under suspicion. That turned out to be a WIRING fault on slot 1 — the pulse reached the
 * wrong module — not a pulse-width problem. With the wiring fixed, a much shorter pulse
 * suffices; the bring-up is serialised, so this delay is paid once per module slot.
 *
 * Tune against the module firmware's real bootloader window. Override at build time with
 * -DMODULE_RESET_ASSERT_MS=<ms> — no need to edit this shared file. */
#ifndef MODULE_RESET_ASSERT_MS
#define MODULE_RESET_ASSERT_MS			250u
#endif

/****************************************************************************************
 * Macro definitions (Hardware configuration)
 ****************************************************************************************/
#define NOT_INSTALLED	0
#define ADC_MCP3004		1
#define ADC_ADS1015		2
#define ADC_INTEGRATED	3

#define LED_GPIO		1
#define LED_RUKR		2

#define MODULESLOT1		0
#define MODULESLOT2		1
#define MODULESLOT3		2
#define MODULESLOT4		3
#define MODULESLOT5		4
#define MODULESLOT6		5
#define MODULESLOT7		6
#define MODULESLOT8		7

#define SLOTOCCUPIED	1
#define SLOTFREE		0

/****************************************************************************************
 * Function prototypes — platform-independent
 ****************************************************************************************/

/**************************************************************************************
** \brief     Calculate the checksum of an SPI message.
** \param     array   Buffer containing the SPI message bytes.
** \param     length  Number of bytes to sum.
** \return    The checksum byte.
***************************************************************************************/
uint8_t GO_communication_modules_checksum_calculator(uint8_t *array, uint8_t length);

/**************************************************************************************
** \brief     Initialize a module in a specific slot.
** \param     moduleslot  Slot index (0-based).
** \return    0 on success, -ENODEV on failure.
***************************************************************************************/
int GO_communication_modules_initialize(uint8_t moduleslot);

/**************************************************************************************
** \brief     One-time SPI priming (throwaway transfer) so the first real module transfer
**            is not the one that initializes the peripheral. Ports the Linux DummySpiSend();
**            prevents the output module's slave from staying a byte out of phase. Call once
**            before the first module transaction. (STM32/IOT only.)
***************************************************************************************/
void GO_communication_modules_dummy_spi(uint8_t module);

/**************************************************************************************
** \brief     Register a detected module in the hardware config table.
** \param     slot  Module slot index (0-7).
** \param     rx    Bootloader RX buffer containing firmware info.
** \return    none
***************************************************************************************/
void GO_communication_modules_register_module(uint8_t slot, uint8_t *rx);

/**************************************************************************************
** \brief     Delay execution for a number of milliseconds.
** \param     times  Number of milliseconds to sleep.
** \return    none
***************************************************************************************/
void GO_communication_modules_delay_1ms(uint32_t times);

/**************************************************************************************
** \brief     Set the state of the reset pin of a module.
** \param     module  Slot index (0-7).
** \param     state   Desired pin state: 1 = assert reset, 0 = release.
** \return    0 on success, -1 on failure.
***************************************************************************************/
int8_t GO_communication_modules_reset_state_module(uint8_t module, uint8_t state);

/**************************************************************************************
** \brief     Assert a module's reset line for assert_ms, then release it.
**
**            The single shared hardware-reset implementation. Use this instead of
**            hand-rolling reset_state_module(1) / delay / reset_state_module(0), so the
**            bootloader-detection path and any application-level recovery path cannot
**            drift apart in pulse width or ordering.
**
**            On IOT the actual pad level is sampled while asserted and after release, so
**            a reset line that never moves is reported instead of silently producing a
**            module that stays in its application.
**
** \param     moduleslot  Slot index (0-based).
** \param     assert_ms   Reset-assert pulse width in ms (see MODULE_RESET_ASSERT_MS).
** \return    0 on success, -1 when the pad did not follow the requested level.
***************************************************************************************/
int8_t GO_communication_modules_reset_module(uint8_t moduleslot, uint32_t assert_ms);

/**************************************************************************************
** \brief     Get a module out of its bootloader state.
** \param     module  Slot index (0-7).
** \param     dataTx  Transmit buffer.
** \param     dataRx  Receive buffer.
** \return    0 on success, -1 on failure.
***************************************************************************************/
int GO_communication_modules_escape_from_bootloader(uint8_t module,
												  uint8_t *dataTx,
												  uint8_t *dataRx);

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
									 uint8_t *dataTx, uint32_t delay);

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
											uint8_t *dataRx);

/****************************************************************************************
 * Function prototypes — STM32H5 (GOCONTROLL_IOT) specific
 ****************************************************************************************/
#ifdef GOCONTROLL_IOT

/**************************************************************************************
** \brief     Delay execution using the RTOS scheduler (non-blocking for other tasks).
** \param     times  Number of milliseconds to delay.
** \return    none
***************************************************************************************/
void GO_communication_modules_delay_1ms_os(uint32_t times);

#endif /* GOCONTROLL_IOT */

#ifdef __cplusplus
}
#endif

#endif /* GO_COMMUNICATION_MODULES_H */

/* end of GO_communication_modules.h */
