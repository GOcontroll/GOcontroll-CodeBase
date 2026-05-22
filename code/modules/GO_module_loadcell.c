/**************************************************************************************
 * \file   GO_module_loadcell.c
 * \brief  Utility functions to interface the GOcontroll loadcell module.
 * \internal
 *----------------------------------------------------------------------------------------
 *                          C O P Y R I G H T
 *----------------------------------------------------------------------------------------
 * Copyright 2024 (c) by GOcontroll http://www.gocontroll.com All rights
 * reserved
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
 * Include files
 ****************************************************************************************/
#include "GO_module_loadcell.h"

#include <errno.h>
#include <string.h>

#include "GO_communication_modules.h"
#include "print.h"

/****************************************************************************************
 * Macro definitions
 ****************************************************************************************/

/****************************************************************************************
 * Local data declarations
 ****************************************************************************************/
static uint8_t loadcellDataTx[LOADCELLMODULEMESSAGELENGTH + MESSAGEOVERLENGTH];
static uint8_t loadcellDataRx[LOADCELLMODULEMESSAGELENGTH + MESSAGEOVERLENGTH];
const uint8_t LOADCELLMODULECHANNELID[] = {20, 10, 4};
const uint32_t VERSIONSPIPROTOCOLV2_LOADCELL = 2 << 16;

extern _hardwareConfig hardwareConfig;

/****************************************************************************************
** \brief  Packs four channels of gain/datarate/sensitivity/FullScale into the SPI
**         TX frame starting at byte 6 (5 bytes per channel, one reserved byte per
**         channel for future use) and sends it as message type 1 to the module.
**         The firmware version is read from hardwareConfig before the frame is built.
****************************************************************************************/
int GO_module_loadcell_configuration(_loadcellModule* loadcellModule) {
	if (hardwareConfig.moduleOccupancy[loadcellModule->moduleSlot][0] == 0) {
		return -ENODEV;
	}
	uint8_t dataPointer = 6;
	loadcellModule->sw_version =
		hardwareConfig.moduleOccupancy[loadcellModule->moduleSlot][4] << 16 |
		hardwareConfig.moduleOccupancy[loadcellModule->moduleSlot][5] << 8 |
		hardwareConfig.moduleOccupancy[loadcellModule->moduleSlot][6];

	for (uint8_t pointer = 0; pointer < 4; pointer++) {
		loadcellDataTx[dataPointer++] = loadcellModule->configuration[pointer];
		loadcellDataTx[dataPointer++] = loadcellModule->sensitivity[pointer];
		dataPointer++;  // reserved for future use
		loadcellDataTx[dataPointer++] =
			(loadcellModule->FullScale[pointer] >> 8) & 0xFF;
		loadcellDataTx[dataPointer++] =
			loadcellModule->FullScale[pointer] & 0xFF;
	}

	return GO_communication_modules_send_spi(
		loadcellModule->moduleSlot + 1, LOADCELLMODULEMESSAGELENGTH, 1, 15, 2,
		1, loadcellModule->moduleSlot, &loadcellDataTx[0], 0);
}

/****************************************************************************************
** \brief  Edge-triggered tare command. Writes the channel index to TX byte 6 and
**         the signed 32-bit offset to bytes 7-10, then sends message type 2 to the
**         module. The frame is only transmitted when trigger differs from the stored
**         TareTrigger value, preventing repeated sends on every cycle.
****************************************************************************************/
int GO_module_loadcell_tare(_loadcellModule* loadcellModule, uint8_t channel,
							int32_t value, uint8_t trigger) {
	if (hardwareConfig.moduleOccupancy[loadcellModule->moduleSlot][0] == 0) {
		return -ENODEV;
	}
	if (channel > 3) {
		return -EINVAL;
	}

	if (loadcellModule->TareTrigger[channel] != trigger) {
		loadcellDataTx[6] = channel;
		*(int32_t*)&loadcellDataTx[7] = value;

		int res = GO_communication_modules_send_spi(
			loadcellModule->moduleSlot + 1, LOADCELLMODULEMESSAGELENGTH, 1, 15,
			2, 2, loadcellModule->moduleSlot, &loadcellDataTx[0], 0);

		loadcellModule->TareTrigger[channel] = trigger;
		return res;
	}
	return 0;
}

/****************************************************************************************
** \brief  Sends a type-3 SPI request and reads back 4 signed 32-bit values from the
**         response frame. Each value occupies 4 bytes starting at byte 6 of the RX
**         buffer (offsets 6, 10, 14, 18). Values are written into loadcellModule->value[]
**         only when the SPI transaction succeeds (checksum valid).
****************************************************************************************/
int GO_module_loadcell_receive_values(_loadcellModule* loadcellModule) {
	if (hardwareConfig.moduleOccupancy[loadcellModule->moduleSlot][0] == 0) {
		return -ENODEV;
	}
	int res = GO_communication_modules_send_receive_spi(
		loadcellModule->moduleSlot + 1, LOADCELLMODULEMESSAGELENGTH, 2, 15, 3,
		1, loadcellModule->moduleSlot, &loadcellDataTx[0], &loadcellDataRx[0]);
	if (res == 0) {
		for (uint8_t pointer = 0; pointer < 4; pointer++) {
			loadcellModule->value[pointer] =
				*(int32_t*)&loadcellDataRx[(pointer * 4) + 6];
		}
	}
	return res;
}

/****************************************************************************************
** \brief  Validates the requested slot against hardwareConfig.moduleOccupancy using
**         a 3-byte memcmp against LOADCELLMODULECHANNELID {20, 10, 4}. Stores the
**         slot index on match; logs an error and returns -EINVAL on mismatch or
**         out-of-range slot.
****************************************************************************************/
int GO_module_loadcell_set_module_slot(_loadcellModule* loadcellModule,
									   uint8_t moduleSlot) {
	if (moduleSlot < hardwareConfig.moduleNumber) {
		if (!memcmp(hardwareConfig.moduleOccupancy[moduleSlot],
					LOADCELLMODULECHANNELID, 3)) {
			loadcellModule->moduleSlot = moduleSlot;
			return 0;
		}
		err("module slot %d is contested by multiple module claims, check "
			"*set_module_slot init functions for double slot claims.\n",
			moduleSlot + 1);
		return -EINVAL;
	}
	err("Invalid module slot selected for a loadcell module, selected %d, but "
		"the range is 1-%d.\n",
		moduleSlot + 1, hardwareConfig.moduleNumber);
	return -EINVAL;
}

/****************************************************************************************
** \brief  Validates channel (0-3), gain (0-3), datarate (0-3) and sensitivity (0-3),
**         then stores sensitivity and FullScale directly and encodes gain and datarate
**         into a single configuration byte as (datarate << 2) | gain. Settings are
**         buffered in the struct and committed to the module by a subsequent call to
**         GO_module_loadcell_configuration().
****************************************************************************************/
int GO_module_loadcell_configure_channel(_loadcellModule* loadcellModule,
										 uint8_t channel, uint8_t gain,
										 uint8_t datarate, uint8_t sensitivity,
										 uint16_t FullScale) {
	if (loadcellModule->moduleType != LOADCELLMODULE) {
		err("Incorrect module type selected for channel %d, loadcell module in "
			"slot %d, this function is only meant for loadcell modules.\n",
			channel + 1, loadcellModule->moduleSlot + 1);
		return -EINVAL;
	}
	if (channel > 3) {
		err("Configured channel is out of range for loadcell module in slot %d, "
			"range is 1-4, entered is %d, please use the macros to configure "
			"channels.\n",
			loadcellModule->moduleSlot + 1, channel + 1);
		return -EINVAL;
	}
	if (gain > 3) {
		err("Invalid gain configuration for channel %d, loadcell module in slot "
			"%d, range is 0-3, entered is %d, please use the macros to configure "
			"channels.\n",
			channel + 1, loadcellModule->moduleSlot + 1, gain);
		return -EINVAL;
	}
	if (datarate > 3) {
		err("Invalid datarate configuration for channel %d, loadcell module in "
			"slot %d, range is 0-3, entered is %d, please use the macros to "
			"configure channels.\n",
			channel + 1, loadcellModule->moduleSlot + 1, datarate);
		return -EINVAL;
	}
	if (sensitivity > 3) {
		err("Invalid sensitivity configuration for channel %d, loadcell module "
			"in slot %d, range is 0-3, entered is %d, please use the macros to "
			"configure channels.\n",
			channel + 1, loadcellModule->moduleSlot + 1, sensitivity);
		return -EINVAL;
	}
	loadcellModule->sensitivity[channel] = sensitivity;
	loadcellModule->FullScale[channel] = FullScale;
	loadcellModule->configuration[channel] = (datarate << 2) | gain;
	return 0;
}

/****************************************************************************************
** \brief  Validates that moduleType equals LOADCELLMODULE and stores it in the struct.
**         Must be called before GO_module_loadcell_configure_channel() so that the
**         channel configuration function can verify the module type is correct.
****************************************************************************************/
int GO_module_loadcell_set_module_type(_loadcellModule* loadcellModule,
									   uint8_t moduleType) {
	if (moduleType == LOADCELLMODULE) {
		loadcellModule->moduleType = LOADCELLMODULE;
		return 0;
	}
	err("Invalid module type selected for loadcell module in slot %d, please "
		"use the LOADCELLMODULE macro.\n",
		loadcellModule->moduleSlot + 1);
	return -EINVAL;
}
