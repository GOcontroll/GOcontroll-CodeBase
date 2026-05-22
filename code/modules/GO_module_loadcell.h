/**************************************************************************************
 * \file         GO_module_loadcell.h
 * \brief        Utility functions to interface the GOcontroll loadcell module.
 * \internal
 *----------------------------------------------------------------------------------------
 *                          C O P Y R I G H T
 *----------------------------------------------------------------------------------------
 * Copyright 2024 (c) by GOcontroll http://www.gocontroll.com
 * All rights reserved
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
#ifndef GO_MODULE_LOADCELL_H
#define GO_MODULE_LOADCELL_H

/****************************************************************************************
 * Include files
 ****************************************************************************************/
#include "GO_communication_modules.h"

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************************
 * Module specific defines
 ****************************************************************************************/
#define LOADCELLMODULE (1)

#define LOADCELLGAIN_64  1
#define LOADCELLGAIN_128 2
#define LOADCELLGAIN_32  3

#define LOADCELLDATARATE_10 1
#define LOADCELLDATARATE_80 2

#define LOADCELLSENSITIVITY_RAW  0
#define LOADCELLSENSITIVITY_1MVV 1
#define LOADCELLSENSITIVITY_2MVV 2
#define LOADCELLSENSITIVITY_3MVV 3

#define LOADCELLCHANNEL1 0
#define LOADCELLCHANNEL2 1
#define LOADCELLCHANNEL3 2
#define LOADCELLCHANNEL4 3

/****************************************************************************************
 * Data declarations
 ****************************************************************************************/

typedef struct {
	uint8_t configuration[4];
	uint8_t sensitivity[4];
	uint16_t FullScale[4];
	int32_t value[4];
	uint32_t moduleIdentifier;
	uint8_t moduleType;
	uint8_t moduleSlot;
	uint8_t TareTrigger[4];
	uint32_t sw_version;
} _loadcellModule;

/****************************************************************************************
 * Function prototypes
 ****************************************************************************************/

/**************************************************************************************
** \brief     Sends the configuration data to the loadcell module.
**            Packs the per-channel gain, data rate, sensitivity and full-scale
**            settings into an SPI frame and transmits it to the module. Also reads
**            the firmware version from hardwareConfig into loadcellModule->sw_version.
**            Must be called after GO_module_loadcell_configure_channel() for all
**            channels that need to be active.
** \param     loadcellModule  Pointer to a _loadcellModule struct that holds the
**                            relevant module configuration.
** \return    0 if successful, negative errno value if failed.
***************************************************************************************/
int GO_module_loadcell_configuration(_loadcellModule* loadcellModule);

/**************************************************************************************
** \brief     Retrieves measurement values from the loadcell module via SPI.
**            Sends a request frame and reads back the four channel values into
**            loadcellModule->value[]. Each value is a signed 32-bit integer whose
**            unit depends on the sensitivity mode configured for that channel:
**            raw ADC counts (LOADCELLSENSITIVITY_RAW) or scaled millinewtons /
**            engineering units for the mV/V sensitivity modes.
** \param     loadcellModule  Pointer to a _loadcellModule struct that holds the
**                            relevant module configuration.
** \return    0 if successful, negative errno value if failed.
***************************************************************************************/
int GO_module_loadcell_receive_values(_loadcellModule* loadcellModule);

/**************************************************************************************
** \brief     Sets the module slot for a loadcell module and validates the slot
**            assignment. Verifies that the module physically present in the given
**            slot matches the loadcell module ID {20, 10, 4}.
**            GO_communication_modules_initialize() must be called for this slot
**            before this function, otherwise the occupancy table is empty and
**            validation will fail.
** \param     loadcellModule  Pointer to a _loadcellModule struct that holds the
**                            relevant module configuration.
** \param     moduleSlot      The slot index (0-based) that the module is inserted in.
** \return    0 if successful, negative errno value if failed.
***************************************************************************************/
int GO_module_loadcell_set_module_slot(_loadcellModule* loadcellModule,
									   uint8_t moduleSlot);

/**************************************************************************************
** \brief     Configures a single channel on a loadcell module.
**            Stores gain, data rate, sensitivity and full-scale in the struct;
**            the settings are not sent to the module until
**            GO_module_loadcell_configuration() is called.
** \param     loadcellModule  Pointer to a _loadcellModule struct that holds the
**                            relevant module configuration.
** \param     channel         Channel index to configure (use LOADCELLCHANNEL* macros).
** \param     gain            ADC programmable gain amplifier setting.
**                            LOADCELLGAIN_64 = 64x, LOADCELLGAIN_128 = 128x,
**                            LOADCELLGAIN_32 = 32x.
** \param     datarate        ADC conversion rate.
**                            LOADCELLDATARATE_10 = 10 SPS, LOADCELLDATARATE_80 = 80 SPS.
** \param     sensitivity     Output scaling mode.
**                            LOADCELLSENSITIVITY_RAW  = raw ADC counts,
**                            LOADCELLSENSITIVITY_1MVV = 1 mV/V rated output,
**                            LOADCELLSENSITIVITY_2MVV = 2 mV/V rated output,
**                            LOADCELLSENSITIVITY_3MVV = 3 mV/V rated output.
** \param     FullScale       Full-scale capacity of the load cell in the application
**                            engineering unit (e.g. kg or N). Used by the module
**                            firmware to scale the output for mV/V sensitivity modes.
** \return    0 if successful, negative errno value if failed.
***************************************************************************************/
int GO_module_loadcell_configure_channel(_loadcellModule* loadcellModule,
										 uint8_t channel, uint8_t gain,
										 uint8_t datarate, uint8_t sensitivity,
										 uint16_t FullScale);

/**************************************************************************************
** \brief     Sets the module type for a loadcell module.
**            Must be called before GO_module_loadcell_configure_channel() so that
**            the channel configuration function can verify the correct module type.
** \param     loadcellModule  Pointer to a _loadcellModule struct that holds the
**                            relevant module configuration.
** \param     moduleType      Module type identifier. Use the LOADCELLMODULE macro.
** \return    0 if successful, negative errno value if failed.
***************************************************************************************/
int GO_module_loadcell_set_module_type(_loadcellModule* loadcellModule,
									   uint8_t moduleType);

/**************************************************************************************
** \brief     Sends a tare (zero offset) command to a channel on the loadcell module.
**            The command is edge-triggered: a frame is only sent to the module when
**            the trigger value differs from the last acknowledged trigger. Toggle the
**            trigger between 0 and 1 on each desired tare event to control when the
**            module re-applies the offset.
** \param     loadcellModule  Pointer to a _loadcellModule struct that holds the
**                            relevant module configuration.
** \param     channel         Channel index to tare (use LOADCELLCHANNEL* macros).
** \param     value           Tare offset to apply, in the same unit as value[channel].
** \param     trigger         Edge signal: send occurs only when this differs from the
**                            stored TareTrigger[channel] value.
** \return    0 if successful, negative errno value if failed.
***************************************************************************************/
int GO_module_loadcell_tare(_loadcellModule* loadcellModule, uint8_t channel,
							int32_t value, uint8_t trigger);

#ifdef __cplusplus
}
#endif

#endif /* GO_MODULE_LOADCELL_H */
