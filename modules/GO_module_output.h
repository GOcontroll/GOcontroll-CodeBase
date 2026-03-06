/**************************************************************************************
 * \file         GO_module_output.h
 * \brief        Utility functions to interface the GOcontroll output module.
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
#ifndef GO_MODULE_OUTPUT_H
#define GO_MODULE_OUTPUT_H

/****************************************************************************************
 * Include files
 ****************************************************************************************/
#include "GO_communication_modules.h"

/****************************************************************************************
 * Module specific defines
 ****************************************************************************************/
#define OUTPUTMODULE6CHANNEL (1)
#define OUTPUTMODULE10CHANNEL (2)

#define OUTPUTFUNC_DISABLED 1  // channel is unused

#define OUTPUTFUNC_6CH_HALFBRIDGE 2	 // half bridge duty cycle controlled
#define OUTPUTFUNC_6CH_LOWSIDEDUTY \
	3  // low side switching duty cycle
	   // controlled
#define OUTPUTFUNC_6CH_HIGHSIDEDUTY \
	4  // high side switching duty cycle controlled
#define OUTPUTFUNC_6CH_LOWSIDEBOOL 5   // low side switching on or off (0-1)
#define OUTPUTFUNC_6CH_HIGHSIDEBOOL 6  // high side switching on or off (0-1)
#define OUTPUTFUNC_6CH_PEAKANDHOLD \
	7  // peak and hold function (uses half bridge switching)
#define OUTPUTFUNC_6CH_FREQUENCYOUT 8  // frequency output (0-500Hz)

#define OUTPUTFUNC_10CH_HIGHSIDEDUTY \
	2  // high side switching duty cycle controlled
#define OUTPUTFUNC_10CH_HIGHSIDEBOOL 3	// high side switching on or off (0-1)

#define OUTPUTFREQ_100HZ 1		// duty cycle frequency
#define OUTPUTFREQ_200HZ 2		// duty cycle frequency
#define OUTPUTFREQ_6CH_500HZ 3	// duty cycle frequency (6 channel module only)
#define OUTPUTFREQ_6CH_1KHZ 4	// duty cycle frequency (6 channel module only)
#define OUTPUTFREQ_6CH_2KHZ 5	// duty cycle frequency (6 channel module only)
#define OUTPUTFREQ_6CH_5KHZ 6	// duty cycle frequency (6 channel module only)
#define OUTPUTFREQ_6CH_10KHZ 7	// duty cycle frequency (6 channel module only)

#define PEAKCURRENTMAX 3500	 // max allowed duty cycle
#define CURRENTMAXMAX 4000	 // max allowed current through a channel

#define OUTPUTFREQCHANNEL1AND2 \
	0  // macro for selecting the frequency channels on the output modules
#define OUTPUTFREQCHANNEL3AND4 \
	1  // macro for selecting the frequency channels on the output modules
#define OUTPUTFREQCHANNEL5AND6 \
	2  // macro for selecting the frequency channels on the output modules
#define OUTPUTFREQCHANNEL7AND8_10CH \
	3  // macro for selecting the frequency channels on the output modules (10
	   // channel module only)
#define OUTPUTFREQCHANNEL9AND10_10CH \
	4  // macro for selecting the frequency channels on the output modules (10
	   // channel module only)

#define OUTPUTCHANNEL1 0
#define OUTPUTCHANNEL2 1
#define OUTPUTCHANNEL3 2
#define OUTPUTCHANNEL4 3
#define OUTPUTCHANNEL5 4
#define OUTPUTCHANNEL6 5
#define OUTPUTCHANNEL7 6
#define OUTPUTCHANNEL8 7
#define OUTPUTCHANNEL9 8
#define OUTPUTCHANNEL10 9

#define OUTPUTFUNCMASK 0b11110000
#define OUTPUTFREQMASK 0b00001111

/****************************************************************************************
 * Data declarations
 ****************************************************************************************/

union parameter1 {
	uint16_t raw;
	uint16_t peakCurrent;
	uint16_t fastLoopGain;
};

union parameter2 {
	uint16_t raw;
	uint16_t peakTime;
	uint16_t fastLoopBasicDuty;
};

typedef struct {
	uint8_t configuration[10];
	uint16_t value[10];
	uint16_t dutyCycle[6];
	uint32_t syncCounter[6];
	int16_t current[6];
	uint16_t currentMax[6];
	union parameter1 channelParameter1[6];
	union parameter2 channelParameter2[6];
	uint8_t fastLoopModule[6];
	uint8_t fastLoopChannel[6];
	int16_t temperature;
	int16_t ground;
	uint16_t supply;
	int16_t totalCurrent;
	uint32_t errorCode;
	uint8_t communicationCheck;
	uint32_t moduleIdentifier;
	uint8_t moduleType;
	uint8_t moduleSlot;
	uint32_t sw_version;
} _outputModule;

/****************************************************************************************
 * Function prototypes
 ****************************************************************************************/

/**************************************************************************************
** \brief     Sends the configuration data to the output module.
** \param     outputModule  Pointer to a _outputModule struct that holds the
**                          data for the module configuration.
** \return    0 if successful, negative errno value if failed.
***************************************************************************************/
int GOmoduleoutput_configuration(_outputModule* outputModule);

/**************************************************************************************
** \brief     Sends the output values to the output module and reads back feedback.
** \param     outputModule  Pointer to a _outputModule struct that holds the
**                          data for the module configuration.
** \return    0 if successful, negative errno value if failed.
***************************************************************************************/
int GOmoduleoutput_sendValues(_outputModule* outputModule);

/**************************************************************************************
** \brief     Sets the module slot for an output module and validates the slot assignment.
** \param     outputModule  Pointer to a _outputModule struct that holds the
**                          data for the module configuration.
** \param     moduleSlot    The slot index (0-based) that the module is inserted in.
** \return    0 if successful, negative errno value if failed.
***************************************************************************************/
int GOmoduleoutput_setModuleSlot(_outputModule* outputModule, uint8_t moduleSlot);

/**************************************************************************************
** \brief     Sets the module type (6 or 10 channel) for an output module.
** \param     outputModule  Pointer to a _outputModule struct that holds the
**                          data for the module configuration.
** \param     moduleType    The type of module, either OUTPUTMODULE6CHANNEL or
**                          OUTPUTMODULE10CHANNEL.
** \return    0 if successful, negative errno value if failed.
***************************************************************************************/
int GOmoduleoutput_setModuleType(_outputModule* outputModule, uint8_t moduleType);

/**************************************************************************************
** \brief     Configures an output channel on a 6-channel output module.
** \param     outputModule  Pointer to a _outputModule struct that holds the
**                          data for the module configuration.
** \param     channel       The channel index to be configured (use OUTPUTCHANNEL* macros).
** \param     func          The function of the channel, range 1-8 (use OUTPUTFUNC_6CH_* macros).
** \param     currentMax    Maximum continuous current limit for the channel, range 0-4000.
** \param     peak_current  Peak current duty cycle for peak-and-hold mode, range 0-3500.
** \param     peak_time     Duration of the peak current phase in peak-and-hold mode.
** \return    0 if successful, negative errno value if failed.
***************************************************************************************/
int GOmoduleoutput_6chConfigureChannel(_outputModule* outputModule,
									 uint8_t channel, uint8_t func,
									 uint16_t currentMax, uint16_t peak_current,
									 uint16_t peak_time);

/**************************************************************************************
** \brief     Configures an output channel on a 10-channel output module.
** \param     outputModule  Pointer to a _outputModule struct that holds the
**                          data for the module configuration.
** \param     channel       The channel index to be configured (use OUTPUTCHANNEL* macros).
** \param     func          The function of the channel, range 1-3 (use OUTPUTFUNC_10CH_* macros).
** \return    0 if successful, negative errno value if failed.
***************************************************************************************/
int GOmoduleoutput_10chConfigureChannel(_outputModule* outputModule,
									  uint8_t channel, uint8_t func);

/**************************************************************************************
** \brief     Configures the PWM frequency for a frequency channel pair on an output module.
** \param     outputModule  Pointer to a _outputModule struct that holds the
**                          data for the module configuration.
** \param     channel       The frequency channel index to configure (use OUTPUTFREQCHANNEL* macros).
** \param     frequency     The PWM frequency to set for this channel (use OUTPUTFREQ* macros).
** \return    0 if successful, negative errno value if failed.
***************************************************************************************/
int GOmoduleoutput_configureFrequency(_outputModule* outputModule,
									uint8_t channel, uint8_t frequency);

#endif /* GO_MODULE_OUTPUT_H */
