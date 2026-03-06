/**************************************************************************************
 * \file         GO_module_input.h
 * \brief        Utility functions to interface the GOcontroll input module.
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
#ifndef GO_MODULE_INPUT_H
#define GO_MODULE_INPUT_H

/****************************************************************************************
 * Include files
 ****************************************************************************************/
#include "GO_communication_modules.h"

/****************************************************************************************
 * Module specific defines
 ****************************************************************************************/
#define INPUTMODULE6CHANNEL (1)
#define INPUTMODULE10CHANNEL (2)

#define INPUTFUNC_12BITADC 1
#define INPUTFUNC_MVANALOG 2
#define INPUTFUNC_DIGITAL_IN 3
#define INPUTFUNC_FREQUENCY 4
#define INPUTFUNC_DUTY_LOW 5
#define INPUTFUNC_DUTY_HIGH 6
#define INPUTFUNC_RPM 7
#define INPUTFUNC_PULSECOUNTER 8

#define INPUTVOLTAGERANGE_5V 0
#define INPUTVOLTAGERANGE_12V 1
#define INPUTVOLTAGERANGE_24V 2

#define INPUTPULLUP_NULL 0
#define INPUTPULLUP_3_3K 1
#define INPUTPULLUP6CH_4_7K 2
#define INPUTPULLUP6CH_10K 3

#define INPUTPULLDOWN_NULL 0
#define INPUTPULLDOWN_3_3K 1
#define INPUTPULLDOWN6CH_4_7K 2
#define INPUTPULLDOWN6CH_10K 3

#define INPUTSENSSUPPLYON 1
#define INPUTSENSSUPPLYOFF 2

#define PULSESPERROTATIONMAX 200
#define ANALOGSAMPLESMAX 1000

#define INPUTCHANNEL1 0
#define INPUTCHANNEL2 1
#define INPUTCHANNEL3 2
#define INPUTCHANNEL4 3
#define INPUTCHANNEL5 4
#define INPUTCHANNEL6 5
#define INPUTCHANNEL7 6
#define INPUTCHANNEL8 7
#define INPUTCHANNEL9 8
#define INPUTCHANNEL10 9

/****************************************************************************************
 * Data declarations
 ****************************************************************************************/
extern uint8_t resistorMatrix[4];

typedef struct {
	uint8_t configuration[10];
	uint8_t interface[10];
	uint8_t callibrationValue1[10];
	uint8_t callibrationValue2[10];
	uint8_t callibrationValue3[6];
	uint8_t callibrationValue4[6];
	int32_t value[10];
	uint32_t syncCounter[6];
	uint8_t pulscounterResetTrigger[10];
	uint32_t moduleIdentifier;
	uint8_t sensorSupply1;
	uint8_t sensorSupply2;
	uint8_t sensorSupply3;
	uint8_t moduleType;
	uint8_t moduleSlot;
	uint32_t sw_version;
} _inputModule;

/****************************************************************************************
 * Function prototypes
 ****************************************************************************************/

/**************************************************************************************
** \brief     Sends the configuration data to the input module.
** \param     inputModule  Pointer to a _inputModule struct that holds the
**                         relevant module configuration.
** \return    0 if successful, negative errno value if failed.
***************************************************************************************/
int GOmoduleinput_configuration(_inputModule* inputModule);

/**************************************************************************************
** \brief     Retrieves measurement values from the input module via SPI.
** \param     inputModule  Pointer to a _inputModule struct that holds the
**                         relevant module configuration.
** \return    0 if successful, negative errno value if failed.
***************************************************************************************/
int GOmoduleinput_receiveValues(_inputModule* inputModule);

/**************************************************************************************
** \brief     Sends a new pulse counter value to a specified channel of the input module.
** \param     inputModule  Pointer to a _inputModule struct that holds the
**                         relevant module configuration.
** \param     channel      The channel index on which the counter needs to be set.
** \param     value        The new value to write to the pulse counter.
** \param     trigger      Change on this input will trigger a send to the module.
** \return    0 if successful, negative errno value if failed.
***************************************************************************************/
int GOmoduleinput_resetPulsCounter(_inputModule* inputModule, uint8_t channel,
								 int32_t value, uint8_t trigger);

/**************************************************************************************
** \brief     Sets the module slot for an input module and validates the slot assignment.
** \param     inputModule  Pointer to a _inputModule struct that holds the
**                         relevant module configuration.
** \param     moduleSlot   The slot index (0-based) that the module is inserted in.
** \return    0 if successful, negative errno value if failed.
***************************************************************************************/
int GOmoduleinput_setModuleSlot(_inputModule* inputModule, uint8_t moduleSlot);

/**************************************************************************************
** \brief     Sets the module type (6 or 10 channel) for an input module.
** \param     inputModule  Pointer to a _inputModule struct that holds the
**                         relevant module configuration.
** \param     moduleType   The type of module, either INPUTMODULE6CHANNEL or
**                         INPUTMODULE10CHANNEL.
** \return    0 if successful, negative errno value if failed.
***************************************************************************************/
int GOmoduleinput_setModuleType(_inputModule* inputModule, uint8_t moduleType);

/**************************************************************************************
** \brief     Configures an input channel on a 6-channel input module.
** \param     inputModule            Pointer to a _inputModule struct that holds the
**                                   relevant module configuration.
** \param     channel                The channel index to be configured (use INPUTCHANNEL* macros).
** \param     func                   The function of the channel, range 1-8 (use INPUTFUNC* macros).
** \param     voltage_range          The voltage range for analog measurements (use INPUTVOLTAGERANGE* macros).
** \param     pull_up                Pull-up resistor selection (use INPUTPULLUP* macros).
** \param     pull_down              Pull-down resistor selection (use INPUTPULLDOWN* macros).
** \param     pulses_per_rotation    Pulses per rotation for rotary encoders, range 0-200.
** \param     analog_filter_samples  Number of analog filter samples per measurement, range 0-1000.
** \return    0 if successful, negative errno value if failed.
***************************************************************************************/
int GOmoduleinput_6chConfigureChannel(_inputModule* inputModule, uint8_t channel,
									uint8_t func, uint8_t voltage_range,
									uint8_t pull_up, uint8_t pull_down,
									uint8_t pulses_per_rotation,
									uint16_t analog_filter_samples);

/**************************************************************************************
** \brief     Configures an input channel on a 10-channel input module.
** \param     inputModule  Pointer to a _inputModule struct that holds the
**                         relevant module configuration.
** \param     channel      The channel index to be configured (use INPUTCHANNEL* macros).
** \param     func         The function of the channel, range 1-8 (use INPUTFUNC* macros).
** \param     pull_up      Pull-up resistor selection (use INPUTPULLUP* macros).
** \param     pull_down    Pull-down resistor selection (use INPUTPULLDOWN* macros).
** \return    0 if successful, negative errno value if failed.
***************************************************************************************/
int GOmoduleinput_10chConfigureChannel(_inputModule* inputModule, uint8_t channel,
									 uint8_t func, uint8_t pull_up,
									 uint8_t pull_down);

/**************************************************************************************
** \brief     Configures the sensor supply outputs on a 6-channel input module.
** \param     inputModule  Pointer to a _inputModule struct that holds the
**                         relevant module configuration.
** \param     supply1      State of sensor supply 1 (use INPUTSENSSUPPLYON or INPUTSENSSUPPLYOFF).
** \param     supply2      State of sensor supply 2 (use INPUTSENSSUPPLYON or INPUTSENSSUPPLYOFF).
** \param     supply3      State of sensor supply 3 (use INPUTSENSSUPPLYON or INPUTSENSSUPPLYOFF).
** \return    0 if successful, negative errno value if failed.
***************************************************************************************/
int GOmoduleinput_6chConfigureSupply(_inputModule* inputModule, uint8_t supply1,
								   uint8_t supply2, uint8_t supply3);

/**************************************************************************************
** \brief     Configures the sensor supply output on a 10-channel input module.
** \param     inputModule  Pointer to a _inputModule struct that holds the
**                         relevant module configuration.
** \param     supply1      State of the sensor supply (use INPUTSENSSUPPLYON or INPUTSENSSUPPLYOFF).
** \return    0 if successful, negative errno value if failed.
***************************************************************************************/
int GOmoduleinput_10chConfigureSupply(_inputModule* inputModule, uint8_t supply1);

#endif /* GO_MODULE_INPUT_H */
