/**************************************************************************************
 * \file   GO_memory.h
 * \brief  Non-volatile memory and diagnostic code storage for GOcontroll hardware.
 *         Combines MemoryEmulation (key-value NVM) and MemoryDiagnostic (DTC storage).
 *
 *         Platform selection via preprocessor define:
 *           GOCONTROLL_IOT  →  STM32H5 (Moduline IOT): TODO — Flash/EEPROM backend
 *           (default)       →  Linux/IMX8 (Moduline IV / Moduline Mini): filesystem
 * \internal
 *----------------------------------------------------------------------------------------
 *                          C O P Y R I G H T
 *----------------------------------------------------------------------------------------
 * Copyright 2024 (c) by GOcontroll http://www.gocontroll.com All rights reserved
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

#ifndef GO_MEMORY_H
#define GO_MEMORY_H

/****************************************************************************************
 * Include files
 ****************************************************************************************/
#include <stdint.h>

/****************************************************************************************
 * Macro definitions — MemoryDiagnostic message types
 ****************************************************************************************/
#define DIAGNOSTICSTART		1
#define DIAGNOSTICFREEZE	2
#define DIAGNOSTICSTOP		3

/****************************************************************************************
 * Function prototypes — MemoryEmulation
 ****************************************************************************************/

/**************************************************************************************
** \brief     Initialize the memory emulation storage.
**            Linux:  creates /usr/mem-sim/ and /etc/go-simulink/ if absent.
**            STM32:  TODO — initialize Flash/EEPROM backend.
** \return    none
***************************************************************************************/
void GO_memory_emulation_initialize(void);

/**************************************************************************************
** \brief     Write a key-value pair to persistent storage.
**            Linux:  key is a file path ("/usr/mem-sim/key" for NVM,
**                    "/dev/shm/key" for volatile).
**            STM32:  TODO — key interpretation to be defined (e.g. index or label).
** \param     key       storage identifier
** \param     value     value to store
** \param     oldValue  previous value; updated on write. Pass NULL on first init.
** \return    none
***************************************************************************************/
void GOmemory_emulationWrite(char *key, float value, float *oldValue);

/**************************************************************************************
** \brief     Read a key-value pair from persistent storage.
**            Linux:  key is a file path.
**            STM32:  TODO — key interpretation to be defined.
** \param     key    storage identifier
** \param     value  output pointer; unchanged if the key is not found
** \return    none
***************************************************************************************/
void GOmemory_emulationRead(char *key, float *value);

/****************************************************************************************
 * Function prototypes — MemoryDiagnostic
 ****************************************************************************************/

/**************************************************************************************
** \brief     Initialize the diagnostic code storage.
**            Linux:  creates /usr/mem-diag/ directory if absent.
**            STM32:  TODO — initialize Flash/EEPROM DTC region.
** \return    none
***************************************************************************************/
void GOmemory_diagnosticInitialize(void);

/**************************************************************************************
** \brief     Write or update a diagnostic trouble code entry.
** \param     spn               Suspect Parameter Number (J1939)
** \param     fmi               Failure Mode Identifier (J1939)
** \param     oc                Occurrence count
** \param     freezedDescription label string for the freeze-frame parameter
** \param     freezedParameter   freeze-frame value
** \param     messageType       DIAGNOSTICSTART, DIAGNOSTICFREEZE or DIAGNOSTICSTOP
** \return    none
***************************************************************************************/
void GOmemory_diagnosticWrite(uint32_t spn, uint8_t fmi, uint8_t oc,
							  char *freezedDescription,
							  float freezedParameter,
							  uint8_t messageType);

/**************************************************************************************
** \brief     Count the number of stored diagnostic codes.
** \return    number of stored DTCs
***************************************************************************************/
uint16_t GOmemory_diagnosticCountCodes(void);

/**************************************************************************************
** \brief     Delete all stored diagnostic codes.
** \return    none
***************************************************************************************/
void GOmemory_diagnosticDeleteAll(void);

/**************************************************************************************
** \brief     Return the raw DTC value at a given index.
** \param     index  zero-based index into the stored DTC list
** \return    encoded DTC value, or 0 if not found
***************************************************************************************/
uint32_t GOmemory_diagnosticCodeOnIndex(uint16_t index);

/**************************************************************************************
** \brief     Delete a single diagnostic code entry.
** \param     spn  Suspect Parameter Number
** \param     fmi  Failure Mode Identifier
** \param     oc   Occurrence count
** \return    none
***************************************************************************************/
void GOmemory_diagnosticDeleteSingle(uint32_t spn, uint8_t fmi, uint8_t oc);

#endif /* GO_MEMORY_H */

/* end of GO_memory.h */
