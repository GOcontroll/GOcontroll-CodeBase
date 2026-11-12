/**************************************************************************************
 * \file   GO_memory.h
 * \brief  Non-volatile memory and diagnostic code storage for GOcontroll hardware.
 *         Combines MemoryEmulation (key-value NVM) and MemoryDiagnostic (DTC storage).
 *
 *         Platform selection via preprocessor define:
 *           GOCONTROLL_STM  →  STM32H5 (Moduline S1): TODO — Flash/EEPROM backend
 *           (default)       →  Linux/IMX8 (Moduline L4 / Moduline M1): filesystem
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
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

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
void GO_memory_emulation_write(char *key, float value, float *oldValue);

/**************************************************************************************
** \brief     Read a key-value pair from persistent storage.
**            Linux:  key is a file path.
**            STM32:  TODO — key interpretation to be defined.
** \param     key    storage identifier
** \param     value  output pointer; unchanged if the key is not found
** \return    none
***************************************************************************************/
void GO_memory_emulation_read(char *key, float *value);

/****************************************************************************************
 * Function prototypes — Application configuration
 *
 * The controller's Web UI keeps the application configuration in one flat JSON object
 * (Configuration -> "Application configuration"):
 *
 *     { "KEY": "value", "OTHER_KEY": "42" }
 *
 * Every value is stored as text, also numbers. Unlike MemoryEmulation this store holds
 * strings, so it is where an application keeps settings such as an MQTT topic, a
 * broker host or a location name. The Web UI rewrites the file atomically on "Save
 * configuration" and does not notify the application: poll GO_config_changed() and
 * re-read when it returns 1.
 *
 * The Web UI only shows keys that are already in the file, so an application seeds
 * its own keys with GO_config_ensure() at start-up.
 *
 * Linux only. On the S1 (no filesystem) every call returns -ENOSYS.
 ****************************************************************************************/

/** Default location of the application configuration on the controller. */
#define GO_CONFIG_PATH "/etc/gocontroll/config.json"

/**************************************************************************************
** \brief     Use another file than GO_CONFIG_PATH (tests, non-standard images).
**            The string is not copied; it must stay valid. NULL restores the default.
** \param     path  path of the JSON file
** \return    none
***************************************************************************************/
void GO_config_set_path(const char *path);

/**************************************************************************************
** \brief     Read a configuration value as text.
** \param     key    configuration key
** \param     value  output buffer, always NUL-terminated when size > 0
** \param     size   size of the output buffer in bytes
** \return    0 on success, -ENOENT if the file or the key is absent, -ENOSPC if the
**            value did not fit (value holds the truncated text), -EBADMSG if the file
**            is not a flat JSON object, -EIO on a read error.
***************************************************************************************/
int GO_config_read_string(const char *key, char *value, size_t size);

/**************************************************************************************
** \brief     Read a configuration value as a number.
** \param     key    configuration key
** \param     value  output; unchanged unless the call returns 0
** \return    0 on success, -EINVAL if the value is not a number, otherwise as
**            GO_config_read_string().
***************************************************************************************/
int GO_config_read_number(const char *key, float *value);

/**************************************************************************************
** \brief     Add a key with a default value if it is not in the file yet.
**            An existing value is never touched. Creates the file (and its
**            directory) when absent. The file is replaced atomically; other keys are
**            preserved.
** \param     key            configuration key
** \param     default_value  text stored when the key is absent
** \return    0 if the key was already present, 1 if it was added, negative errno on
**            failure.
***************************************************************************************/
int GO_config_ensure(const char *key, const char *default_value);

/**************************************************************************************
** \brief     Check whether the configuration file changed since the previous call.
**            Cheap (one stat()); meant to be polled, e.g. once per second.
** \return    1 on the first call and whenever the file was replaced or modified,
**            0 if unchanged, -ENOENT if the file does not exist.
***************************************************************************************/
int GO_config_changed(void);

/****************************************************************************************
 * Function prototypes — MemoryDiagnostic
 ****************************************************************************************/

/**************************************************************************************
** \brief     Initialize the diagnostic code storage.
**            Linux:  creates /usr/mem-diag/ directory if absent.
**            STM32:  TODO — initialize Flash/EEPROM DTC region.
** \return    none
***************************************************************************************/
void GO_memory_diagnostic_initialize(void);

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
void GO_memory_diagnostic_write(uint32_t spn, uint8_t fmi, uint8_t oc,
							  char *freezedDescription,
							  float freezedParameter,
							  uint8_t messageType);

/**************************************************************************************
** \brief     Count the number of stored diagnostic codes.
** \return    number of stored DTCs
***************************************************************************************/
uint16_t GO_memory_diagnostic_count_codes(void);

/**************************************************************************************
** \brief     Delete all stored diagnostic codes.
** \return    none
***************************************************************************************/
void GO_memory_diagnostic_delete_all(void);

/**************************************************************************************
** \brief     Return the raw DTC value at a given index.
** \param     index  zero-based index into the stored DTC list
** \return    encoded DTC value, or 0 if not found
***************************************************************************************/
uint32_t GO_memory_diagnostic_code_on_index(uint16_t index);

/**************************************************************************************
** \brief     Delete a single diagnostic code entry.
** \param     spn  Suspect Parameter Number
** \param     fmi  Failure Mode Identifier
** \param     oc   Occurrence count
** \return    none
***************************************************************************************/
void GO_memory_diagnostic_delete_single(uint32_t spn, uint8_t fmi, uint8_t oc);

#ifdef __cplusplus
}
#endif

#endif /* GO_MEMORY_H */

/* end of GO_memory.h */
