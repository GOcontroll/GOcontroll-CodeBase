/**************************************************************************************
 * \file   GO_board.h
 * \brief  Combined board-level interface for GOcontroll hardware.
 *         Covers program lifecycle, CPU scheduling, display backlight, license
 *         verification, power management (supply voltage, controller-active),
 *         status LEDs, hardware version detection, CPU temperature, and the
 *         on-board accelerometer/gyroscope.
 *
 *         Platform selection via preprocessor define:
 *           GOCONTROLL_IOT   →  STM32H5 (Moduline S1)
 *           GOCONTROLL_LINUX →  Linux/IMX8 (Moduline L4 / Moduline M1)
 * \internal
 *----------------------------------------------------------------------------------------
 *                          C O P Y R I G H T
 *----------------------------------------------------------------------------------------
 * Copyright 2024-2025 (c) by GOcontroll http://www.gocontroll.com All rights reserved
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

#ifndef GO_BOARD_H
#define GO_BOARD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************************
 * Data declarations
 ****************************************************************************************/

typedef struct {
	uint8_t ledControl;
	uint8_t adcControl;
	uint8_t moduleNumber;
	uint8_t moduleOccupancy[8][7];
} _hardwareConfig;

typedef struct {
	uint16_t batteryVoltage;
	uint16_t k15aVoltage;
	uint16_t k15bVoltage;
	uint16_t k15cVoltage;
} _controllerSupply;

typedef enum {
	LED_COLOR_RED	= 0x01,
	LED_COLOR_GREEN	= 0x02,
	LED_COLOR_BLUE	= 0x03
} _ledColor;

typedef struct {
	float acc_x;   /* m/s² or mg depending on platform */
	float acc_y;
	float acc_z;
	float gyro_x;  /* mdps */
	float gyro_y;
	float gyro_z;
	float temp;    /* °C   */
} GOcontrollControllerInfo_t;

/****************************************************************************************
 * Function prototypes — GOcontrollBoard
 ****************************************************************************************/
#ifdef GOCONTROLL_IOT

/*
 * TODO: add STM32H5 board-level function declarations here.
 */

#elif defined(GOCONTROLL_LINUX)

/**************************************************************************************
** \brief     Register SIGTERM/SIGINT handlers and store the Simulink terminate callback.
**            Call with a function pointer on startup to register the handler.
**            Call with NULL to execute the shutdown sequence and exit.
** \param     Terminate  pointer to the Simulink terminate function, or NULL to shut down
** \return    none.
***************************************************************************************/
void GO_board_exit_program(void *Terminate);

/**************************************************************************************
** \brief     Pin the process to CPU core 3 using sched_setaffinity.
** \return    none.
***************************************************************************************/
void GO_board_set_cpu_affinity(void);

/**************************************************************************************
** \brief     Control the display backlight brightness via the sysfs backlight interface.
** \param     brightness  target brightness value (0–100)
** \param     call_type   0 = initialise, 1 = set, 2 = terminate
** \return    0 on success, error code on failure.
***************************************************************************************/
int GO_board_set_screen_brightness(uint8_t brightness,
								uint8_t call_type);

/**************************************************************************************
** \brief     Execute a shell command in a child process (non-blocking).
**            Forks a child process, runs the command via system(), and exits
**            the child. The parent continues immediately.
** \param     command  shell command string to execute
** \return    none.
***************************************************************************************/
void GO_board_exec_shell(const char *command);

#endif /* GOCONTROLL_LINUX */

/****************************************************************************************
 * Function prototypes — GOcontrollControllerPower
 ****************************************************************************************/

/**************************************************************************************
** \brief     Read a supply voltage from the cached ADC data.
**            The ADC thread must be running (via GO_board_controller_power_start_adc_thread)
**            before calling this function.
** \param     supply  Supply index: 1=K30/battery, 2=K15-A, 3=K15-B, 4=K15-C
**                    (STM32H5: only 1 and 2 are valid)
** \param     value   Pointer to variable to store the voltage in mV.
** \return    0 on success, -1 on invalid supply index.
***************************************************************************************/
int GO_board_controller_power_voltage(uint8_t supply, uint16_t* value);

/**************************************************************************************
** \brief     Start the background ADC sampling thread.
**            STM32H5: creates a CMSIS-RTOS osThread.
**            Linux:   creates a POSIX pthread.
** \param     sample_time_ms  Sampling interval in milliseconds.
** \return    none.
***************************************************************************************/
void GO_board_controller_power_start_adc_thread(uint32_t sample_time_ms);

/**************************************************************************************
** \brief     Stop the background ADC sampling thread.
**            STM32H5: signals the thread to exit.
**            Linux:   signals and joins the thread.
** \return    none.
***************************************************************************************/
void GO_board_controller_power_stop_adc_thread(void);

/**************************************************************************************
** \brief     Control the controller-active relay/output.
**            STM32H5: writes the KL15_CONTROLLER_UCO GPIO pin.
**            Linux:   writes to /sys/class/leds/power-active/brightness.
** \param     state  1 = active (on), 0 = inactive (off).
** \return    0 on success, -1 on failure.
***************************************************************************************/
int GO_board_controller_power_controller_active(uint8_t state);

/****************************************************************************************
 * Function prototypes — GOcontrollStatusLeds
 ****************************************************************************************/

/**************************************************************************************
** \brief     Initialize the status LEDs.
**            STM32H5: starts TIM3 PWM channels 1-3 (PC6=Blue, PC7=Red, PC8=Green).
**            Linux:   initializes RUKR I2C LED controller if present.
** \return    0 on success, -1 on failure.
***************************************************************************************/
int GO_board_status_leds_initialize(void);

/**************************************************************************************
** \brief     Control a single status LED color channel.
** \param     led    LED index (STM32H5: always 1 / Linux: 1-4)
** \param     color  LED color channel (_ledColor enum: RED, GREEN, BLUE)
** \param     value  Brightness (STM32H5: 0-255 PWM / Linux: 0=off, non-zero=on)
** \return    0 on success, -1 on failure.
***************************************************************************************/
int GO_board_status_leds_led_control(uint8_t led, _ledColor color, uint8_t value);

/****************************************************************************************
 * Function prototypes — GOcontrollControllerInfo (board subset)
 ****************************************************************************************/

/**************************************************************************************
** \brief     Get the accelerometer sensor temperature (LSM6DS3).
**            Returns the last value read by AccProcess(), updated at 10 Hz.
** \return    Temperature in degrees Celsius, or 0 if not yet available.
***************************************************************************************/
float GO_board_controller_info_get_temperature(void);

/**************************************************************************************
** \brief     Read hardware version from the controller and populate hardwareConfig.
** \return    none.
***************************************************************************************/
void GO_board_get_hardware_version(void);

/**************************************************************************************
** \brief     Start the ControllerInfo background task.
**            Initialises the accelerometer hardware and creates the cyclic task/thread.
**            Must be called once before the scheduler starts (from ert_stm_main.tlc).
** \return    none.
***************************************************************************************/
void GO_board_controller_info_task_start(void);

/**************************************************************************************
** \brief     Copy the latest accelerometer/gyroscope/temperature data into *out.
**            Thread-safe; may be called from any task or the model step.
** \param     out  destination struct
** \return    none.
***************************************************************************************/
void GO_board_controller_info_get_data(GOcontrollControllerInfo_t *out);

#ifdef __cplusplus
}
#endif

#endif /* GO_BOARD_H */

/* end of GO_board.h */
