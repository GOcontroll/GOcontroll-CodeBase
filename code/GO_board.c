/**************************************************************************************
 * \file   GO_board.c
 * \brief  Combined board-level implementation for GOcontroll hardware.
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

/****************************************************************************************
 * Include files — common
 ****************************************************************************************/
#include "GO_board.h"

#include <stdint.h>

/****************************************************************************************
 ****************************************************************************************
 * STM32H5 (GOCONTROLL_IOT) implementations
 ****************************************************************************************
 ****************************************************************************************/
#ifdef GOCONTROLL_IOT

/* ---- ControllerPower ---- */
#include "adc.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "SEGGER_RTT.h"

/* ---- StatusLeds ---- */
#include "stm32h5xx_hal.h"
#include "tim.h"

/* ---- ControllerInfo ---- */
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "i2c.h"

/*
 * TODO: Implement board-level functions for STM32H5.
 */

/* =====================================
 * ControllerPower — S1
 * ===================================== */

/* ADC channel mapping: index 0 = K15/battery (CH9), index 1 = K30 (CH5) */
static uint32_t s_channels[2] = {ADC_CHANNEL_9, ADC_CHANNEL_5};

static _controllerSupply s_controllerSupply;

static struct {
	uint8_t  thread_run;
	uint32_t sample_time;
} s_adcThreadArgs;

static osThreadId_t s_adcThreadId;

static int readAdc(uint8_t index, uint16_t* value) {
	ADC_ChannelConfTypeDef ADCChannelConfiguration = {0};

	HAL_ADC_Stop(&hadc1);
	ADCChannelConfiguration.Channel     = s_channels[index];
	ADCChannelConfiguration.Rank        = ADC_REGULAR_RANK_1;
	ADCChannelConfiguration.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
	ADCChannelConfiguration.SingleDiff  = ADC_SINGLE_ENDED;
	ADCChannelConfiguration.OffsetNumber = ADC_OFFSET_NONE;
	ADCChannelConfiguration.Offset       = 0u;

	HAL_ADC_ConfigChannel(&hadc1, &ADCChannelConfiguration);
	HAL_ADC_Start(&hadc1);

	if (HAL_ADC_PollForConversion(&hadc1, 500) != HAL_OK) {
		return -1;
	}

	*value = (uint16_t)(HAL_ADC_GetValue(&hadc1) * 8.7);
	return 0;
}

static void adcThreadFunc(void* arg) {
	(void)arg;
	uint32_t tick = osKernelGetTickCount();

	while (s_adcThreadArgs.thread_run) {
		tick += s_adcThreadArgs.sample_time;
		readAdc(1, &s_controllerSupply.batteryVoltage);
		readAdc(0, &s_controllerSupply.k15aVoltage);
		osDelayUntil(tick);
	}
	osThreadExit();
}


/**************************************************************************************
** \brief     Read a supply voltage from the cached ADC data.
**            The ADC thread must be running (via GO_board_controller_power_start_adc_thread)
**            before calling this function.
** \param     supply  Supply index: 1=K30/battery, 2=K15-A (STM32H5 only supports 1 and 2)
** \param     value   Pointer to variable to store the voltage in mV.
** \return    0 on success, -1 on invalid supply index.
***************************************************************************************/
int GO_board_controller_power_voltage(uint8_t supply, uint16_t* value) {
	switch (supply) {
		case 1:
			*value = s_controllerSupply.batteryVoltage;
			break;
		case 2:
			*value = s_controllerSupply.k15aVoltage;
			break;
		default:
			return -1;
	}
	return 0;
}

/**************************************************************************************
** \brief     Start the background ADC sampling thread.
**            Creates a CMSIS-RTOS osThread that periodically samples ADC channels.
** \param     sample_time_ms  Sampling interval in milliseconds.
** \return    none.
***************************************************************************************/
void GO_board_controller_power_start_adc_thread(uint32_t sample_time_ms) {
	static const osThreadAttr_t adc_thread_attributes = {
		.name       = "adc_thread",
		.stack_size = 128 * 4,
		.priority   = (osPriority_t) osPriorityNormal,
	};
	s_adcThreadArgs.sample_time = sample_time_ms;
	s_adcThreadArgs.thread_run  = 1;
	s_adcThreadId = osThreadNew(adcThreadFunc, NULL, &adc_thread_attributes);
}


/**************************************************************************************
** \brief     Stop the background ADC sampling thread.
**            Signals the CMSIS-RTOS thread to exit by clearing the run flag.
** \return    none.
***************************************************************************************/
void GO_board_controller_power_stop_adc_thread(void) {
	s_adcThreadArgs.thread_run = 0;
}

/**************************************************************************************
** \brief     Control the controller-active relay/output.
**            Writes the KL15_CONTROLLER_UCO GPIO pin on STM32H5.
** \param     state  1 = active (on), 0 = inactive (off).
** \return    0 on success, -1 on failure.
***************************************************************************************/
int GO_board_controller_power_controller_active(uint8_t state) {
	HAL_GPIO_WritePin(KL15_CONTROLLER_UCO_GPIO_Port, KL15_CONTROLLER_UCO_Pin,
	                  (GPIO_PinState)state);
	return 0;
}

/* =====================================
 * StatusLeds — S1
 * ===================================== */

/**************************************************************************************
** \brief     Initialize the status LEDs.
**            Resets all PWM compare registers to 0, then starts TIM3 PWM channels
**            1-3 (PC6=Blue, PC7=Red, PC8=Green).
** \return    0 on success, -1 on failure.
***************************************************************************************/
int GO_board_status_leds_initialize(void) {
	/* Reset all channels before starting PWM */
	GO_board_status_leds_led_control(1, LED_COLOR_RED,   0);
	GO_board_status_leds_led_control(1, LED_COLOR_GREEN, 0);
	GO_board_status_leds_led_control(1, LED_COLOR_BLUE,  0);
	/* Start PWM on TIM3: CH1=Blue(PC6), CH2=Red(PC7), CH3=Green(PC8) */
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	return 0;
}


/**************************************************************************************
** \brief     Control a single status LED color channel via TIM3 PWM.
** \param     led    LED index (STM32H5: must be 1)
** \param     color  LED color channel (_ledColor enum: RED=CH2, GREEN=CH3, BLUE=CH1)
** \param     value  Brightness value (0-255 PWM duty cycle)
** \return    0 on success, -1 on failure.
***************************************************************************************/
int GO_board_status_leds_led_control(uint8_t led, _ledColor color, uint8_t value) {
	if (led != 1) {
		return -1;
	}
	switch (color) {
		case LED_COLOR_RED:
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)value);
			break;
		case LED_COLOR_GREEN:
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)value);
			break;
		case LED_COLOR_BLUE:
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)value);
			break;
	}
	return 0;
}

/* =====================================
 * ControllerInfo — S1
 * ===================================== */

extern osThreadId_t model_step_thread;
osThreadId_t xcp_thread __attribute__((weak)) = NULL;
extern _hardwareConfig hardwareConfig;

#define ACCELERO_ADDR (0xD6u)

static GOcontrollControllerInfo_t s_info_data;
static osMutexId_t                s_info_data_lock = 0;

/* Exposed to GO_controller_info.c via extern */
uint32_t go_board_model_stack_hwm = 0;
uint32_t go_board_xcp_stack_hwm   = 0;
uint32_t go_board_free_heap       = 0;
uint8_t  go_board_cpu_load        = 0;

#define CONTROLLER_INFO_MAX_TASKS  16u
static TaskStatus_t               s_task_status[CONTROLLER_INFO_MAX_TASKS];

static void AccInit(void) {
	uint8_t data;

	/* WHO_AM_I (0x0F) — expected value 0x69 for LSM6DS3 */
	HAL_I2C_Mem_Read(&hi2c2, ACCELERO_ADDR, 0x0Fu, 1u, &data, 1u, 1000u);

	/* INT1_CTRL (0x0D): INT1 on gyro data-ready */
	data = 0x02u;
	HAL_I2C_Mem_Write(&hi2c2, ACCELERO_ADDR, 0x0Du, 1u, &data, 1u, 1000u);

	/* INT2_CTRL (0x0E): INT2 on accel data-ready */
	data = 0x01u;
	HAL_I2C_Mem_Write(&hi2c2, ACCELERO_ADDR, 0x0Eu, 1u, &data, 1u, 1000u);

	/* CTRL1_XL (0x10): ODR 104 Hz, ±16 g */
	data = 0x44u;
	HAL_I2C_Mem_Write(&hi2c2, ACCELERO_ADDR, 0x10u, 1u, &data, 1u, 1000u);

	/* CTRL2_G (0x11): ODR 104 Hz, 250 dps */
	data = 0x40u;
	HAL_I2C_Mem_Write(&hi2c2, ACCELERO_ADDR, 0x11u, 1u, &data, 1u, 1000u);
}

static void AccProcess(void) {
	static uint8_t data[6];
	int16_t        result;
	float          acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, temp;

	/* Temperature: OUT_TEMP_L / OUT_TEMP_H (0x20-0x21) */
	HAL_I2C_Mem_Read(&hi2c2, ACCELERO_ADDR, 0x20u, 1u, data, 2u, 500u);
	result = (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8u));
	temp = (float)result / 256.0f + 25.0f;

	/* Gyroscope: OUTX_L_G ... OUTZ_H_G (0x22-0x27), 8.75 mdps/LSB */
	HAL_I2C_Mem_Read(&hi2c2, ACCELERO_ADDR, 0x22u, 1u, data, 6u, 500u);
	result = (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8u));
	gyro_x = (float)result * 8.75f;
	result = (int16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8u));
	gyro_y = (float)result * 8.75f;
	result = (int16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8u));
	gyro_z = (float)result * 8.75f;

	/* Accelerometer: OUTX_L_XL ... OUTZ_H_XL (0x28-0x2D), 0.488 mg/LSB */
	HAL_I2C_Mem_Read(&hi2c2, ACCELERO_ADDR, 0x28u, 1u, data, 6u, 500u);
	result = (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8u));
	acc_x = (float)result * 0.488f;
	result = (int16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8u));
	acc_y = (float)result * 0.488f;
	result = (int16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8u));
	acc_z = (float)result * 0.488f;

	if (!osMutexAcquire(s_info_data_lock, 1u)) {
		s_info_data.acc_x  = acc_x;
		s_info_data.acc_y  = acc_y;
		s_info_data.acc_z  = acc_z;
		s_info_data.gyro_x = gyro_x;
		s_info_data.gyro_y = gyro_y;
		s_info_data.gyro_z = gyro_z;
		s_info_data.temp   = temp;
		osMutexRelease(s_info_data_lock);
	}
}

static void ControllerInfoTask(void *args) {
	(void)args;
	MX_I2C2_Init();
	AccInit();
	uint32_t tick = 0u;
	uint32_t prev_total_time = 0u;
	uint32_t prev_idle_time  = 0u;
	for (;;) {
		AccProcess();
		/* Read stack/heap/cpu every 10 iterations (1 s) */
		if (++tick >= 10u) {
			tick = 0u;
			go_board_model_stack_hwm =
			    (uint32_t)uxTaskGetStackHighWaterMark((TaskHandle_t)model_step_thread)
			    * sizeof(StackType_t);
			go_board_xcp_stack_hwm = (xcp_thread != NULL)
			    ? (uint32_t)uxTaskGetStackHighWaterMark((TaskHandle_t)xcp_thread) * sizeof(StackType_t)
			    : 0u;
			go_board_free_heap = (uint32_t)xPortGetFreeHeapSize();
			//SEGGER_RTT_printf(0, "[Stack] model_step: %u bytes free\n", go_board_model_stack_hwm);
			//SEGGER_RTT_printf(0, "[Stack] xcp_thread: %u bytes free\n", go_board_xcp_stack_hwm);
			//SEGGER_RTT_printf(0, "[Heap]  free: %u bytes, min ever: %u bytes\n",
			//                  go_board_free_heap,
			//                  (unsigned)xPortGetMinimumEverFreeHeapSize());
			uint32_t    totalRunTime;
			UBaseType_t n = uxTaskGetSystemState(s_task_status,
			                                     CONTROLLER_INFO_MAX_TASKS,
			                                     &totalRunTime);
			if (n > 0u && totalRunTime > 0u) {
				uint32_t total_delta = totalRunTime - prev_total_time;
				if (total_delta > 0u) {
					for (UBaseType_t i = 0u; i < n; i++) {
						if (s_task_status[i].uxCurrentPriority == tskIDLE_PRIORITY) {
							uint32_t idle_delta = s_task_status[i].ulRunTimeCounter
							                      - prev_idle_time;
							prev_idle_time = s_task_status[i].ulRunTimeCounter;
							go_board_cpu_load = (uint8_t)(100u
							    - (uint8_t)(((uint64_t)idle_delta * 100ULL) / total_delta));
							break;
						}
					}
				}
				prev_total_time = totalRunTime;
				for (UBaseType_t i = 0u; i < n; i++) {
				//	uint32_t pct = (s_task_status[i].ulRunTimeCounter * 100UL)
				//	               / totalRunTime;
				//	SEGGER_RTT_printf(0, "[CPU]   %-16s %2u%%\n",
				//	                  s_task_status[i].pcTaskName, pct);
				}
			}
		}
		osDelay(100u);
	}
	osThreadExit();
}

static osThreadId_t          s_info_task_handle;
static const osThreadAttr_t  s_info_task_attrs = {
	.name       = "ControllerInfo",
	.stack_size = 256u * 4u,
	.priority   = (osPriority_t)osPriorityLow,
};


/**************************************************************************************
** \brief     Get the accelerometer sensor temperature.
**            Returns the last temperature reading from the LSM6DS3 sensor,
**            updated by AccProcess() at 10 Hz.
** \return    Temperature in degrees Celsius, or 0 if not yet available.
***************************************************************************************/
float GO_board_controller_info_get_temperature(void) {
	float t = 0.0f;
	if (!osMutexAcquire(s_info_data_lock, 1u)) {
		t = s_info_data.temp;
		osMutexRelease(s_info_data_lock);
	}
	return t;
}

/**************************************************************************************
** \brief     Read hardware version from the controller and populate hardwareConfig.
**            STM32H5: sets fixed defaults (2 modules, no LED/ADC control).
** \return    none.
***************************************************************************************/
void GO_board_get_hardware_version(void) {
	memset(&hardwareConfig, 0, sizeof(_hardwareConfig));
	hardwareConfig.moduleNumber = 2;
	hardwareConfig.ledControl   = 0;
	hardwareConfig.adcControl   = 0;
}


/**************************************************************************************
** \brief     Start the ControllerInfo background task.
**            Creates the CMSIS-RTOS mutex and spawns the ControllerInfoTask thread
**            which initialises the LSM6DS3 and polls it at 100 ms intervals.
** \return    none.
***************************************************************************************/
void GO_board_controller_info_task_start(void) {
	s_info_data_lock   = osMutexNew(NULL);
	s_info_task_handle = osThreadNew(ControllerInfoTask, NULL, &s_info_task_attrs);
}

/**************************************************************************************
** \brief     Copy the latest accelerometer/gyroscope/temperature data into *out.
**            Thread-safe via CMSIS-RTOS mutex; may be called from any task.
** \param     out  destination struct
** \return    none.
***************************************************************************************/
void GO_board_controller_info_get_data(GOcontrollControllerInfo_t *out) {
	if (!osMutexAcquire(s_info_data_lock, 1u)) {
		*out = s_info_data;
		osMutexRelease(s_info_data_lock);
	}
}

/****************************************************************************************
 ****************************************************************************************
 * Linux (GOCONTROLL_LINUX) implementations
 ****************************************************************************************
 ****************************************************************************************/
#elif defined(GOCONTROLL_LINUX)

/* ---- Board ---- */
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <sched.h>

/* ---- ControllerPower ---- */
#include <pthread.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

/* ---- ControllerInfo ---- */
#include <stdbool.h>

#include "GO_xcp.h"
#include "GO_communication_modules.h"

extern _hardwareConfig hardwareConfig;

/****************************************************************************************
 * IIO sysfs helpers — replace libiio for MCP3004 and LIS2DW12 access
 ****************************************************************************************/

/* Look up an IIO device by its kernel name (e.g. "mcp3004", "lis2dw12") by
 * scanning /sys/bus/iio/devices/iio:device*. Writes the matching basename
 * ("iio:deviceN") into basename_out so the caller can build both sysfs and
 * /dev/<basename> paths. Returns 0 on success, -1 if not found. */
static int iio_find_device(const char *name, char *basename_out, size_t out_size) {
	DIR *dir = opendir("/sys/bus/iio/devices");
	if (!dir) return -1;

	int found = -1;
	struct dirent *ent;
	while ((ent = readdir(dir)) != NULL) {
		if (strncmp(ent->d_name, "iio:device", 10) != 0) continue;

		char name_path[320];
		snprintf(name_path, sizeof name_path,
				 "/sys/bus/iio/devices/%s/name", ent->d_name);

		int fd = open(name_path, O_RDONLY);
		if (fd < 0) continue;

		char buf[64] = {0};
		ssize_t n = read(fd, buf, sizeof buf - 1);
		close(fd);
		if (n <= 0) continue;
		if (buf[n - 1] == '\n') n--;
		buf[n] = '\0';

		if (strcmp(buf, name) == 0) {
			snprintf(basename_out, out_size, "%s", ent->d_name);
			found = 0;
			break;
		}
	}
	closedir(dir);
	return found;
}

/* One-shot write of a string value to a sysfs attribute. Returns 0/-1. */
static int sysfs_write_str(const char *path, const char *value) {
	int fd = open(path, O_WRONLY);
	if (fd < 0) return -1;
	ssize_t n = write(fd, value, strlen(value));
	close(fd);
	return (n < 0) ? -1 : 0;
}

/****************************************************************************************
 * Board — Linux
 ****************************************************************************************/

#ifndef __min
#define __min(a, b) (((a) < (b)) ? (a) : (b))
#endif


/**************************************************************************************
** \brief     Pin the process to CPU core 3 using sched_setaffinity.
** \return    none.
***************************************************************************************/
void GO_board_set_cpu_affinity(void) {
	cpu_set_t mask;
	CPU_ZERO(&mask);
	CPU_SET(3, &mask);
	sched_setaffinity(0, sizeof(mask), &mask);
}

static void GocontrollProcessorboard_ProgramStop(int x, siginfo_t *y, void *z) {
	(void)x; (void)y; (void)z;
	printf("shutting down\n");
	GO_board_exit_program(NULL);
}


/**************************************************************************************
** \brief     Register SIGTERM/SIGINT handlers and store the Simulink terminate callback.
**            Call with a function pointer on startup to register the handler.
**            Call with NULL to execute the shutdown sequence and exit.
** \param     Terminate  pointer to the Simulink terminate function, or NULL to shut down
** \return    none.
***************************************************************************************/
void GO_board_exit_program(void *Terminate) {
	static void (*TerminateFunction)(void);

	if (Terminate != NULL) {
		static struct sigaction _sigact;

		memset(&_sigact, 0, sizeof(_sigact));
		_sigact.sa_sigaction = GocontrollProcessorboard_ProgramStop;
		_sigact.sa_flags = SA_SIGINFO;

		/* Register CTRL-C and daemon terminate signals for a gentle shutdown */
		sigaction(SIGINT, &_sigact, NULL);
		sigaction(SIGTERM, &_sigact, NULL);

		TerminateFunction = Terminate;
		return;
	}
	GO_xcp_stop_connection();
	TerminateFunction();
	exit(0);
}


/**************************************************************************************
** \brief     Control the display backlight brightness via the sysfs backlight interface.
** \param     brightness  target brightness value (0-100)
** \param     call_type   0 = initialise, 1 = set, 2 = terminate
** \return    0 on success, error code on failure.
***************************************************************************************/
int GO_board_set_screen_brightness(uint8_t brightness,
								uint8_t call_type) {
	DIR *d;
	struct dirent *dir;
	uint8_t temp_brightness = brightness;
	static int brightness_file = 0;
	static uint8_t old_brightness = 0;
	/* Buffer sized to avoid truncation warning on long sysfs paths */
	char buff[288];

	switch (call_type) {
		case 0:	 /* initialize */
			d = opendir("/sys/class/backlight");
			if (d) {
				while ((dir = readdir(d)) != NULL) {
					if (dir->d_type == DT_LNK) {
						snprintf(buff, sizeof(buff),
								 "/sys/class/backlight/%s/brightness",
								 dir->d_name);
						brightness_file = open(buff, O_WRONLY);
						closedir(d);
						return 0;
					}
				}
				closedir(d);
			}
			return -EIO;

		case 1:	 /* set brightness */
			if (brightness_file > 0 && old_brightness != temp_brightness) {
				if (temp_brightness > 100) {
					temp_brightness = 100;
				}
				old_brightness = temp_brightness;
				snprintf(buff, 5, "%d", temp_brightness);
				write(brightness_file, buff, 3);
			}
			break;

		case 2:	 /* terminate */
			if (brightness_file > 0) {
				close(brightness_file);
			}
			break;

		default:
			return -EINVAL;
	}
	return 0;
}


/**************************************************************************************
** \brief     Execute a shell command in a child process (non-blocking).
**            Forks a child process, runs the command via system(), and exits
**            the child. The parent continues immediately.
** \param     command  shell command string to execute
** \return    none.
***************************************************************************************/
void GO_board_exec_shell(const char *command) {
	pid_t pid = fork();
	if (pid == 0) {
		exit(system(command));
	}
}

/****************************************************************************************
 * ControllerPower — Linux
 ****************************************************************************************/

#define LOW 0

static _controllerSupply s_controllerSupply;

static struct {
	uint8_t  thread_run;
	uint32_t sample_time;
} s_adcThreadArgs;

static pthread_t s_adcThreadId;

/* fd cache for /sys/bus/iio/devices/<mcp3004>/in_voltage{0..3}_raw, indexed
 * by physical MCP3004 channel number (0..3). Physical pin → supply mapping:
 *   ch0 = K15-A,  ch1 = K15-B,  ch2 = K15-C,  ch3 = K30 (battery)
 * This matches the pre-refactor libiio mapping after accounting for libiio's
 * channel enumeration order on this driver ({voltage1, voltage2, voltage3,
 * voltage0}); expressing it as the physical channel here is clearer. */
static int s_mcp_fds[4] = {-1, -1, -1, -1};

static int readAdc(uint8_t supply, uint16_t* value) {
	if (hardwareConfig.adcControl == ADC_MCP3004) {
		char    buf[100];
		ssize_t res;
		int     fd;

		switch (supply) {
			case 1: fd = s_mcp_fds[3]; break; /* K30   on MCP3004 ch3 */
			case 2: fd = s_mcp_fds[0]; break; /* K15-A on MCP3004 ch0 */
			case 3: fd = s_mcp_fds[1]; break; /* K15-B on MCP3004 ch1 */
			case 4: fd = s_mcp_fds[2]; break; /* K15-C on MCP3004 ch2 */
			default: return -1;
		}
		if (fd < 0) return -1;

		res = pread(fd, buf, sizeof buf - 1, 0);
		if (res < 0) res = 0;
		buf[res] = '\0';
		/* 25.54 = ((3.35/1023)/1.5)*11700 */
		*value = (uint16_t)((float)(strtof(buf, NULL) * 25.54));
		return 0;
	}

	if (hardwareConfig.adcControl == ADC_ADS1015) {
		static uint8_t dataTxConfig[3]  = {0};
		static uint8_t dataTxConvert[1] = {0};
		static uint8_t dataRx[2]        = {0};

		static int   i2cDevice    = 0;
		static float decimalFactor = 4.095 / 2047;

		const int addr = 0x48;

		const int convertContact1 = 0xC3;
		const int convertContact2 = 0xD3;
		const int convertContact3 = 0xE3;
		const int convertBattery  = 0xF3;

		dataTxConfig[0] = 0x01;
		dataTxConfig[1] = 0xF3;
		dataTxConfig[2] = 0xE3;

		dataTxConvert[0] = 0x00;

		if      (supply == 1) dataTxConfig[1] = convertBattery;
		else if (supply == 2) dataTxConfig[1] = convertContact1;
		else if (supply == 3) dataTxConfig[1] = convertContact2;
		else if (supply == 4) dataTxConfig[1] = convertContact3;
		else                  return -1;

		if ((i2cDevice = open("/dev/i2c-2", O_RDWR)) < 0) {
			printf("Failed to open the bus for voltage measurement.\n");
			close(i2cDevice);
			return -1;
		}
		if (ioctl(i2cDevice, I2C_SLAVE, addr) < 0) {
			printf("Failed to acquire bus access and/or talk to slave.\n");
			close(i2cDevice);
			return -1;
		}
		if (write(i2cDevice, dataTxConfig, 3) != 3) {
			printf("Failed to write to the i2c bus.\n");
			close(i2cDevice);
			return -1;
		}
		if (write(i2cDevice, dataTxConvert, 1) != 1) {
			printf("Failed to write to the i2c bus.\n");
			close(i2cDevice);
			return -1;
		}
		if (read(i2cDevice, dataRx, 2) != 2) {
			printf("Failed to read from the i2c bus.\n");
			close(i2cDevice);
			return -1;
		}
		close(i2cDevice);

		int16_t valueTemp = (dataRx[0] << 4) | ((dataRx[1] & 0xf0) >> 4);
		if (((dataRx[0] & 0x80) >> 7) == 1) {
			valueTemp = 0;
		}
		*value = (uint16_t)((float)(((valueTemp * decimalFactor) / 1.5) * 11700));
	}

	return 0;
}

static void* adcThreadFunc(void* arg) {
	(void)arg;
	__useconds_t sample_time = (__useconds_t)(s_adcThreadArgs.sample_time * 1000);

	if (hardwareConfig.adcControl == ADC_MCP3004) {
		char devname[32];
		if (iio_find_device("mcp3004", devname, sizeof devname) == 0) {
			for (int i = 0; i < 4; ++i) {
				char path[128];
				snprintf(path, sizeof path,
						 "/sys/bus/iio/devices/%s/in_voltage%d_raw",
						 devname, i);
				s_mcp_fds[i] = open(path, O_RDONLY);
			}
		} else {
			fprintf(stderr, "ADC: could not find iio device 'mcp3004'\n");
		}
	}

	while (s_adcThreadArgs.thread_run) {
		readAdc(1, &s_controllerSupply.batteryVoltage);
		readAdc(2, &s_controllerSupply.k15aVoltage);
		readAdc(3, &s_controllerSupply.k15bVoltage);
		readAdc(4, &s_controllerSupply.k15cVoltage);
		usleep(sample_time);
	}

	for (int i = 0; i < 4; ++i) {
		if (s_mcp_fds[i] >= 0) {
			close(s_mcp_fds[i]);
			s_mcp_fds[i] = -1;
		}
	}
	return 0;
}


/**************************************************************************************
** \brief     Read a supply voltage from the cached ADC data.
**            The ADC thread must be running (via GO_board_controller_power_start_adc_thread)
**            before calling this function.
** \param     supply  Supply index: 1=K30/battery, 2=K15-A, 3=K15-B, 4=K15-C
** \param     value   Pointer to variable to store the voltage in mV.
** \return    0 on success, -1 on invalid supply index.
***************************************************************************************/
int GO_board_controller_power_voltage(uint8_t supply, uint16_t* value) {
	switch (supply) {
		case 1: *value = s_controllerSupply.batteryVoltage; break;
		case 2: *value = s_controllerSupply.k15aVoltage;    break;
		case 3: *value = s_controllerSupply.k15bVoltage;    break;
		case 4: *value = s_controllerSupply.k15cVoltage;    break;
		default: return -1;
	}
	return 0;
}

/**************************************************************************************
** \brief     Start the background ADC sampling thread.
**            Creates a POSIX pthread that periodically samples all four supply voltages.
** \param     sample_time_ms  Sampling interval in milliseconds.
** \return    none.
***************************************************************************************/
void GO_board_controller_power_start_adc_thread(uint32_t sample_time_ms) {
	s_adcThreadArgs.sample_time = sample_time_ms;
	s_adcThreadArgs.thread_run  = 1;
	pthread_create(&s_adcThreadId, NULL, adcThreadFunc, NULL);
}

/**************************************************************************************
** \brief     Stop the background ADC sampling thread.
**            Clears the run flag and joins the POSIX pthread.
** \return    none.
***************************************************************************************/
void GO_board_controller_power_stop_adc_thread(void) {
	s_adcThreadArgs.thread_run = 0;
	pthread_join(s_adcThreadId, NULL);
}

/**************************************************************************************
** \brief     Control the controller-active relay/output.
**            Writes to /sys/class/leds/power-active/brightness sysfs node.
** \param     state  1 = active (on), 0 = inactive (off).
** \return    0 on success, -1 on failure.
***************************************************************************************/
int GO_board_controller_power_controller_active(uint8_t state) {
	static int controllerActive = 0;

	if (controllerActive == 0) {
		char path[45];
		snprintf(path, 41, "/sys/class/leds/power-active/brightness");
		controllerActive = open(path, O_WRONLY);

		if (-1 == controllerActive) {
			fprintf(stderr, "Error GPIO write controller active!\n");
			return -1;
		}
	}

	static const char s_values_str[] = "01";

	if (1 != write(controllerActive, &s_values_str[LOW == state ? 0 : 1], 1)) {
		fprintf(stderr, "Error GPIO write controller active!\n");
		return -1;
	}
	return 0;
}

/****************************************************************************************
 * StatusLeds — Linux
 ****************************************************************************************/

/**************************************************************************************
** \brief     Initialize the status LEDs.
**            Resets and enables the RUKR I2C LED controller if present on this hardware.
** \return    0 on success, -1 on failure.
***************************************************************************************/
int GO_board_status_leds_initialize(void) {
	if (hardwareConfig.ledControl == LED_RUKR) {
		const uint8_t addr = 0x14;
		uint8_t dataTx[2];
		static int i2cDevice = 0;

		if ((i2cDevice = open("/dev/i2c-2", O_RDWR)) < 0) {
			close(i2cDevice);
			printf("Error I2C open for LED's.\n");
			return -1;
		}
		if (ioctl(i2cDevice, I2C_SLAVE, addr) < 0) {
			close(i2cDevice);
			printf("Error I2C require bus for LED's.\n");
			return -1;
		}
		/* Reset the RUKR controller */
		dataTx[0] = 0x17;
		dataTx[1] = 0xFF;
		if (write(i2cDevice, dataTx, 2) != 2) {
			close(i2cDevice);
			printf("Error I2C write to bus for LED's.\n");
			return -1;
		}
		/* Enable the RUKR controller (Chip_EN) */
		dataTx[0] = 0x00;
		dataTx[1] = 0x40;
		if (write(i2cDevice, dataTx, 2) != 2) {
			close(i2cDevice);
			printf("Error I2C write to bus for LED's.\n");
			return -1;
		}
		close(i2cDevice);
	}
	return 0;
}


/**************************************************************************************
** \brief     Control a single status LED color channel.
** \param     led    LED index (1-4)
** \param     color  LED color channel (_ledColor enum: RED, GREEN, BLUE)
** \param     value  Brightness (0 = off, non-zero = on for GPIO; 0-255 for RUKR)
** \return    0 on success, -1 on failure.
***************************************************************************************/
int GO_board_status_leds_led_control(uint8_t led, _ledColor color, uint8_t value) {
	if (hardwareConfig.ledControl == LED_RUKR) {
		/* RUKR I2C register map: LED1=0x0A, LED2=0x0D, LED3=0x10, LED4=0x13 (+color offset) */
		static uint8_t dataTx[3] = {0};
		const int addr = 0x14;
		static int i2cDevice = 0;

		if      (led == 1) dataTx[0] = 0x0A + color;
		else if (led == 2) dataTx[0] = 0x0D + color;
		else if (led == 3) dataTx[0] = 0x10 + color;
		else if (led == 4) dataTx[0] = 0x13 + color;
		else               return -1;

		dataTx[1] = value;

		if ((i2cDevice = open("/dev/i2c-2", O_RDWR)) < 0) {
			close(i2cDevice);
			printf("Error I2C open for LED's.\n");
			return -1;
		}
		if (ioctl(i2cDevice, I2C_SLAVE, addr) < 0) {
			close(i2cDevice);
			printf("Error I2C require bus for LED's.\n");
			return -1;
		}
		if (write(i2cDevice, dataTx, 2) != 2) {
			close(i2cDevice);
			printf("Error I2C write to bus for LED's.\n");
			return -1;
		}
		close(i2cDevice);
		return 0;

	} else if (hardwareConfig.ledControl == LED_GPIO) {
		/* sysfs LED path: /sys/class/leds/Status<led>-<r|g|b>/brightness */
		char path[40];
		switch (color) {
			case LED_COLOR_RED:
				snprintf(path, 40, "/sys/class/leds/Status%d-r/brightness", led);
				break;
			case LED_COLOR_GREEN:
				snprintf(path, 40, "/sys/class/leds/Status%d-g/brightness", led);
				break;
			case LED_COLOR_BLUE:
				snprintf(path, 40, "/sys/class/leds/Status%d-b/brightness", led);
				break;
		}

		static const char s_values_str[] = "01";
		int ledControl = open(path, O_WRONLY);
		if (ledControl == -1) {
			fprintf(stderr, "Error GPIO write led %d!\n", led);
			close(ledControl);
			return -1;
		}
		if (1 != write(ledControl, &s_values_str[LOW == value ? 0 : 1], 1)) {
			fprintf(stderr, "Error GPIO write led %d!\n", led);
			close(ledControl);
			return -1;
		}
		close(ledControl);

	} else if (hardwareConfig.ledControl == NOT_INSTALLED) {
		return 0;
	}
	return 0;
}

/****************************************************************************************
 * ControllerInfo — Linux
 ****************************************************************************************/

static GOcontrollControllerInfo_t s_info_data;
static pthread_mutex_t            s_info_data_lock;
static pthread_t                  s_info_task_thread;
static uint8_t                    s_info_task_run = 0;


/**************************************************************************************
** \brief     Get the CPU/controller temperature.
**            Reads from the thermal_zone0 sysfs node and converts millidegrees to Celsius.
** \return    Temperature in degrees Celsius, or 0 on failure.
***************************************************************************************/
float GO_board_controller_info_get_temperature(void) {
	int fileId = open("/sys/devices/virtual/thermal/thermal_zone0/temp",
					  O_RDONLY | O_NONBLOCK);
	if (fileId <= 0) {
		close(fileId);
		return 0;
	}
	char tempValue[15] = {0};
	read(fileId, &tempValue[0], 15);
	close(fileId);
	return strtof(tempValue, NULL) / 1000;
}


/**************************************************************************************
** \brief     Read hardware version from the controller and populate hardwareConfig.
**            Reads the devicetree hardware string and sets module count, LED type,
**            and ADC type. Falls back to Moduline L4 defaults on read failure.
** \return    none.
***************************************************************************************/
void GO_board_get_hardware_version(void) {
	int fileId = 0;
	memset(&hardwareConfig, 0, sizeof(_hardwareConfig));
	if ((fileId = open("/sys/firmware/devicetree/base/hardware",
					   O_RDONLY | O_NONBLOCK)) <= 0) {
		close(fileId);
		hardwareConfig.moduleNumber = 8;
		hardwareConfig.ledControl   = LED_RUKR;
		hardwareConfig.adcControl   = ADC_MCP3004;
		printf("Failed to detect hardware! Set default.");
		return;
	}

	char tempValue[30] = {0};
	read(fileId, &tempValue[0], 30);
	close(fileId);

	printf("Detected hardware: ");

	if (strcmp(tempValue, "Moduline IV V3.06-D") == 0) {
		printf("%s", tempValue);
		hardwareConfig.moduleNumber = 8;
		hardwareConfig.ledControl   = LED_RUKR;
		hardwareConfig.adcControl   = ADC_MCP3004;
	} else if (strcmp(tempValue, "Moduline Mini V1.11") == 0) {
		printf("%s", tempValue);
		hardwareConfig.moduleNumber = 4;
		hardwareConfig.ledControl   = LED_RUKR;
		hardwareConfig.adcControl   = ADC_MCP3004;
	} else if (strcmp(tempValue, "Moduline Mini V1.03") == 0) {
		printf("%s", tempValue);
		hardwareConfig.moduleNumber = 4;
		hardwareConfig.ledControl   = LED_RUKR;
		hardwareConfig.adcControl   = ADC_ADS1015;
	} else if (strcmp(tempValue, "Moduline Mini V1.05") == 0) {
		printf("%s", tempValue);
		hardwareConfig.moduleNumber = 4;
		hardwareConfig.ledControl   = LED_RUKR;
		hardwareConfig.adcControl   = ADC_MCP3004;
	} else if (strcmp(tempValue, "Moduline Mini V1.06") == 0) {
		printf("%s", tempValue);
		hardwareConfig.moduleNumber = 4;
		hardwareConfig.ledControl   = LED_RUKR;
		hardwareConfig.adcControl   = ADC_MCP3004;
	} else if (strcmp(tempValue, "Moduline Mini V1.07") == 0) {
		printf("%s", tempValue);
		hardwareConfig.moduleNumber = 4;
		hardwareConfig.ledControl   = LED_RUKR;
		hardwareConfig.adcControl   = ADC_MCP3004;
	} else if (strcmp(tempValue, "Moduline Mini V1.10") == 0) {
		printf("%s", tempValue);
		hardwareConfig.moduleNumber = 4;
		hardwareConfig.ledControl   = LED_RUKR;
		hardwareConfig.adcControl   = ADC_MCP3004;
	} else if (strcmp(tempValue, "Moduline IV V3.00") == 0) {
		printf("%s", tempValue);
		hardwareConfig.moduleNumber = 8;
		hardwareConfig.ledControl   = LED_GPIO;
		hardwareConfig.adcControl   = ADC_ADS1015;
	} else if (strcmp(tempValue, "Moduline IV V3.01") == 0) {
		printf("%s", tempValue);
		hardwareConfig.moduleNumber = 8;
		hardwareConfig.ledControl   = LED_GPIO;
		hardwareConfig.adcControl   = ADC_ADS1015;
	} else if (strcmp(tempValue, "Moduline IV V3.02") == 0) {
		printf("%s", tempValue);
		hardwareConfig.moduleNumber = 8;
		hardwareConfig.ledControl   = LED_RUKR;
		hardwareConfig.adcControl   = ADC_ADS1015;
	} else if (strcmp(tempValue, "Moduline IV V3.03") == 0) {
		printf("%s", tempValue);
		hardwareConfig.moduleNumber = 8;
		hardwareConfig.ledControl   = LED_RUKR;
		hardwareConfig.adcControl   = ADC_ADS1015;
	} else if (strcmp(tempValue, "Moduline IV V3.04") == 0) {
		printf("%s", tempValue);
		hardwareConfig.moduleNumber = 8;
		hardwareConfig.ledControl   = LED_RUKR;
		hardwareConfig.adcControl   = ADC_ADS1015;
	} else if (strcmp(tempValue, "Moduline IV V3.05") == 0) {
		printf("%s", tempValue);
		hardwareConfig.moduleNumber = 8;
		hardwareConfig.ledControl   = LED_RUKR;
		hardwareConfig.adcControl   = ADC_ADS1015;
	} else if (strcmp(tempValue, "Moduline IV V3.06") == 0) {
		printf("%s", tempValue);
		hardwareConfig.moduleNumber = 8;
		hardwareConfig.ledControl   = LED_RUKR;
		hardwareConfig.adcControl   = ADC_MCP3004;
	} else if (strcmp(tempValue, "Moduline Display V1.01") == 0) {
		printf("%s", tempValue);
		hardwareConfig.moduleNumber = 2;
		hardwareConfig.ledControl   = NOT_INSTALLED;
		hardwareConfig.adcControl   = ADC_MCP3004;
	} else if (strcmp(tempValue, "Moduline Display V1.02") == 0) {
		printf("%s", tempValue);
		hardwareConfig.moduleNumber = 2;
		hardwareConfig.ledControl   = NOT_INSTALLED;
		hardwareConfig.adcControl   = ADC_MCP3004;
	} else if (strcmp(tempValue, "Moduline Display V1.03") == 0) {
		printf("%s", tempValue);
		hardwareConfig.moduleNumber = 2;
		hardwareConfig.ledControl   = NOT_INSTALLED;
		hardwareConfig.adcControl   = ADC_MCP3004;
	} else if (strcmp(tempValue, "Moduline Display V1.04") == 0) {
		printf("%s", tempValue);
		hardwareConfig.moduleNumber = 2;
		hardwareConfig.ledControl   = NOT_INSTALLED;
		hardwareConfig.adcControl   = ADC_MCP3004;
	} else if (strcmp(tempValue, "Moduline Display V1.05") == 0) {
		printf("%s", tempValue);
		hardwareConfig.moduleNumber = 2;
		hardwareConfig.ledControl   = NOT_INSTALLED;
		hardwareConfig.adcControl   = ADC_MCP3004;
	} else if (strcmp(tempValue, "Moduline Display V1.06") == 0) {
		printf("%s", tempValue);
		hardwareConfig.moduleNumber = 2;
		hardwareConfig.ledControl   = NOT_INSTALLED;
		hardwareConfig.adcControl   = ADC_MCP3004;
	} else if (strcmp(tempValue, "Moduline Display V1.07") == 0) {
		printf("%s", tempValue);
		hardwareConfig.moduleNumber = 2;
		hardwareConfig.ledControl   = NOT_INSTALLED;
		hardwareConfig.adcControl   = ADC_MCP3004;
	}

	printf("\n");
}

static void *ControllerInfoThread(void *args) {
	(void)args;
	char devname[32];
	char path[160];
	char buf_enable_path[160];
	int  fd;
	bool run = true;

	if (iio_find_device("lis2dw12", devname, sizeof devname) != 0) {
		fprintf(stderr, "ControllerInfo: could not find iio device 'lis2dw12'\n");
		return NULL;
	}

	/* Disable buffer first so scan_elements writes are accepted even if the
	 * buffer was left enabled by a previous run. */
	snprintf(buf_enable_path, sizeof buf_enable_path,
			 "/sys/bus/iio/devices/%s/buffer/enable", devname);
	sysfs_write_str(buf_enable_path, "0");

	snprintf(path, sizeof path,
			 "/sys/bus/iio/devices/%s/sampling_frequency", devname);
	if (sysfs_write_str(path, "100") != 0) {
		fprintf(stderr, "ControllerInfo: could not set sampling_frequency: %s\n",
				strerror(errno));
		return NULL;
	}

	snprintf(path, sizeof path,
			 "/sys/bus/iio/devices/%s/in_accel_x_scale", devname);
	if (sysfs_write_str(path, "0.009571") != 0) {
		fprintf(stderr, "ControllerInfo: could not set scale: %s\n",
				strerror(errno));
		return NULL;
	}

	const char *axes[] = { "x", "y", "z" };
	for (int i = 0; i < 3; ++i) {
		snprintf(path, sizeof path,
				 "/sys/bus/iio/devices/%s/scan_elements/in_accel_%s_en",
				 devname, axes[i]);
		if (sysfs_write_str(path, "1") != 0) {
			fprintf(stderr, "ControllerInfo: could not enable accel_%s: %s\n",
					axes[i], strerror(errno));
			return NULL;
		}
	}

	snprintf(path, sizeof path,
			 "/sys/bus/iio/devices/%s/buffer/length", devname);
	sysfs_write_str(path, "1");

	if (sysfs_write_str(buf_enable_path, "1") != 0) {
		fprintf(stderr, "ControllerInfo: could not enable buffer: %s\n",
				strerror(errno));
		return NULL;
	}

	char chardev[64];
	snprintf(chardev, sizeof chardev, "/dev/%s", devname);
	fd = open(chardev, O_RDONLY);
	if (fd < 0) {
		fprintf(stderr, "ControllerInfo: could not open %s: %s\n",
				chardev, strerror(errno));
		sysfs_write_str(buf_enable_path, "0");
		return NULL;
	}

	while (s_info_task_run && run) {
		uint8_t frame[6]; /* 3 x int16 LE: x, y, z — LIS2DW12 layout */
		ssize_t bytes = read(fd, frame, sizeof frame);

		if (bytes < 0) {
			if (errno == EAGAIN) continue;
			fprintf(stderr, "ControllerInfo: could not read iio chardev: %s\n",
					strerror(errno));
			run = false;
			continue;
		}
		if (bytes != (ssize_t)sizeof frame) {
			continue;
		}

		/* Scan-element type is "le:s12/16>>4": 12-bit signed data
		 * left-aligned in a 16-bit little-endian container. Arithmetic
		 * shift right by 4 to recover the signed sample; the 0.009571
		 * scale factor is calibrated against this post-shift value
		 * (matches libiio's iio_channel_read semantics). */
		float x = (float)((*(int16_t *)&frame[0]) >> 4) * 0.009571f;
		float y = (float)((*(int16_t *)&frame[2]) >> 4) * 0.009571f;
		float z = (float)((*(int16_t *)&frame[4]) >> 4) * 0.009571f;

		pthread_mutex_lock(&s_info_data_lock);
		s_info_data.acc_x = x;
		s_info_data.acc_y = y;
		s_info_data.acc_z = z;
		pthread_mutex_unlock(&s_info_data_lock);
	}

	close(fd);
	sysfs_write_str(buf_enable_path, "0");
	return NULL;
}


/**************************************************************************************
** \brief     Start the ControllerInfo background task.
**            Initialises the POSIX mutex and spawns the ControllerInfoThread which
**            reads the lis2dw12 accelerometer via the IIO subsystem.
** \return    none.
***************************************************************************************/
void GO_board_controller_info_task_start(void) {
	pthread_mutex_init(&s_info_data_lock, NULL);
	s_info_task_run = 1;
	pthread_create(&s_info_task_thread, NULL, ControllerInfoThread, NULL);
}

/**************************************************************************************
** \brief     Copy the latest accelerometer/gyroscope/temperature data into *out.
**            Thread-safe via POSIX mutex; may be called from any task or model step.
** \param     out  destination struct
** \return    none.
***************************************************************************************/
void GO_board_controller_info_get_data(GOcontrollControllerInfo_t *out) {
	pthread_mutex_lock(&s_info_data_lock);
	*out = s_info_data;
	pthread_mutex_unlock(&s_info_data_lock);
}

#endif /* GOCONTROLL_IOT / GOCONTROLL_LINUX */

/* end of GO_board.c */
