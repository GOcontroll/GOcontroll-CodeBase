/**************************************************************************************
 * \file   bridge_module.c
 * \brief  Drive a bridge module on slot 2 — half-bridge PWM sweep on channel 1,
 *         high-side boolean toggle on channel 2.
 *
 *         The bridge module has 2 channels, each independently configurable.
 *         Common use cases:
 *           BRIDGEFUNC_HALFBRIDGE     — H-bridge / half-bridge, duty cycle 0-1000
 *                                       (0 = 0 %, 1000 = 100 %)
 *           BRIDGEFUNC_HIGHSIDEDUTY   — high-side PWM, duty cycle 0-1000
 *           BRIDGEFUNC_HIGHSIDEBOOL   — high-side on/off (value 0 or 1)
 *           BRIDGEFUNC_LOWSIDEBOOL    — low-side on/off (value 0 or 1)
 *
 *         This example configures:
 *           Channel 1 — BRIDGEFUNC_HALFBRIDGE at 200 Hz
 *                        Sweeps duty cycle from 0 to 1000 and back,
 *                        stepping by 10 every 100 ms (10 cycles × 10 ms).
 *           Channel 2 — BRIDGEFUNC_HIGHSIDEBOOL
 *                        Toggles on/off every 2 s in sync with the sweep.
 *
 *         Feedback read back each cycle:
 *           bridgeModule.current[ch]  — output current in mA
 *           bridgeModule.temperature  — module temperature in 0.1 °C units
 *           bridgeModule.ground       — ground reference in mV
 *
 *         This example demonstrates:
 *           - Setting up a bridge module (slot, channel function and frequency)
 *           - Sending configuration to the module hardware
 *           - Setting output values and reading back current/temperature feedback
 *           - Clean shutdown that turns both channels off on exit
 **************************************************************************************/

#include <stdint.h>
#include <unistd.h>

#include "GO_board.h"
#include "GO_communication_modules.h"
#include "GO_module_bridge.h"
#include "print.h"

/* Module slot */
#define BRIDGE_SLOT  MODULESLOT2

/* Sweep parameters for channel 1 (half-bridge duty cycle) */
#define DUTY_MIN     0
#define DUTY_MAX     1000
#define DUTY_STEP    10

/* Step interval: 10 cycles × 10 ms = 100 ms per step */
#define STEP_CYCLES  10

/* Feedback print interval: 100 cycles × 10 ms = 1 s */
#define PRINT_INTERVAL_CYCLES 100

/* -------------------------------------------------------------------------
 * Module instance — holds configuration, output values and feedback.
 * ------------------------------------------------------------------------- */
static _bridgeModule bridgeModule;

/* -------------------------------------------------------------------------
 * Shutdown callback — turn both channels off before exiting.
 * ------------------------------------------------------------------------- */
static void app_terminate(void)
{
	info("Shutting down — turning both channels off\n");
	bridgeModule.value[BRIDGECHANNEL1] = 0;
	bridgeModule.value[BRIDGECHANNEL2] = 0;
	GO_module_bridge_send_values(&bridgeModule);
}

/* -------------------------------------------------------------------------
 * main
 * ------------------------------------------------------------------------- */
int main(void)
{
	info("=== Bridge module example ===\n");
	info("Slot 2 | CH1: half-bridge sweep 0-100%% | CH2: high-side boolean\n");
	info("Press Ctrl+C to stop.\n\n");

	/* Detect the hardware variant (required before any module function). */
	GO_board_get_hardware_version();

	/* --- Module setup ---------------------------------------------------- */

	/* Assign module slot — validated against detected hardware. */
	GO_module_bridge_set_module_slot(&bridgeModule, BRIDGE_SLOT);

	/* Initialise SPI communication for this slot. */
	GO_communication_modules_initialize(BRIDGE_SLOT);

	/* Channel 1: half-bridge, 200 Hz PWM.
	 * value range: 0 (0 %) to 1000 (100 %) */
	GO_module_bridge_configure_channel(&bridgeModule,
	                                   BRIDGECHANNEL1,
	                                   BRIDGEFUNC_HALFBRIDGE,
	                                   BRIDGEFREQ_200HZ);

	/* Channel 2: high-side boolean on/off.
	 * value: 0 = off, 1 = on */
	GO_module_bridge_configure_channel(&bridgeModule,
	                                   BRIDGECHANNEL2,
	                                   BRIDGEFUNC_HIGHSIDEBOOL,
	                                   BRIDGEFREQ_200HZ);

	/* Send the full configuration to the module over SPI. */
	GO_module_bridge_configuration(&bridgeModule);

	info("Module configured — starting sweep on CH1\n\n");

	/* Register the shutdown callback for SIGTERM and SIGINT. */
	GO_board_exit_program(app_terminate);

	/* --- Application loop (10 ms cycle) ---------------------------------- */
	int     step_cycle   = 0;
	int     print_cycle  = 0;
	int16_t duty         = DUTY_MIN;
	int8_t  direction    = 1;      /* +1 = increasing, -1 = decreasing */

	while (1) {
		/* --- Sweep duty cycle every STEP_CYCLES ----------------------------- */
		if (++step_cycle >= STEP_CYCLES) {
			step_cycle = 0;

			duty += direction * DUTY_STEP;

			if (duty >= DUTY_MAX) {
				duty      = DUTY_MAX;
				direction = -1;
			} else if (duty <= DUTY_MIN) {
				duty      = DUTY_MIN;
				direction = +1;
			}

			bridgeModule.value[BRIDGECHANNEL1] = (uint16_t)duty;

			/* Toggle CH2 in sync with direction changes */
			bridgeModule.value[BRIDGECHANNEL2] = (direction > 0) ? 1u : 0u;
		}

		/* Send output values and receive feedback (current, temperature). */
		GO_module_bridge_send_values(&bridgeModule);

		/* --- Print feedback once per second --------------------------------- */
		if (++print_cycle >= PRINT_INTERVAL_CYCLES) {
			print_cycle = 0;

			info("CH1 duty: %4d/1000 (%3d%%)  CH2: %s  |  "
			     "I1: %4dmA  I2: %4dmA  Temp: %4.1f°C  GND: %umV\n",
			     bridgeModule.value[BRIDGECHANNEL1],
			     bridgeModule.value[BRIDGECHANNEL1] / 10,
			     bridgeModule.value[BRIDGECHANNEL2] ? "ON " : "OFF",
			     bridgeModule.current[BRIDGECHANNEL1],
			     bridgeModule.current[BRIDGECHANNEL2],
			     bridgeModule.temperature / 10.0f,
			     bridgeModule.ground);
		}

		usleep(10000); /* 10 ms cycle */
	}

	return 0;
}
