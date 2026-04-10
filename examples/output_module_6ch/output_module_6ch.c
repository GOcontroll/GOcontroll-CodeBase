/**************************************************************************************
 * \file   output_module_6ch.c
 * \brief  Drive a 6-channel output module on slot 2.
 *
 *         The 6-channel output module supports richer output functions than the
 *         10-channel variant: half-bridge, low/high-side duty cycle, boolean,
 *         peak-and-hold, and frequency output.  Per-channel current limits and
 *         peak-and-hold parameters are also configurable.
 *
 *         This example uses two different output modes simultaneously:
 *
 *           CH1 – CH4   OUTPUTFUNC_6CH_HIGHSIDEBOOL  (on/off, boolean high-side)
 *                        Channels cycle on one at a time, 500 ms each.
 *                        value = 1 → output active (high-side switch closed)
 *                        value = 0 → output inactive
 *
 *           CH5 – CH6   OUTPUTFUNC_6CH_HIGHSIDEDUTY  (PWM, high-side)
 *                        Duty cycle ramps 0 → 1000 → 0 (= 0–100 % in 0.1 % steps).
 *                        PWM frequency configured to 1 kHz for the CH5/CH6 pair.
 *
 *         The module provides per-channel current feedback and overall temperature.
 *         These are available after each GO_module_output_send_values() call in:
 *           outputModule.current[ch]   — measured channel current (mA)
 *           outputModule.temperature   — module PCB temperature (°C)
 *
 *         This example demonstrates:
 *           - Setting up a 6-channel output module (type, slot, channel config)
 *           - Mixing boolean and duty-cycle output functions in one module
 *           - Setting per-channel current limits
 *           - Configuring PWM frequency for a channel pair
 *           - Reading current and temperature feedback
 *           - Clean shutdown that turns all outputs off on exit
 *
 *         Wiring note:
 *           High-side outputs switch the positive supply to the load.
 *           Connect the load between the output pin and GND.
 *           Do not exceed the per-channel current rating of the module.
 *
 *         Build:
 *           make output_module_6ch
 **************************************************************************************/

#include <stdint.h>
#include <unistd.h>

#include "GO_board.h"
#include "GO_communication_modules.h"
#include "GO_module_output.h"
#include "print.h"

/* Module configuration */
#define OUTPUT_SLOT         MODULESLOT2     /* Physical slot 2 */

/* Timing */
#define CYCLE_US            10000           /* 10 ms main loop */
#define BOOL_CHANNEL_CYCLES 50             /* 50 × 10 ms = 500 ms per channel */
#define PWM_RAMP_CYCLES     1              /* ramp step every cycle */

/* Current limit applied to all channels (mA).
 * Keep within the module's rated maximum per channel. */
#define CHANNEL_CURRENT_MAX 2000

/* Duty-cycle limits (0–1000 = 0.0–100.0 %) */
#define DUTY_MAX            1000
#define DUTY_STEP           5    /* increment/decrement per cycle (= 0.5 % per 10 ms) */

/* -------------------------------------------------------------------------
 * Module instance
 * ------------------------------------------------------------------------- */
static _outputModule outputModule;

/* -------------------------------------------------------------------------
 * Shutdown callback — turn all outputs off before exiting.
 * ------------------------------------------------------------------------- */
static void app_terminate(void)
{
	info("Shutting down — disabling all channels (floating)\n");
	for (uint8_t ch = OUTPUTCHANNEL1; ch <= OUTPUTCHANNEL6; ch++) {
		GO_module_output_6ch_configure_channel(&outputModule, ch,
		                                       OUTPUTFUNC_DISABLED,
		                                       0, 0, 0);
	}
	GO_module_output_configuration(&outputModule);
}

/* -------------------------------------------------------------------------
 * main
 * ------------------------------------------------------------------------- */
int main(void)
{
	info("=== 6-channel output module example ===\n");
	info("Slot 2 | CH1-4: high-side boolean (cycling) | CH5-6: high-side PWM (ramp)\n");
	info("Press Ctrl+C to stop.\n\n");

	/* Detect the hardware variant (required before any module function). */
	GO_board_get_hardware_version();

	/* --- Module setup ---------------------------------------------------- */

	/* Set module type, initialise SPI, then assign slot.
	 * IMPORTANT: GO_communication_modules_initialize() must be called before
	 * GO_module_output_set_module_slot(). The initialize call populates
	 * hardwareConfig.moduleOccupancy; set_module_slot reads this data to
	 * verify that the correct module type is physically present in the slot.
	 * Reversing the order produces a "contested slot" error at runtime. */
	GO_module_output_set_module_type(&outputModule, OUTPUTMODULE6CHANNEL);
	GO_communication_modules_initialize(OUTPUT_SLOT);
	GO_module_output_set_module_slot(&outputModule, OUTPUT_SLOT);

	/* Configure CH1–CH4 as high-side boolean outputs.
	 *
	 * GO_module_output_6ch_configure_channel() parameters:
	 *   channel      — OUTPUTCHANNEL1 … OUTPUTCHANNEL6
	 *   func         — OUTPUTFUNC_6CH_*
	 *   currentMax   — continuous current limit, 0–4000 mA
	 *   peak_current — peak duty cycle for peak-and-hold (0 = unused)
	 *   peak_time    — peak phase duration for peak-and-hold (0 = unused)
	 *
	 * Available 6-channel functions (use OUTPUTFUNC_6CH_* macros):
	 *   HALFBRIDGE     — half-bridge (H-bridge leg), duty-cycle controlled
	 *   LOWSIDEDUTY    — low-side PWM
	 *   HIGHSIDEDUTY   — high-side PWM  (value 0–1000 = 0.0–100.0 %)
	 *   LOWSIDEBOOL    — low-side on/off (value 0 or 1)
	 *   HIGHSIDEBOOL   — high-side on/off (value 0 or 1)
	 *   PEAKANDHOLD    — inrush current control via half-bridge
	 *   FREQUENCYOUT   — frequency output (0–500 Hz) */
	for (uint8_t ch = OUTPUTCHANNEL1; ch <= OUTPUTCHANNEL4; ch++) {
		GO_module_output_6ch_configure_channel(&outputModule, ch,
		                                       OUTPUTFUNC_6CH_HIGHSIDEBOOL,
		                                       CHANNEL_CURRENT_MAX,
		                                       0,   /* peak_current — not used */
		                                       0);  /* peak_time    — not used */
	}

	/* Configure CH5–CH6 as high-side PWM outputs. */
	for (uint8_t ch = OUTPUTCHANNEL5; ch <= OUTPUTCHANNEL6; ch++) {
		GO_module_output_6ch_configure_channel(&outputModule, ch,
		                                       OUTPUTFUNC_6CH_HIGHSIDEDUTY,
		                                       CHANNEL_CURRENT_MAX,
		                                       0,
		                                       0);
	}

	/* Set the PWM frequency for the CH5/CH6 channel pair to 1 kHz.
	 * Channels are grouped in pairs: CH1&2 = OUTPUTFREQCHANNEL1AND2, etc.
	 * Available 6-channel frequencies: 100 Hz, 200 Hz, 500 Hz, 1 kHz,
	 *                                  2 kHz, 5 kHz, 10 kHz */
	GO_module_output_configure_frequency(&outputModule,
	                                     OUTPUTFREQCHANNEL5AND6,
	                                     OUTPUTFREQ_6CH_1KHZ);

	/* Send the full configuration to the module over SPI.
	 * Must be called once after all channels are configured. */
	GO_module_output_configuration(&outputModule);

	info("Module configured — starting output sequence\n\n");

	/* Register the shutdown callback for SIGTERM and SIGINT. */
	GO_board_exit_program(app_terminate);

	/* --- Application loop (10 ms cycle) ---------------------------------- */
	uint8_t  active_ch  = OUTPUTCHANNEL1;  /* boolean channel currently on   */
	int      bool_cycle = 0;
	uint16_t duty       = 0;               /* current duty cycle for CH5/CH6 */
	int      duty_dir   = 1;               /* +1 ramp up, -1 ramp down        */
	int      print_cyc  = 0;

	while (1) {

		/* --- CH1–CH4: sequential boolean cycling --- */
		if (++bool_cycle >= BOOL_CHANNEL_CYCLES) {
			bool_cycle = 0;

			outputModule.value[active_ch] = 0;

			active_ch = (active_ch >= OUTPUTCHANNEL4)
			          ? OUTPUTCHANNEL1
			          : active_ch + 1;

			outputModule.value[active_ch] = 1;
		}

		/* --- CH5–CH6: duty-cycle ramp --- */
		duty = (uint16_t)((int)duty + duty_dir * DUTY_STEP);
		if (duty >= DUTY_MAX) { duty = DUTY_MAX; duty_dir = -1; }
		if (duty == 0)        { duty_dir = 1; }

		outputModule.value[OUTPUTCHANNEL5] = duty;
		outputModule.value[OUTPUTCHANNEL6] = duty;

		/* Send current output values and receive feedback. */
		GO_module_output_send_values(&outputModule);

		/* Print status once per second (100 cycles × 10 ms). */
		if (++print_cyc >= 100) {
			print_cyc = 0;
			info("CH%d ON | duty %4u/1000 | curr CH1:%4dmA CH2:%4dmA CH3:%4dmA CH4:%4dmA CH5:%4dmA CH6:%4dmA | temp %d°C\n",
			     active_ch + 1,
			     duty,
			     outputModule.current[OUTPUTCHANNEL1],
			     outputModule.current[OUTPUTCHANNEL2],
			     outputModule.current[OUTPUTCHANNEL3],
			     outputModule.current[OUTPUTCHANNEL4],
			     outputModule.current[OUTPUTCHANNEL5],
			     outputModule.current[OUTPUTCHANNEL6],
			     outputModule.temperature);
		}

		usleep(CYCLE_US);
	}

	return 0;
}
