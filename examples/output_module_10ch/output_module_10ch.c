/**************************************************************************************
 * \file   output_module_10ch.c
 * \brief  Drive a 10-channel output module on slot 2.
 *
 *         All 10 channels are configured as high-side boolean outputs
 *         (OUTPUTFUNC_10CH_HIGHSIDEBOOL).  The example cycles through the
 *         channels one by one, switching each on for 500 ms before moving
 *         to the next.
 *
 *         To switch a channel on:   outputModule.value[ch] = 1
 *         To switch a channel off:  outputModule.value[ch] = 0
 *         Then call:                GO_module_output_send_values(&outputModule)
 *
 *         This example demonstrates:
 *           - Setting up a 10-channel output module (type, slot, channel config)
 *           - Sending the configuration to the module hardware
 *           - Turning individual output channels on and off
 *           - Clean shutdown that turns all outputs off on exit
 *
 *         Wiring note:
 *           Each high-side output switches the positive supply to the load.
 *           Make sure the load is connected between the output pin and ground
 *           and does not exceed the module's current rating per channel.
 **************************************************************************************/

#include <stdint.h>
#include <unistd.h>

#include "GO_board.h"
#include "GO_communication_modules.h"
#include "GO_module_output.h"
#include "print.h"

/* Module configuration */
#define OUTPUT_SLOT         MODULESLOT2          /* Physical slot (0-based: slot 2 = index 1) */
#define NUM_CHANNELS        10

/* Cycle timing: 50 cycles × 10 ms = 500 ms per channel */
#define CHANNEL_ON_CYCLES   50

/* -------------------------------------------------------------------------
 * Module instance — holds configuration, output values and feedback.
 * ------------------------------------------------------------------------- */
static _outputModule outputModule;

/* -------------------------------------------------------------------------
 * Shutdown callback — turn all outputs off before exiting.
 * ------------------------------------------------------------------------- */
static void app_terminate(void)
{
	info("Shutting down — turning all outputs off\n");
	for (uint8_t ch = OUTPUTCHANNEL1; ch <= OUTPUTCHANNEL10; ch++) {
		outputModule.value[ch] = 0;
	}
	GO_module_output_send_values(&outputModule);
}

/* -------------------------------------------------------------------------
 * main
 * ------------------------------------------------------------------------- */
int main(void)
{
	info("=== 10-channel output module example ===\n");
	info("Slot 2 | Function: high-side boolean | Cycling through all 10 channels\n");
	info("Each channel is switched on for 500 ms in sequence. Press Ctrl+C to stop.\n\n");

	/* Detect the hardware variant (required before any module function). */
	GO_board_get_hardware_version();

	/* --- Module setup ---------------------------------------------------- */

	/* Set module type and slot.
	 * OUTPUTMODULE10CHANNEL selects the 10-channel variant.
	 * MODULESLOT2 = index 1 (0-based), matching the second physical slot. */
	GO_module_output_set_module_type(&outputModule, OUTPUTMODULE10CHANNEL);
	GO_module_output_set_module_slot(&outputModule, OUTPUT_SLOT);

	/* Initialise SPI communication for this slot. */
	GO_communication_modules_initialize(OUTPUT_SLOT);

	/* Configure all 10 channels as high-side boolean outputs.
	 * OUTPUTFUNC_10CH_HIGHSIDEBOOL: value 1 = output active (high-side switch closed),
	 *                               value 0 = output inactive.
	 * Use OUTPUTFUNC_10CH_HIGHSIDEDUTY for PWM duty-cycle control instead. */
	for (uint8_t ch = OUTPUTCHANNEL1; ch <= OUTPUTCHANNEL10; ch++) {
		GO_module_output_10ch_configure_channel(&outputModule, ch,
		                                        OUTPUTFUNC_10CH_HIGHSIDEBOOL);
	}

	/* Send the full configuration to the module over SPI.
	 * Must be called once after all channels are configured. */
	GO_module_output_configuration(&outputModule);

	info("Module configured — starting output sequence\n\n");

	/* Register the shutdown callback for SIGTERM and SIGINT. */
	GO_board_exit_program(app_terminate);

	/* --- Application loop (10 ms cycle) ---------------------------------- */
	uint8_t active_ch = OUTPUTCHANNEL1; /* Channel currently switched on     */
	int     cycle     = 0;

	while (1) {
		if (++cycle >= CHANNEL_ON_CYCLES) {
			cycle = 0;

			/* Turn off the current channel */
			outputModule.value[active_ch] = 0;

			/* Advance to the next channel (wrap around after CH10) */
			active_ch = (active_ch >= OUTPUTCHANNEL10)
			          ? OUTPUTCHANNEL1
			          : active_ch + 1;

			/* Turn on the new channel */
			outputModule.value[active_ch] = 1;

			info("CH%02d ON  (all others off)\n", active_ch + 1);
		}

		/* Send current output values to the module.
		 * Must be called every cycle to keep the SPI watchdog alive. */
		GO_module_output_send_values(&outputModule);

		usleep(10000); /* 10 ms cycle */
	}

	return 0;
}
