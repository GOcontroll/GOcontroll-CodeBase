/**************************************************************************************
 * \file   input_module_10ch.c
 * \brief  Read all 10 channels of a 10-channel input module on slot 1.
 *
 *         Each channel is configured as:
 *           - Function  : analog mV resolution (INPUTFUNC_MVANALOG)
 *           - Pull-up   : 10 kΩ (INPUTPULLUP10CH_10K)
 *           - Pull-down : 3.3 kΩ (INPUTPULLDOWN10CH_3_3K)
 *
 *         Sensor supply:
 *           - Supply 1 : INPUTSENSSUPPLYON  (5 V output)
 *           - Supply 2 : INPUTSENSSUPPLYON  (5 V output)
 *
 *         The measured value of each channel is available in millivolts via
 *         inputModule.value[channel] after calling GO_module_input_receive_values().
 *
 *         This example demonstrates:
 *           - Setting up a 10-channel input module (type, slot, channel config)
 *           - Enabling the 5 V sensor supply outputs
 *           - Sending the configuration to the module hardware
 *           - Reading channel values in the application loop
 *           - Printing all channels at a reduced rate (once per second)
 *           - Clean shutdown on SIGTERM / SIGINT (Ctrl+C)
 **************************************************************************************/

#include <stdint.h>
#include <unistd.h>

#include "GO_board.h"
#include "GO_communication_modules.h"
#include "GO_module_input.h"
#include "print.h"

/* Number of channels on this module type */
#define NUM_CHANNELS 10

/* Print interval: 100 cycles × 10 ms = 1 s */
#define PRINT_INTERVAL_CYCLES 100

/* -------------------------------------------------------------------------
 * Module instance — holds configuration and live measurement values.
 * ------------------------------------------------------------------------- */
static _inputModule inputModule;

/* -------------------------------------------------------------------------
 * Shutdown callback — registered via GO_board_exit_program().
 * ------------------------------------------------------------------------- */
static void app_terminate(void)
{
	info("Shutting down\n");
}

/* -------------------------------------------------------------------------
 * main
 * ------------------------------------------------------------------------- */
int main(void)
{
	info("=== 10-channel input module example ===\n");
	info("Slot 1 | Function: analog mV | Pull-up: 10k | Pull-down: 3.3k\n");
	info("All 10 channels printed once per second. Press Ctrl+C to stop.\n\n");

	/* Detect the hardware variant (required before any module function). */
	GO_board_get_hardware_version();



	/* --- Module setup ---------------------------------------------------- */

	/* Assign module type and slot.
	 * The slot index matches the physical slot on the controller (MODULESLOT1 = slot 1). */
	GO_module_input_set_module_type(&inputModule, INPUTMODULE10CHANNEL);
	GO_module_input_set_module_slot(&inputModule, MODULESLOT1);

	/* Initialise the SPI communication for this slot. */
	GO_communication_modules_initialize(MODULESLOT1);


	/* Configure all 10 channels identically:
	 *   INPUTFUNC_MVANALOG      — returns the measured voltage in millivolts
	 *   INPUTPULLUP10CH_10K     — enable 10 kΩ pull-up resistor (10-channel module)
	 *   INPUTPULLDOWN10CH_3_3K  — enable 3.3 kΩ pull-down resistor (10-channel module)
	 *
	 * Use the INPUTPULLUP10CH_* / INPUTPULLDOWN10CH_* macros for this module type.
	 * The INPUTPULLUP6CH_* / INPUTPULLDOWN6CH_* macros are for the 6-channel module only. */
	for (uint8_t ch = INPUTCHANNEL1; ch <= INPUTCHANNEL10; ch++) {
		GO_module_input_10ch_configure_channel(&inputModule, ch,
											   INPUTFUNC_MVANALOG,
											   INPUTPULLUP10CH_10K,
											   INPUTPULLDOWN10CH_3_3K);
	}

	/* Enable both 5 V sensor supply outputs.
	 * Must be called before GO_module_input_configuration() so the supply state
	 * is included in the first configuration frame sent to the module.
	 * Supply 2 is only active on modules with firmware >= VERSIONSECONDSUPPLY_10CHANNELIN. */
	GO_module_input_10ch_configure_supply(&inputModule,
	                                      INPUTSENSSUPPLYON,   /* supply 1 — active */
	                                      INPUTSENSSUPPLYON);  /* supply 2 — active */

	/* Send the full configuration to the module over SPI.
	 * Must be called once after all channels and supplies are configured and before
	 * the first GO_module_input_receive_values() call. */
	GO_module_input_configuration(&inputModule);


	info("Module configured and communication initialised\n\n");

	/* Register the shutdown callback for SIGTERM and SIGINT. */
	GO_board_exit_program(app_terminate);

	/* --- Application loop (10 ms cycle) ---------------------------------- */
	int cycle = 0;

	while (1) {
		/* Fetch the latest values from the module via SPI.
		 * Results are stored in inputModule.value[0..9] in millivolts. */
		GO_module_input_receive_values(&inputModule);

		if (++cycle >= PRINT_INTERVAL_CYCLES) {
			cycle = 0;

			/* Print a fixed-width header and value row so columns line up.
			 * Each column is 10 characters wide: "  CHxx   " / " 12345mV ". */
			info("  CH01      CH02      CH03      CH04      CH05      CH06      CH07      CH08      CH09      CH10\n");
			for (uint8_t ch = INPUTCHANNEL1; ch <= INPUTCHANNEL10; ch++) {
				info("%6ldmV  ", (long)inputModule.value[ch]);
			}
			info("\n");
		}

		usleep(10000); /* 10 ms */
	}

	return 0;
}
