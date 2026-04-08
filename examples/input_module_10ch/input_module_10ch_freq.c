/**************************************************************************************
 * \file   input_module_10ch_freq.c
 * \brief  Read a frequency signal on CH1 of a 10-channel input module on slot 1.
 *
 *         CH1 is configured as a frequency input (INPUTFUNC_FREQUENCY).
 *         The module measures the frequency of the signal on that channel and
 *         returns the result in Hz via inputModule.value[INPUTCHANNEL1].
 *
 *         CH2–CH10 remain configured as analog mV inputs (INPUTFUNC_MVANALOG).
 *
 *         Channel configuration:
 *           - CH1  : INPUTFUNC_FREQUENCY   — frequency measurement (Hz)
 *           - CH2–10 : INPUTFUNC_MVANALOG  — analog voltage (mV)
 *           - Pull-up   : INPUTPULLUP_NULL      (no pull-up; signal is externally driven)
 *           - Pull-down : INPUTPULLDOWN_NULL     (no pull-down)
 *
 *         Sensor supply:
 *           - Supply 1 : INPUTSENSSUPPLYON  (5 V output)
 *           - Supply 2 : INPUTSENSSUPPLYON  (5 V output)
 *
 *         This example demonstrates:
 *           - Mixing frequency and analog functions on a 10-channel input module
 *           - Reading the frequency value from inputModule.value[channel] in Hz
 *           - Clean shutdown on SIGTERM / SIGINT (Ctrl+C)
 **************************************************************************************/

#include <stdint.h>
#include <unistd.h>

#include "GO_board.h"
#include "GO_communication_modules.h"
#include "GO_module_input.h"
#include "print.h"

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
	info("=== 10-channel input module frequency example ===\n");
	info("Slot 1 | CH1: frequency (Hz) | CH2-10: analog mV\n");
	info("CH1 frequency printed once per second. Press Ctrl+C to stop.\n\n");

	/* Detect the hardware variant (required before any module function). */
	GO_board_get_hardware_version();

	/* --- Module setup ---------------------------------------------------- */

	GO_module_input_set_module_type(&inputModule, INPUTMODULE10CHANNEL);
	GO_module_input_set_module_slot(&inputModule, MODULESLOT1);

	/* Initialise the SPI communication for this slot. */
	GO_communication_modules_initialize(MODULESLOT1);

	/* Configure CH1 as a frequency input.
	 * The module returns the measured frequency in Hz in inputModule.value[INPUTCHANNEL1].
	 * No pull-up or pull-down is used: the frequency signal is assumed to be externally
	 * driven (e.g. a square wave from a sensor or signal generator). */
	GO_module_input_10ch_configure_channel(&inputModule, INPUTCHANNEL1,
	                                       INPUTFUNC_FREQUENCY,
	                                       INPUTPULLUP_NULL,
	                                       INPUTPULLDOWN_NULL);

	/* Configure CH2–CH10 as analog mV inputs. */
	for (uint8_t ch = INPUTCHANNEL2; ch <= INPUTCHANNEL10; ch++) {
		GO_module_input_10ch_configure_channel(&inputModule, ch,
		                                       INPUTFUNC_MVANALOG,
		                                       INPUTPULLUP10CH_10K,
		                                       INPUTPULLDOWN10CH_3_3K);
	}

	/* Enable both 5 V sensor supply outputs.
	 * Must be called before GO_module_input_configuration(). */
	GO_module_input_10ch_configure_supply(&inputModule,
	                                      INPUTSENSSUPPLYON,   /* supply 1 — active */
	                                      INPUTSENSSUPPLYON);  /* supply 2 — active */

	/* Send the full configuration to the module over SPI. */
	GO_module_input_configuration(&inputModule);

	info("Module configured and communication initialised\n\n");

	/* Register the shutdown callback for SIGTERM and SIGINT. */
	GO_board_exit_program(app_terminate);

	/* --- Application loop (10 ms cycle) ---------------------------------- */
	int cycle = 0;

	while (1) {
		GO_module_input_receive_values(&inputModule);

		if (++cycle >= PRINT_INTERVAL_CYCLES) {
			cycle = 0;
			info("CH1 frequency: %ld Hz\n", (long)inputModule.value[INPUTCHANNEL1]);
		}

		usleep(10000); /* 10 ms */
	}

	return 0;
}
