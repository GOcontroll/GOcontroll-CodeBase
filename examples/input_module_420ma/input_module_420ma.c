/**************************************************************************************
 * \file   input_module_420ma.c
 * \brief  Read all 10 channels of a 4-20 mA input module on slot 1.
 *
 *         The 4-20 mA standard maps the process variable to a current range:
 *           4 mA  = minimum value (0 %)
 *           20 mA = maximum value (100 %)
 *
 *         Raw values returned by the module are in microamps (µA).
 *         This example prints both the raw value and the percentage of the
 *         4-20 mA range for each channel.
 *
 *         This example demonstrates:
 *           - Setting up a 4-20 mA input module (slot assignment, channel enable)
 *           - Sending the configuration to the module hardware
 *           - Reading channel values in the application loop
 *           - Converting raw µA values to a 4-20 mA percentage
 *           - Printing all channels at a reduced rate (once per second)
 *           - Clean shutdown on SIGTERM / SIGINT (Ctrl+C)
 **************************************************************************************/

#include <stdint.h>
#include <unistd.h>

#include "GO_board.h"
#include "GO_communication_modules.h"
#include "GO_module_input_420ma.h"
#include "print.h"

/* Number of channels on this module */
#define NUM_CHANNELS 10

/* 4-20 mA range in µA */
#define MA4_UA  4000u
#define MA20_UA 20000u

/* Print interval: 100 cycles × 10 ms = 1 s */
#define PRINT_INTERVAL_CYCLES 100

/* -------------------------------------------------------------------------
 * Module instance — holds configuration and live measurement values.
 * ------------------------------------------------------------------------- */
static _inputModule420ma inputModule420ma;

/* -------------------------------------------------------------------------
 * Convert a raw µA reading to a 4-20 mA percentage (0-100 %).
 * Values below 4 mA are clamped to 0, values above 20 mA to 100.
 * ------------------------------------------------------------------------- */
static int ma_to_percent(uint16_t ua)
{
	if (ua <= MA4_UA)  return 0;
	if (ua >= MA20_UA) return 100;
	return (int)((ua - MA4_UA) * 100u / (MA20_UA - MA4_UA));
}

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
	info("=== 4-20 mA input module example ===\n");
	info("Slot 1 | 10 channels | Values printed once per second.\n");
	info("Press Ctrl+C to stop.\n\n");

	/* Detect the hardware variant (required before any module function). */
	GO_board_get_hardware_version();

	/* --- Module setup ---------------------------------------------------- */

	/* Assign the module to its physical slot.
	 * The 4-20 mA module does not have a set_module_type function —
	 * the slot is written directly to the struct. */
	inputModule420ma.moduleSlot = MODULESLOT1;

	/* Initialise SPI communication for this slot. */
	GO_communication_modules_initialize(MODULESLOT1);

	/* Enable all 10 channels by setting configuration to 1.
	 * A value of 0 disables the channel. */
	for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
		inputModule420ma.configuration[ch] = 1;
	}

	/* Send the full configuration to the module over SPI.
	 * Must be called once before the first GO_module_input_420ma_receive_values()
	 * call. */
	GO_module_input_420ma_configuration(&inputModule420ma);

	info("Module configured and communication initialised\n\n");

	/* Register the shutdown callback for SIGTERM and SIGINT. */
	GO_board_exit_program(app_terminate);

	/* --- Application loop (10 ms cycle) ---------------------------------- */
	int cycle = 0;

	while (1) {
		/* Fetch the latest values from the module via SPI.
		 * Results are stored in inputModule420ma.value[0..9] in µA. */
		GO_module_input_420ma_receive_values(&inputModule420ma);

		if (++cycle >= PRINT_INTERVAL_CYCLES) {
			cycle = 0;

			info("  CH01      CH02      CH03      CH04      CH05"
			     "      CH06      CH07      CH08      CH09      CH10\n");

			for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
				uint16_t ua  = inputModule420ma.value[ch];
				int      pct = ma_to_percent(ua);
				info("%5uµA(%3d%%)  ", ua, pct);
			}
			info("\n");
		}

		usleep(10000); /* 10 ms cycle */
	}

	return 0;
}
