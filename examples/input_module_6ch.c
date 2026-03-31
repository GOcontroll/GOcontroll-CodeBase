/**************************************************************************************
 * \file   input_module_6ch.c
 * \brief  Read all 6 channels of a 6-channel input module on slot 3.
 *
 *         Each channel is configured as:
 *           - Function      : analog mV resolution (INPUTFUNC_MVANALOG)
 *           - Voltage range : 0 – 5 V (INPUTVOLTAGERANGE_5V)
 *           - Pull-up       : none (INPUTPULLUP_NULL)
 *           - Pull-down     : none (INPUTPULLDOWN_NULL)
 *           - Filter        : 100 samples
 *
 *         All three 5 V sensor supply outputs are enabled so that connected
 *         sensors are powered before the configuration is sent to the module.
 *
 *         Initialisation order (must be respected):
 *           1. GO_board_get_hardware_version()
 *           2. GO_module_input_set_module_type()
 *           3. GO_communication_modules_initialize()    ← populates moduleOccupancy
 *           4. GO_module_input_set_module_slot()        ← verifies module type in slot
 *           5. GO_module_input_6ch_configure_supply()   ← supply before channels
 *           6. GO_module_input_6ch_configure_channel()  ← once per channel
 *           7. GO_module_input_configuration()          ← send config to hardware
 *           8. GO_board_exit_program()
 *
 *         The measured value of each channel is available in millivolts via
 *         inputModule.value[channel] after calling GO_module_input_receive_values().
 *
 *         This example demonstrates:
 *           - Setting up a 6-channel input module (type, slot, channel config)
 *           - Enabling all three 5 V sensor supply outputs
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
#define NUM_CHANNELS 6

/* Number of analog filter samples per measurement (0 – 1000) */
#define ANALOG_FILTER_SAMPLES 100

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
	info("=== 6-channel input module example ===\n");
	info("Slot 3 | Function: analog mV | Range: 0-5 V | Sensor supply: ON\n");
	info("All 6 channels printed once per second. Press Ctrl+C to stop.\n\n");

	/* 1. Detect the hardware variant (required before any module function). */
	GO_board_get_hardware_version();

	/* --- Module setup ---------------------------------------------------- */

	/* 2. Assign module type. */
	GO_module_input_set_module_type(&inputModule, INPUTMODULE6CHANNEL);

	/* 3. Initialise the SPI communication for this slot.
	 *    Must be called before set_module_slot: it populates
	 *    hardwareConfig.moduleOccupancy, which set_module_slot uses to
	 *    verify that the correct module type is physically present. */
	GO_communication_modules_initialize(MODULESLOT3);

	/* 4. Assign slot — verified against the module occupancy data. */
	GO_module_input_set_module_slot(&inputModule, MODULESLOT3);

	/* 5. Enable all three 5 V sensor supply outputs.
	 *    Must be configured before the channel configuration is sent so that
	 *    the supply state is included in the first configuration frame. */
	GO_module_input_6ch_configure_supply(&inputModule,
										 INPUTSENSSUPPLYON,   /* supply 1 */
										 INPUTSENSSUPPLYON,   /* supply 2 */
										 INPUTSENSSUPPLYON);  /* supply 3 */

	/* 6. Configure all 6 channels identically:
	 *   INPUTFUNC_MVANALOG       — returns the measured voltage in millivolts
	 *   INPUTVOLTAGERANGE_5V     — measurement range 0 – 5 V
	 *   INPUTPULLUP_NULL         — no pull-up resistor (sensor drives the line)
	 *   INPUTPULLDOWN_NULL       — no pull-down resistor
	 *   0                        — pulses per rotation (unused for analog)
	 *   ANALOG_FILTER_SAMPLES    — number of samples averaged per measurement
	 *
	 * Use the INPUTPULLUP6CH_* / INPUTPULLDOWN6CH_* macros when a resistor
	 * is required; for sensor-supply-driven 0–5 V signals none is needed. */
	for (uint8_t ch = INPUTCHANNEL1; ch <= INPUTCHANNEL6; ch++) {
		GO_module_input_6ch_configure_channel(&inputModule, ch,
											  INPUTFUNC_MVANALOG,
											  INPUTVOLTAGERANGE_5V,
											  INPUTPULLUP_NULL,
											  INPUTPULLDOWN_NULL,
											  0,
											  ANALOG_FILTER_SAMPLES);
	}

	/* 7. Send the full configuration to the module over SPI.
	 *    Must be called once after all channels and the supply are configured
	 *    and before the first GO_module_input_receive_values() call. */
	GO_module_input_configuration(&inputModule);

	info("Module configured and communication initialised\n\n");

	/* 8. Register the shutdown callback for SIGTERM and SIGINT. */
	GO_board_exit_program(app_terminate);

	/* --- Application loop (10 ms cycle) ---------------------------------- */
	int cycle = 0;

	while (1) {
		/* Fetch the latest values from the module via SPI.
		 * Results are stored in inputModule.value[0..5] in millivolts. */
		GO_module_input_receive_values(&inputModule);

		if (++cycle >= PRINT_INTERVAL_CYCLES) {
			cycle = 0;

			info("  CH01      CH02      CH03      CH04      CH05      CH06\n");
			for (uint8_t ch = INPUTCHANNEL1; ch <= INPUTCHANNEL6; ch++) {
				info("%6ldmV  ", (long)inputModule.value[ch]);
			}
			info("\n");
		}

		usleep(10000); /* 10 ms */
	}

	return 0;
}
