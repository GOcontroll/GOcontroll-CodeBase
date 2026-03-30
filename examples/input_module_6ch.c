/**************************************************************************************
 * \file   input_module_6ch.c
 * \brief  Read all 6 channels of a 6-channel input module on slot 1.
 *
 *         The 6-channel input module has richer configuration than the 10-channel
 *         variant: per-channel voltage range, analogue filter depth, and three
 *         independent 5 V sensor supply outputs.
 *
 *         Channel configuration in this example:
 *           - Function      : INPUTFUNC_MVANALOG (raw voltage in millivolts)
 *           - Voltage range : INPUTVOLTAGERANGE_24V (0 – 24 V full scale)
 *           - Pull-up       : INPUTPULLUP6CH_10K   (10 kΩ to sensor supply)
 *           - Pull-down     : INPUTPULLDOWN6CH_3_3K (3.3 kΩ to GND)
 *           - Filter samples: 10 (ADC measurements are averaged over 10 samples)
 *           - Pulses/rot    : 0  (not used for analogue function)
 *
 *         Sensor supply:
 *           - Supply 1 : INPUTSENSSUPPLYON   (5 V output, powers sensors on CH1–2)
 *           - Supply 2 : INPUTSENSSUPPLYOFF
 *           - Supply 3 : INPUTSENSSUPPLYOFF
 *
 *         This example demonstrates:
 *           - Setting up a 6-channel input module (type, slot, channel config)
 *           - Configuring the per-channel voltage range and analogue filter
 *           - Enabling sensor supply outputs
 *           - Reading channel values in the application loop
 *           - Printing all channels at a reduced rate (once per second)
 *           - Clean shutdown on SIGTERM / SIGINT (Ctrl+C)
 *
 *         Build:
 *           make input_module_6ch
 **************************************************************************************/

#include <stdint.h>
#include <unistd.h>

#include "GO_board.h"
#include "GO_communication_modules.h"
#include "GO_module_input.h"
#include "print.h"

/* Number of channels on this module type */
#define NUM_CHANNELS 6

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
	info("Slot 1 | Function: analog mV | Range: 24 V | Pull-up: 10 kΩ | Filter: 10 samples\n");
	info("All 6 channels printed once per second. Press Ctrl+C to stop.\n\n");

	/* Detect the hardware variant (required before any module function). */
	GO_board_get_hardware_version();

	/* --- Module setup ---------------------------------------------------- */

	/* Assign module type and slot.
	 * INPUTMODULE6CHANNEL selects the 6-channel variant.
	 * The slot index matches the physical slot on the controller (MODULESLOT1 = slot 1). */
	GO_module_input_set_module_type(&inputModule, INPUTMODULE6CHANNEL);
	GO_module_input_set_module_slot(&inputModule, MODULESLOT1);

	/* Initialise the SPI communication for this slot. */
	GO_communication_modules_initialize(MODULESLOT1);

	/* Configure all 6 channels identically.
	 *
	 * GO_module_input_6ch_configure_channel() takes more parameters than the
	 * 10-channel equivalent:
	 *
	 *   func                  — measurement function (INPUTFUNC_*)
	 *   voltage_range         — full-scale range: INPUTVOLTAGERANGE_5V / 12V / 24V
	 *   pull_up               — pull-up resistor: INPUTPULLUP6CH_3_3K / 4_7K / 10K
	 *   pull_down             — pull-down resistor: INPUTPULLDOWN6CH_3_3K / 4_7K / 10K
	 *   pulses_per_rotation   — encoder PPR for INPUTFUNC_RPM; set 0 for analogue
	 *   analog_filter_samples — ADC averaging depth (0 = no filter, 1000 = max filter)
	 *
	 * Use the INPUTPULLUP6CH_* / INPUTPULLDOWN6CH_* macros for this module type.
	 * The INPUTPULLUP10CH_* / INPUTPULLDOWN10CH_* macros are for the 10-channel module. */
	for (uint8_t ch = INPUTCHANNEL1; ch <= INPUTCHANNEL6; ch++) {
		GO_module_input_6ch_configure_channel(&inputModule, ch,
		                                      INPUTFUNC_MVANALOG,
		                                      INPUTVOLTAGERANGE_24V,
		                                      INPUTPULLUP6CH_10K,
		                                      INPUTPULLDOWN6CH_3_3K,
		                                      0,   /* pulses_per_rotation — not used */
		                                      10); /* analog_filter_samples */
	}

	/* Configure the three independent sensor supply outputs.
	 * Each supply provides a 5 V output to power external sensors.
	 * Enable only the supplies that are wired to a sensor. */
	GO_module_input_6ch_configure_supply(&inputModule,
	                                     INPUTSENSSUPPLYON,   /* supply 1 — active */
	                                     INPUTSENSSUPPLYOFF,  /* supply 2 — inactive */
	                                     INPUTSENSSUPPLYOFF); /* supply 3 — inactive */

	/* Send the full configuration to the module over SPI.
	 * Must be called once after all channels and supplies are configured and
	 * before the first GO_module_input_receive_values() call. */
	GO_module_input_configuration(&inputModule);

	info("Module configured and communication initialised\n\n");

	/* Register the shutdown callback for SIGTERM and SIGINT. */
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
