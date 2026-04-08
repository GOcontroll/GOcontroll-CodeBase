/**************************************************************************************
 * \file   read_supply_voltages.c
 * \brief  Read and print the K30 and K15-A/B/C supply voltages every second.
 *
 *         The controller measures four supply lines:
 *           K30   — permanent battery voltage (always present when powered)
 *           K15-A — ignition-switched supply, channel A
 *           K15-B — ignition-switched supply, channel B
 *           K15-C — ignition-switched supply, channel C
 *
 *         Voltages are sampled by a background ADC thread and returned in
 *         millivolts (mV). Divide by 1000 for volts.
 *
 *         This example demonstrates:
 *           - Starting and stopping the ADC background thread
 *           - Reading supply voltages with GO_board_controller_power_voltage()
 *           - Printing formatted output at a lower rate than the loop cycle
 *           - Clean shutdown on SIGTERM / SIGINT (Ctrl+C)
 **************************************************************************************/

#include <stdint.h>
#include <unistd.h>

#include "GO_board.h"
#include "print.h"

/* Print voltages every 100 cycles: 100 × 10 ms = 1 s */
#define PRINT_INTERVAL_CYCLES 100

/* -------------------------------------------------------------------------
 * Shutdown callback — registered via GO_board_exit_program().
 * Stops the ADC thread and deactivates the controller-active output.
 * ------------------------------------------------------------------------- */
static void app_terminate(void)
{
	info("Shutting down — stopping ADC thread\n");
	GO_board_controller_power_stop_adc_thread();
	GO_board_controller_power_controller_active(0);
}

/* -------------------------------------------------------------------------
 * main
 * ------------------------------------------------------------------------- */
int main(void)
{
	info("=== Supply voltage monitor ===\n");
	info("Reading K30, K15-A, K15-B, K15-C every second. Press Ctrl+C to stop.\n");

	/* Detect the hardware variant (required before any board function). */
	GO_board_get_hardware_version();

	/* Start the background ADC thread with a 100 ms sample interval.
	 * The thread continuously updates the internal voltage cache used by
	 * GO_board_controller_power_voltage(). It must be running before
	 * any voltage is read. */
	GO_board_controller_power_start_adc_thread(100);
	info("ADC thread started (100 ms sample interval)\n\n");

	/* Register the shutdown callback for SIGTERM and SIGINT. */
	GO_board_exit_program(app_terminate);

	/* --- Application loop (10 ms cycle) ---------------------------------- */
	int cycle = 0;

	while (1) {
		if (++cycle >= PRINT_INTERVAL_CYCLES) {
			cycle = 0;

			/* Read each supply voltage in mV.
			 * Supply indices: 1=K30, 2=K15-A, 3=K15-B, 4=K15-C
			 * Returns -1 on invalid index; voltage is 0 if not yet sampled. */
			uint16_t k30, k15a, k15b, k15c;
			GO_board_controller_power_voltage(1, &k30);
			GO_board_controller_power_voltage(2, &k15a);
			GO_board_controller_power_voltage(3, &k15b);
			GO_board_controller_power_voltage(4, &k15c);

			info("K30: %u mV  |  K15-A: %u mV  |  K15-B: %u mV  |  K15-C: %u mV\n",
				 k30, k15a, k15b, k15c);
		}

		usleep(10000); /* 10 ms */
	}

	return 0;
}
