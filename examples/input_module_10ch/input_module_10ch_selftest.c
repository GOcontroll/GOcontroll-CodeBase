/**************************************************************************************
 * \file   input_module_10ch_selftest.c
 * \brief  Hardware self-test for the 10-channel input module on slot 1.
 *
 *         Each channel is tested in three configurations using the on-board
 *         pull resistors as a known load. The expected ADC counts are derived
 *         from the input circuit (12-bit ADC, 3.3 V reference):
 *
 *         Input circuit (per channel, simplified):
 *
 *           5V ── pull-up(10kΩ) ──┬── series(105kΩ) ──┬── ADC
 *                                 │                    │
 *                          pull-down(3.3kΩ)       divider(210kΩ)
 *                                 │                    │
 *                                GND                  GND
 *
 *         The pull-up and pull-down connect at the external PIN (left side).
 *         The voltage divider (series 105kΩ + lower 210kΩ) sits between the
 *         PIN and the ADC. The ADC measures the midpoint of the divider.
 *
 *         Test phases (no external signal connected):
 *
 *           Phase 1 — pull-up only:
 *             I = 5V / (10k + 105k + 210k) = 15.4 µA
 *             V_adc = 15.4µA × 210kΩ = 3.23V  →  ADC ≈ 4013
 *
 *           Phase 2 — pull-down only:
 *             PIN is pulled to GND (no opposing voltage source)
 *             V_adc = 0V × 210 / (105 + 210) = 0V  →  ADC ≈ 0
 *
 *           Phase 3 — both pull-up and pull-down active:
 *             Thevenin at PIN: V_th = 1.24V, R_th = 2.48kΩ
 *             V_adc = 1.24V × 210 / (2.48 + 105 + 210) = 0.82V  →  ADC ≈ 1020
 *
 *         A tolerance of ±300 counts is applied to account for 5% component
 *         spread and ADC offset.
 **************************************************************************************/

#include <stdint.h>
#include <unistd.h>

#include "GO_board.h"
#include "GO_communication_modules.h"
#include "GO_module_input.h"
#include "print.h"

/* ANSI colour codes — wrap text with these to get coloured console output */
#define COL_RED     "\033[31m"
#define COL_GREEN   "\033[32m"
#define COL_RESET   "\033[0m"

#define NUM_CHANNELS     10
#define SETTLE_CYCLES    50    /* 50 × 10 ms = 500 ms after reconfiguration  */
#define SAMPLE_CYCLES    10    /* average over 10 consecutive readings        */
#define TOLERANCE         75   /* ±counts, covers 5% component spread         */

/* Expected ADC values per test phase */
#define EXPECT_PULLUP      4013  /* pull-up only, lower divider sets ~3.23V  */
#define EXPECT_PULLDOWN       0  /* pull-down only, PIN at GND → V_adc = 0V  */
#define EXPECT_BOTH        1020  /* both active, Thevenin divider → ~0.82V   */

/* -------------------------------------------------------------------------
 * Module instance
 * ------------------------------------------------------------------------- */
static _inputModule inputModule;

/* -------------------------------------------------------------------------
 * configure_all_channels — apply one resistor combination to all channels
 * and push the configuration to the module hardware.
 * ------------------------------------------------------------------------- */
static void configure_all_channels(uint8_t pull_up, uint8_t pull_down)
{
	for (uint8_t ch = INPUTCHANNEL1; ch <= INPUTCHANNEL10; ch++) {
		GO_module_input_10ch_configure_channel(&inputModule, ch,
											   INPUTFUNC_12BITADC,
											   pull_up,
											   pull_down);
	}
	GO_module_input_configuration(&inputModule);
}

/* -------------------------------------------------------------------------
 * read_average — drain SETTLE_CYCLES to let the new config take effect,
 * then accumulate SAMPLE_CYCLES readings and return the per-channel average.
 * ------------------------------------------------------------------------- */
static void read_average(int32_t avg[NUM_CHANNELS])
{
	/* Settle: discard readings taken during/after reconfiguration */
	for (int i = 0; i < SETTLE_CYCLES; i++) {
		GO_module_input_receive_values(&inputModule);
		usleep(10000);
	}

	/* Accumulate */
	int64_t acc[NUM_CHANNELS] = {0};
	for (int s = 0; s < SAMPLE_CYCLES; s++) {
		GO_module_input_receive_values(&inputModule);
		for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
			acc[ch] += inputModule.value[ch];
		}
		usleep(10000);
	}

	for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
		avg[ch] = (int32_t)(acc[ch] / SAMPLE_CYCLES);
	}
}

/* -------------------------------------------------------------------------
 * run_phase — configure, measure, check and report one test phase.
 * Returns the number of channel failures in this phase.
 * ------------------------------------------------------------------------- */
static int run_phase(const char *name,
					  uint8_t pull_up, uint8_t pull_down,
					  int32_t expect, int32_t min, int32_t max)
{
	info("\n--- %s ---\n", name);
	info("Expected ADC: %ld  (range %ld – %ld)\n",
		 (long)expect, (long)min, (long)max);

	configure_all_channels(pull_up, pull_down);

	int32_t avg[NUM_CHANNELS];
	read_average(avg);

	/* Print header and values */
	info("  CH01      CH02      CH03      CH04      CH05      CH06      CH07      CH08      CH09      CH10\n");
	for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
		info("%6ld     ", (long)avg[ch]);
	}
	info("\n");

	/* Check each channel */
	int failures = 0;
	for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
		if (avg[ch] < min || avg[ch] > max) {
			info(COL_RED "  [FAIL] CH%02u: %ld  (out of range %ld – %ld)" COL_RESET "\n",
				 ch + 1, (long)avg[ch], (long)min, (long)max);
			failures++;
		}
	}

	if (failures == 0) {
		info(COL_GREEN "  All channels PASS" COL_RESET "\n");
	}

	return failures;
}

/* -------------------------------------------------------------------------
 * Shutdown callback
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
	info("=== 10-channel input module self-test ===\n");
	info("Slot 1 | 12-bit ADC | No external signals should be connected\n\n");

	GO_board_get_hardware_version();

	GO_module_input_set_module_type(&inputModule, INPUTMODULE10CHANNEL);
	GO_module_input_set_module_slot(&inputModule, MODULESLOT1);
	GO_communication_modules_initialize(MODULESLOT1);

	GO_board_exit_program(app_terminate);

	/* Run the three test phases */
	int total_failures = 0;

	/* Phase 1: pull-up only
	 * The pull-up (5V) drives current through the series and lower divider
	 * resistors to GND. V_adc = 5V × lower/(series+lower) ≈ 3.23V → ADC ≈ 4013 */
	total_failures += run_phase(
		"Phase 1: pull-up only (10 kΩ to 5V)",
		INPUTPULLUP10CH_10K, INPUTPULLDOWN_NULL,
		EXPECT_PULLUP,
		EXPECT_PULLUP - TOLERANCE, 4095);

	/* Phase 2: pull-down only
	 * The PIN is pulled to GND with no opposing voltage. V_adc = 0V → ADC ≈ 0 */
	total_failures += run_phase(
		"Phase 2: pull-down only (3.3 kΩ to GND)",
		INPUTPULLUP_NULL, INPUTPULLDOWN10CH_3_3K,
		EXPECT_PULLDOWN,
		0,
		EXPECT_PULLDOWN + TOLERANCE);

	/* Phase 3: both pull-up and pull-down active
	 * Thevenin equivalent at PIN: V_th = 1.24V, R_th = 2.48kΩ.
	 * V_adc = V_th × lower/(R_th+series+lower) ≈ 0.82V → ADC ≈ 1020 */
	total_failures += run_phase(
		"Phase 3: pull-up and pull-down active",
		INPUTPULLUP10CH_10K, INPUTPULLDOWN10CH_3_3K,
		EXPECT_BOTH,
		EXPECT_BOTH - TOLERANCE,
		EXPECT_BOTH + TOLERANCE);

	/* Final verdict */
	info("\n========================================\n");
	if (total_failures == 0) {
		info(COL_GREEN "Self-test PASSED — all channels OK" COL_RESET "\n");
	} else {
		info(COL_RED "Self-test FAILED — %d channel/phase failure(s)" COL_RESET "\n",
			 total_failures);
	}
	info("========================================\n");

	return (total_failures == 0) ? 0 : 1;
}
