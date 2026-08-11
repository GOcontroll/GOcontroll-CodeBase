/**************************************************************************************
 * \file   output_module_6ch_s1.c
 * \brief  Drive a 6-channel output module from a Moduline S1 (STM32H5 / FreeRTOS).
 *
 *         This is the CANONICAL S1 bring-up procedure for an output module. It exists
 *         because the naive sequence — detect, configure, drive — silently produces a
 *         module that communicates perfectly (valid frames, plausible temperature and
 *         supply, no error bits) while **every output stays dead**. Each step below is
 *         annotated with the failure it prevents; all of them were found the hard way on
 *         real hardware. Read `output_module_6ch_s1.md` next to this file for the
 *         diagnosis flow, and `../../AGENTS.md` rules 8, 13, 14, 15 and 16 for the rules
 *         these steps implement.
 *
 *         The Linux reference this mirrors is the generated Simulink application
 *         (`GOcontroll_Linux_3_0.c`, block "Start for M-S-Function: Output Module
 *         (6 channel)"): reset+detect once per module, configure once, then only
 *         `send_values()` on a 10 ms tick. Nothing else. Where this example deviates from
 *         "obvious" code, it is because the Linux version does it this way and works.
 *
 *         Wiring for this example:
 *           CH1 + CH2   OUTPUTFUNC_6CH_HALFBRIDGE  — one half-bridge leg each.
 *                       A DC motor / linear actuator goes BETWEEN the CH1 and CH2 output
 *                       pins. Driving CH1 at duty D with CH2 at 0 runs it one way;
 *                       swapping the two runs it the other way. NEVER drive both.
 *           CH3..CH6    OUTPUTFUNC_DISABLED — outputs float.
 *
 *         This example demonstrates:
 *           - The exact bring-up order and the settle delays that make it reliable
 *           - Configuring ONCE and never again (and why re-configuring breaks the module)
 *           - VERIFYING that the configuration actually reached the module
 *           - The 10 ms send_values loop with per-channel current feedback
 *           - The five readings that tell you where a dead output really went wrong
 *
 *         Build: this file is a subsystem for an S1 application, not a standalone main().
 *         Drop it in an application unit directory (see `make/README.md`), call
 *         output_module_6ch_s1_init() from main() before osKernelStart(), and define
 *         `osThreadId_t model_step_thread` (AGENTS.md rule 12).
 **************************************************************************************/

#include <stdint.h>

#include "GO_board.h"
#include "GO_communication_modules.h"
#include "GO_module_output.h"
#include "cmsis_os2.h"
#include "print.h"

/****************************************************************************************
 * Configuration
 ****************************************************************************************/

#define OUTPUT_SLOT          MODULESLOT1   /* physical slot 1 (0-based macro)            */
#define CHANNEL_CURRENT_MAX  4000u         /* per-channel continuous limit in mA (<=4000) */
#define DUTY_MAX             1000u         /* 0..1000 = 0.0..100.0 %                      */

/* PWM frequency nibble for every channel pair.
 *
 * Use the value the WORKING Linux application programs: it writes
 * `configuration[ch] = 4 | (func << 4)` for every channel of every 6-channel output
 * module. The CodeBase frequency table is off by one against the SPI specification
 * (spec: 1 = 200 Hz, 2 = 500 Hz, 3 = 1 kHz, 4 = 2 kHz, 5 = 5 kHz, 6 = 10 kHz — there is
 * no 100 Hz, and OUTPUTFREQ_6CH_10KHZ = 7 is outside the spec range), so the macro names
 * cannot be trusted as documentation. A frequency value the module firmware does not
 * recognise can leave a duty-controlled channel silent WITHOUT any error bit. */
#define PWM_FREQ             OUTPUTFREQ_6CH_1KHZ   /* nibble 4 — Linux reference value   */

/* --- Timing ---------------------------------------------------------------------------
 * Every one of these has a failure mode attached to it. */

/* Let a freshly powered module finish booting before the first reset. On S1 the module
 * powers up together with the MCU (unlike Linux, where the module has been up for
 * seconds by the time the application starts). */
#define BOOT_SETTLE_MS       50u

/* Between a successful detect and the configuration upload. The second bootloader escape
 * makes the module LEAVE its bootloader and START ITS APPLICATION;
 * GO_communication_modules_initialize() only waits 3 ms after that. A configuration that
 * lands in that gap cannot be processed by the module — and nobody notices, because the
 * module registers fine and answers feedback frames afterwards. Symptom: everything looks
 * healthy, all channels report duty 0, nothing moves. */
#define POST_DETECT_MS       300u

/* Between the configuration burst and the first send_values. The module has to digest two
 * back-to-back config messages and re-arm its SPI DMA; reads taken inside that window
 * come back byte-shifted and fail their checksum. */
#define POST_CONFIG_MS       100u

/* The module loop period. AGENTS.md rule 8: minimum 10 ms (faster overdrives the module
 * SPI slave and produces garbled frames), maximum 400 ms for output modules (the module's
 * own watchdog disables all outputs beyond that). 10 ms is the canonical value. */
#define LOOP_MS              10u

/****************************************************************************************
 * Local data
 ****************************************************************************************/

/* File-scope, never on the stack: the CodeBase module drivers keep shared static message
 * buffers and expect the struct to live as long as the module does. */
static _outputModule outputModule;

static volatile uint8_t s_configured = 0u;   /* 1 once bring-up completed */

static const osThreadAttr_t s_task_attrs = {
	.name       = "out6ch",
	/* 2048, not the default 1024: printf/dbg() frames in the detect path overflow 1024
	 * and hard-fault the MCU in a way that looks like a module problem. */
	.stack_size = 2048u,
	.priority   = (osPriority_t)osPriorityNormal,
};

/****************************************************************************************
 * Step 1 — reset + detect, EXACTLY ONCE
 ****************************************************************************************/

/* AGENTS.md rule 2: initialize() before set_module_slot(), or you get a misleading
 * "contested slot" error. AGENTS.md rule 13: initialize() resets the module ONCE and then
 * retries only the bootloader-escape handshake — never call it in a tight loop, because
 * re-asserting reset before the module has booted throws it straight back into its
 * bootloader and blocks detection.
 *
 * Returns 0 when the expected module type is registered in the slot. */
static int detect_module(void)
{
	GO_module_output_set_module_type(&outputModule, OUTPUTMODULE6CHANNEL);

	osDelay(BOOT_SETTLE_MS);

	/* Reset + two bootloader escapes + registration of the module's real ID and firmware
	 * stamp. The firmware stamp matters: GO_module_output_configuration() derives
	 * sw_version from it, and sw_version >= 2.0.0 selects the V2 SPI protocol. Adopt a
	 * module with a made-up stamp and you may be speaking the wrong protocol version at
	 * it for the rest of the session. */
	(void)GO_communication_modules_initialize(OUTPUT_SLOT);

	/* Verifies the registered ID against OUTPUTMODULE6CHANNELID {20,20,2}. A negative
	 * return means detection did not land — do NOT press on, because configuration() and
	 * send_values() will then quietly do nothing useful (-ENODEV) and every channel reads
	 * as 0, which looks exactly like a broken module. */
	return GO_module_output_set_module_slot(&outputModule, OUTPUT_SLOT);
}

/****************************************************************************************
 * Step 2 — build the configuration (memory only, no SPI)
 ****************************************************************************************/

/* AGENTS.md rule 4: every configure_* call must happen BEFORE configuration(), which is
 * the single commit that ships the gathered configuration over SPI.
 *
 * AGENTS.md rule 17: CHECK THE RETURN VALUES. `*_configure_channel()` validates its
 * arguments FIRST and returns -EINVAL *without writing configuration[]*, so one
 * out-of-range parameter leaves that channel with function nibble 0 — a floating output —
 * while everything downstream reports success. Returns 0 only when every call was accepted. */
static int build_config(void)
{
	int rc = 0;

	/* Fold in every return value but keep going, so one run names every bad parameter. */
	#define CFG_CHECK(what, call)                                                    \
		do {                                                                         \
			int rc_ = (call);                                                        \
			if (rc_ != 0) {                                                          \
				err("out6ch: %s rejected (%d) — channel left UNCONFIGURED "          \
				    "(floating); check the parameter against the *MAX macros\n",      \
				    (what), rc_);                                                    \
				rc = rc_;                                                            \
			}                                                                        \
		} while (0)

	/* Frequency is per PAIR of channels (CH1&2, CH3&4, CH5&6), not per channel.
	 * configure_frequency() writes only the frequency nibble and configure_channel()
	 * writes only the function nibble, so the two are order-independent. */
	CFG_CHECK("freq CH1&2",
		GO_module_output_configure_frequency(&outputModule, OUTPUTFREQCHANNEL1AND2, PWM_FREQ));
	CFG_CHECK("freq CH3&4",
		GO_module_output_configure_frequency(&outputModule, OUTPUTFREQCHANNEL3AND4, PWM_FREQ));
	CFG_CHECK("freq CH5&6",
		GO_module_output_configure_frequency(&outputModule, OUTPUTFREQCHANNEL5AND6, PWM_FREQ));

	/* CH1 + CH2: the half-bridge pair that drives the load.
	 *
	 * peak_current and peak_time are 0 ON PURPOSE. They only apply to
	 * OUTPUTFUNC_6CH_PEAKANDHOLD (function 7). They are stored in channelParameter1/2,
	 * which are UNIONS that mean fastLoopGain / fastLoopBasicDuty for other functions, so
	 * non-zero values there feed a half-bridge channel parameters it never asked for. The
	 * Linux reference does set 1000/500 here and works, so this is not the difference
	 * between driving and not driving — but 0/0 is what the specification asks for. */
	CFG_CHECK("CH1 half bridge",
		GO_module_output_6ch_configure_channel(&outputModule, OUTPUTCHANNEL1,
		                                      OUTPUTFUNC_6CH_HALFBRIDGE,
		                                      CHANNEL_CURRENT_MAX, 0u, 0u));
	CFG_CHECK("CH2 half bridge",
		GO_module_output_6ch_configure_channel(&outputModule, OUTPUTCHANNEL2,
		                                      OUTPUTFUNC_6CH_HALFBRIDGE,
		                                      CHANNEL_CURRENT_MAX, 0u, 0u));

	/* Need a limit ABOVE CURRENTMAXMAX (4000)? configure_channel() rejects it outright, so
	 * write the field directly afterwards — which is exactly what the Simulink-generated
	 * Linux application does (it programs 4500 and never calls the validating helpers):
	 *
	 *   outputModule.currentMax[OUTPUTCHANNEL1] = 4500u;
	 *
	 * Only do that when the module's per-channel rating really allows it. */

	/* Unused channels. NOTE: the Linux reference gives all six channels a real function
	 * and uses configuration[ch] = 0 (not OUTPUTFUNC_DISABLED) when it wants outputs to
	 * float on shutdown — so treat the "disabled" encoding as the one thing here that is
	 * not confirmed against a working system. */
	for (uint8_t ch = OUTPUTCHANNEL3; ch <= OUTPUTCHANNEL6; ch++) {
		CFG_CHECK("CH3..CH6 disable",
			GO_module_output_6ch_configure_channel(&outputModule, ch,
			                                      OUTPUTFUNC_DISABLED, 0u, 0u, 0u));
	}

	#undef CFG_CHECK

	/* Log the bytes that will actually be shipped, so a rejected parameter shows up as a 0
	 * function nibble instead of having to be inferred. Expect 0x24 for a half bridge at
	 * frequency nibble 4. */
	info("out6ch: config bytes CH1/CH2 = 0x%02X/0x%02X, currentMax = %u mA, build rc=%d\n",
	     (unsigned)outputModule.configuration[OUTPUTCHANNEL1],
	     (unsigned)outputModule.configuration[OUTPUTCHANNEL2],
	     (unsigned)outputModule.currentMax[OUTPUTCHANNEL1], rc);

	return rc;
}

/****************************************************************************************
 * Step 3 — upload the configuration ONCE, and verify it
 ****************************************************************************************/

/* AGENTS.md rule 15: upload the configuration exactly once. NOT in a retry loop, and not
 * again later at runtime. Repeated reconfiguration breaks the module's SPI
 * synchronisation: measured result was errorCode 0x30000000
 * (INIT_FAILURE | NOT_COMMUNICATING) with temperature/supply frozen at their last valid
 * values. The Linux reference configures each output module exactly once, at startup.
 *
 * AGENTS.md rule 16: the return value of configuration() is NOT proof of delivery. The
 * configuration is transmit-only — no response, no checksum check — so a 0 return means
 * only that the module is registered and that the transfer ran. The first evidence that
 * the configuration actually landed is the feedback in the drive loop below: a module
 * that answers with a valid header but reports dutyCycle 0 for a commanded channel never
 * received (or applied) its configuration. */
static void upload_and_verify_config(void)
{
	int cfg = GO_module_output_configuration(&outputModule);

	info("out6ch: configuration() = %d | cfg bytes CH1/CH2 = 0x%02X/0x%02X "
	     "(expect func nibble 2 | freq nibble 4 = 0x24)\n",
	     cfg,
	     (unsigned)outputModule.configuration[OUTPUTCHANNEL1],
	     (unsigned)outputModule.configuration[OUTPUTCHANNEL2]);
}

/****************************************************************************************
 * Step 4 — the drive loop
 ****************************************************************************************/

static void output_module_6ch_s1_task(void *argument)
{
	(void)argument;

	if (detect_module() != 0) {
		/* Nothing below this point can work. Say so loudly instead of running a loop
		 * that reports zeros for every channel. */
		err("out6ch: module not detected in slot %u — check power-cycle the module, the "
		    "reset line, and AGENTS.md rules 13/14\n", (unsigned)(OUTPUT_SLOT + 1u));
		for (;;) { osDelay(1000u); }
	}

	osDelay(POST_DETECT_MS);   /* let the module's own application come up */

	/* Uploading a rejected configuration would ship a half-configured module and report
	 * success. Refuse — the err() lines from build_config() name the offending parameter. */
	if (build_config() != 0) {
		err("out6ch: ABORTED — channel configuration rejected, not uploading\n");
		for (;;) { osDelay(1000u); }
	}
	upload_and_verify_config();

	osDelay(POST_CONFIG_MS);   /* let the module digest the config and re-arm its DMA */

	s_configured = 1u;

	uint16_t duty     = 0u;
	int      duty_dir = 1;
	uint8_t  reverse  = 0u;    /* 0 = drive CH1, 1 = drive CH2 */
	uint16_t cycle    = 0u;

	for (;;) {
		/* Ramp the duty 0 -> 1000 -> 0, then swap direction. The two channels are
		 * MUTUALLY EXCLUSIVE: driving both legs of a half-bridge pair at once shorts the
		 * load across the supply. */
		duty = (uint16_t)((int)duty + duty_dir * 5);
		if (duty >= DUTY_MAX) { duty = DUTY_MAX; duty_dir = -1; }
		if (duty == 0u)       { duty_dir =  1; reverse = !reverse; }

		outputModule.value[OUTPUTCHANNEL1] = reverse ? 0u   : duty;
		outputModule.value[OUTPUTCHANNEL2] = reverse ? duty : 0u;

		/* One exchange: the values go out, the feedback comes back in the same frame.
		 * A non-zero return means the response failed its checksum or header check —
		 * everything in the struct is then STALE, so never act on it. */
		int res = GO_module_output_send_values(&outputModule);

		/* Once per second: the five readings that localise a dead output.
		 *
		 *   res      != 0        -> no valid exchange; all values below are stale.
		 *   duty     ~= command  -> the module IS switching. A dead load is then wiring
		 *                           or the power stage, not software.
		 *   duty     == 0        -> the module is not switching: the channel
		 *                           configuration never landed (see step 3) or the
		 *                           function/frequency encoding is wrong.
		 *   supply   ~= 12000+   -> the module measures its own power rail. Low or 0 is
		 *                           the classic "communicates but never drives".
		 *   err      != 0        -> the module reports a fault itself; decode against the
		 *                           OUTPUTERR_* macros in GO_module_output.h.
		 *                           0x08000000 = its 400 ms comms watchdog tripped, which
		 *                           also proves it validates the frames you send. */
		if (++cycle >= 100u) {
			cycle = 0u;
			info("out6ch: res=%d cmd=[%4u,%4u] duty=[%4u,%4u] cur=[%5d,%5d] mA "
			     "temp=%3d C supply=%5u mV gnd=%4d err=0x%08lX\n",
			     res,
			     (unsigned)outputModule.value[OUTPUTCHANNEL1],
			     (unsigned)outputModule.value[OUTPUTCHANNEL2],
			     (unsigned)outputModule.dutyCycle[OUTPUTCHANNEL1],
			     (unsigned)outputModule.dutyCycle[OUTPUTCHANNEL2],
			     (int)outputModule.current[OUTPUTCHANNEL1],
			     (int)outputModule.current[OUTPUTCHANNEL2],
			     (int)outputModule.temperature,
			     (unsigned)outputModule.supply,
			     (int)outputModule.ground,
			     (unsigned long)outputModule.errorCode);
		}

		osDelay(LOOP_MS);
	}
}

/****************************************************************************************
 * Public entry point
 ****************************************************************************************/

/* 1 once detection + configuration have completed. Other subsystems should wait for this
 * before commanding outputs. */
uint8_t output_module_6ch_s1_ready(void) { return s_configured; }

/* Spawn the task. Call from main() BEFORE osKernelStart(): all module SPI on S1 blocks on
 * an RTOS semaphore, so no module I/O may happen before the scheduler runs.
 *
 * If a SECOND module shares the SPI bus, guard every module call with one shared mutex —
 * the CodeBase drivers have no internal bus lock and use shared static buffers — and let
 * the other slots wait until this task has finished its detect handshake, so the fragile
 * bootloader exchange runs uninterrupted. */
void output_module_6ch_s1_init(void)
{
	if (osThreadNew(output_module_6ch_s1_task, NULL, &s_task_attrs) == NULL) {
		err("out6ch: failed to create task\n");
	}
}
