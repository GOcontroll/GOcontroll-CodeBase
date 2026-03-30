/**************************************************************************************
 * \file   xcp_basic.c
 * \brief  XCP over TCP example — platform: Moduline IV / Moduline Mini (Linux).
 *
 *         Demonstrates how to expose variables for HANtune via the XCP protocol.
 *
 *         Variable naming convention used throughout this example:
 *           xcpr_ prefix — MEASUREMENT  (read-only signal, written by the application,
 *                                        read by HANtune for live monitoring)
 *           xcpw_ prefix — CHARACTERISTIC (writable parameter, set from HANtune,
 *                                          read by the application to change behaviour)
 *
 *         All XCP-visible variables are declared volatile at file scope so the
 *         compiler cannot cache or remove them, and the XCP stack can access them
 *         directly by address.
 *
 *         Exposed variables:
 *           Measurements  — supply voltages K30, K15-A, K15-B, K15-C  (uint16, mV)
 *           Characteristics — blue LED intensity for LED 1–4           (uint8,  0-255)
 *
 *         XCP transport: TCP, port 50002.
 *         Connect HANtune to <controller-IP>:50002 using xcp_basic.a2l.
 *
 *         After each build, run the address update script to sync the a2l:
 *           ./GOcontroll-CodeBase/examples/update_xcp_a2l.sh \
 *               build/app.elf \
 *               GOcontroll-CodeBase/examples/xcp_basic.a2l \
 *               xcp_basic_connected.a2l
 *         Open xcp_basic_connected.a2l in HANtune.
 *
 *         Build:
 *           make xcp_basic
 **************************************************************************************/

#include <stdint.h>
#include <unistd.h>
#include <pthread.h>

#include "GO_board.h"
#include "GO_xcp.h"
#include "print.h"

/* ============================================================================
 * XCP-visible variables
 *
 * Place all XCP variables here, at file scope, with the volatile qualifier.
 * The a2l file references these variables by name; the update_xcp_a2l.sh
 * script reads their addresses from the ELF and patches the a2l after each build.
 * ========================================================================= */

/* --- Measurements: supply voltages (xcpr_ = XCP read, written by app, read by HANtune) --- */
volatile uint16_t xcpr_K30_mV  = 0u;  /**< Battery voltage  K30  (mV) */
volatile uint16_t xcpr_K15A_mV = 0u;  /**< Ignition voltage K15-A (mV) */
volatile uint16_t xcpr_K15B_mV = 0u;  /**< Ignition voltage K15-B (mV) */
volatile uint16_t xcpr_K15C_mV = 0u;  /**< Ignition voltage K15-C (mV) */

/* --- Characteristics: LED intensities (xcpw_ = XCP write, set by HANtune, read by app) --- */
volatile uint8_t  xcpw_LED1_Blue = 0u; /**< Blue intensity LED 1 (0 = off, 255 = max) */
volatile uint8_t  xcpw_LED2_Blue = 0u; /**< Blue intensity LED 2 (0 = off, 255 = max) */
volatile uint8_t  xcpw_LED3_Blue = 0u; /**< Blue intensity LED 3 (0 = off, 255 = max) */
volatile uint8_t  xcpw_LED4_Blue = 0u; /**< Blue intensity LED 4 (0 = off, 255 = max) */

/* ============================================================================
 * Internals
 * ========================================================================= */

/* ADC sample interval for power supply readings */
#define ADC_SAMPLE_MS   100u

/* Main loop cycle time */
#define CYCLE_US        10000u   /* 10 ms */

static void app_terminate(void)
{
    info("Shutting down — stopping XCP and LEDs\n");
    GO_board_controller_power_stop_adc_thread();
    GO_xcp_stop_connection();
}

/* ============================================================================
 * main
 * ========================================================================= */
int main(void)
{
    info("=== XCP basic example ===\n");
    info("Measurements : xcpr_K30_mV, xcpr_K15A_mV, xcpr_K15B_mV, xcpr_K15C_mV\n");
    info("Characteristics: xcpw_LED1_Blue .. xcpw_LED4_Blue\n");
    info("XCP transport  : TCP port 50002\n\n");

    GO_board_get_hardware_version();

    /* --- Status LEDs -------------------------------------------------------- */
    GO_board_status_leds_initialize();

    /* --- ADC thread for supply voltages ------------------------------------- */
    GO_board_controller_power_start_adc_thread(ADC_SAMPLE_MS);

    /* --- XCP over TCP (runs in a background thread) ------------------------- */
    pthread_t xcp_thread;
    pthread_create(&xcp_thread, NULL, GO_xcp_initialize_tcp, NULL);
    pthread_detach(xcp_thread);

    info("XCP listening on TCP port 50002 — connect HANtune now\n\n");

    /* --- Shutdown callback -------------------------------------------------- */
    GO_board_exit_program(app_terminate);

    /* --- Application loop (10 ms) ------------------------------------------ */
    while (1) {

        /* Read supply voltages into XCP measurement variables */
        GO_board_controller_power_voltage(1u, (uint16_t *)&xcpr_K30_mV);
        GO_board_controller_power_voltage(2u, (uint16_t *)&xcpr_K15A_mV);
        GO_board_controller_power_voltage(3u, (uint16_t *)&xcpr_K15B_mV);
        GO_board_controller_power_voltage(4u, (uint16_t *)&xcpr_K15C_mV);

        /* Apply XCP characteristic values to the blue LED channels */
        GO_board_status_leds_led_control(1u, LED_COLOR_BLUE, xcpw_LED1_Blue);
        GO_board_status_leds_led_control(2u, LED_COLOR_BLUE, xcpw_LED2_Blue);
        GO_board_status_leds_led_control(3u, LED_COLOR_BLUE, xcpw_LED3_Blue);
        GO_board_status_leds_led_control(4u, LED_COLOR_BLUE, xcpw_LED4_Blue);

        usleep(CYCLE_US);
    }

    return 0;
}
