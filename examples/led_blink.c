/**************************************************************************************
 * \file   led_blink.c
 * \brief  Blink the green status LED at 1 Hz (500 ms on / 500 ms off).
 *
 *         This is the minimal example for LED control on the GOcontroll
 *         Moduline Linux platform. It demonstrates:
 *           - Hardware version detection (required before using LEDs)
 *           - LED initialisation and control
 *           - Clean shutdown on SIGTERM / SIGINT (Ctrl+C)
 *           - A fixed-interval application loop using usleep()
 **************************************************************************************/

#include <stdint.h>
#include <unistd.h>

#include "GO_board.h"
#include "print.h"

/* -------------------------------------------------------------------------
 * Shutdown callback — registered via GO_board_exit_program().
 * Called automatically when the process receives SIGTERM or SIGINT (Ctrl+C).
 * Turn the LED off before exiting so it does not stay on after the program ends.
 * ------------------------------------------------------------------------- */
static void app_terminate(void)
{
	info("Shutting down — turning LED off\n");
	GO_board_status_leds_led_control(1, LED_COLOR_GREEN, 0);
}

/* -------------------------------------------------------------------------
 * main
 * ------------------------------------------------------------------------- */
int main(void)
{
	info("=== LED blink example ===\n");
	info("Green LED 1 will blink at 1 Hz. Press Ctrl+C to stop.\n");

	/* Detect the hardware variant (module count, LED controller type, ADC type).
	 * This must be called before any LED or module function. */
	GO_board_get_hardware_version();

	/* Initialise the LED controller (I2C on Linux, TIM3 PWM on STM32). */
	GO_board_status_leds_initialize();
	info("LED controller initialised\n");

	/* Register the shutdown callback for SIGTERM and SIGINT. */
	GO_board_exit_program(app_terminate);

	/* --- Application loop (10 ms cycle) ---------------------------------- */
	uint8_t led_state = 0;
	int     cycle     = 0;

	while (1) {
		/* Toggle the LED every 50 cycles: 50 × 10 ms = 500 ms.
		 * This gives a 1 Hz blink (500 ms on, 500 ms off). */
		if (++cycle >= 50) {
			cycle     = 0;
			led_state = !led_state;

			/* LED 1, green channel. Value 0 = off, 1 = on.
			 * Use LED_COLOR_RED or LED_COLOR_BLUE for other colours. */
			GO_board_status_leds_led_control(1, LED_COLOR_GREEN, led_state);
			info("LED %s\n", led_state ? "ON" : "OFF");
		}

		usleep(10000); /* 10 ms */
	}

	return 0;
}
