# GOcontroll Linux Examples

Each example is a self-contained `main()` that can be built independently.

## Building an example

```bash
make <example_name>        # e.g. make led_blink
```

The output binary is always placed at `build/app`.
Cross-compile for ARM64 by prefixing `CC=aarch64-linux-gnu-gcc`.

---

## Examples

### `led_blink`
Blinks the green status LED on LED slot 1 at 1 Hz (500 ms on / 500 ms off).
Demonstrates the minimal startup sequence: hardware version detection, LED
initialisation, and a clean shutdown handler.

### `input_module_10ch_selftest`
Hardware self-test for a 10-channel input module on slot 1. Runs three phases
using the on-board pull resistors as a known load (no external wiring required):
pull-up only, pull-down only, and both active. Expected ADC counts are calculated
from the input circuit component values (R51=10kΩ, R52=3.3kΩ, R41=105kΩ,
R42=210kΩ, 3.3V reference). Each channel is checked against the expected range
and reported as PASS or FAIL. Returns exit code 0 on success, 1 on failure.

### `input_module_10ch`
Reads all 10 channels of a 10-channel input module on slot 1 and prints the
measured voltages in millivolts once per second. All channels are configured for
analog mV resolution with a 3.3 kΩ pull-up and 3.3 kΩ pull-down resistor.
Demonstrates module type/slot assignment, per-channel configuration, sending the
configuration to the hardware, and polling values in the application loop.

### `input_module_6ch`
Reads all 6 channels of a 6-channel input module on slot 1 and prints the measured
voltages in millivolts once per second. Demonstrates the 6-channel-specific
configuration: voltage range selection (5 V / 12 V / 24 V), per-channel analogue
filter depth (ADC averaging), and the three independent 5 V sensor supply outputs.
All channels are configured for analogue mV measurement with a 24 V range, 10 kΩ
pull-up and 3.3 kΩ pull-down. Sensor supply 1 is switched on.

### `output_module_6ch`
Drives a 6-channel output module on slot 2 using two output functions simultaneously.
CH1–CH4 are configured as high-side boolean outputs and cycle on one at a time
(500 ms each). CH5–CH6 are configured as high-side PWM and the duty cycle ramps
continuously from 0 to 100 % and back, at a PWM frequency of 1 kHz. Demonstrates
per-channel current limits, PWM frequency configuration (`GO_module_output_configure_frequency`),
and reading per-channel current feedback and module temperature after each
`GO_module_output_send_values()` call.

### `read_supply_voltages`
Reads the K30 (battery) and K15-A/B/C (ignition-switched) supply voltages and
prints them in millivolts once per second. Demonstrates starting the background
ADC thread, polling the voltage cache with `GO_board_controller_power_voltage()`,
and cleanly stopping the ADC thread on shutdown.
