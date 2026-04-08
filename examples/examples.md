# GOcontroll Linux Examples

Each example is a self-contained `main()` that can be built independently.

## Building an example

```bash
make <example_name>        # e.g. make led_blink
```

The output binary is always placed at `build/app`.
Cross-compile for ARM64 by prefixing `CC=aarch64-linux-gnu-gcc`.

---

## Directory structure

```
examples/
├── board/
│   ├── led_blink.c
│   └── read_supply_voltages.c
├── communication/
│   ├── can_send.c
│   ├── can_receive.c
│   ├── mqtt_basic.c
│   ├── xcp_basic.c
│   ├── xcp_basic.a2l
│   └── update_xcp_a2l.sh
├── input_module_10ch/
│   ├── input_module_10ch.c
│   ├── input_module_10ch_selftest.c
│   └── input_module_10ch_freq.c
├── input_module_6ch/
│   └── input_module_6ch.c
├── input_module_420ma/
│   └── input_module_420ma.c
├── output_module_6ch/
│   └── output_module_6ch.c
├── output_module_10ch/
│   └── output_module_10ch.c
└── bridge_module/
    └── bridge_module.c
```

---

## Examples

### board

#### `led_blink`
Blinks the green status LED on LED slot 1 at 1 Hz (500 ms on / 500 ms off).
Demonstrates the minimal startup sequence: hardware version detection, LED
initialisation, and a clean shutdown handler.

#### `read_supply_voltages`
Reads the K30 (battery) and K15-A/B/C (ignition-switched) supply voltages and
prints them in millivolts once per second. Demonstrates starting the background
ADC thread, polling the voltage cache with `GO_board_controller_power_voltage()`,
and cleanly stopping the ADC thread on shutdown.

---

### communication

#### `can_send`
Sends a CAN frame on CAN bus 1 once per second. Demonstrates opening a CAN
socket, composing a frame, and transmitting it.

#### `can_receive`
Receives CAN frames on CAN bus 1 and prints each frame to the console.
Demonstrates opening a CAN socket and blocking on receive.

#### `mqtt_basic`
Connects to a local MQTT broker, publishes CH1 of a 10-channel input module to
`/gocontroll/sensor/ch1` once per second, and subscribes to
`/gocontroll/cmd/ch1` for incoming commands. Demonstrates MQTT configure,
subscribe, publish, and the status callback.

#### `xcp_basic`
Exposes a set of variables over XCP-on-UDP so they can be measured and
calibrated from a host tool (e.g. CANape). After building, `update_xcp_a2l.sh`
patches the symbol addresses from the ELF into `xcp_basic.a2l` and writes
`xcp_basic_connected.a2l` ready for the host tool.

---

### input_module_10ch

#### `input_module_10ch`
Reads all 10 channels of a 10-channel input module on slot 1 and prints the
measured voltages in millivolts once per second. All channels are configured for
analog mV measurement with a 10 kΩ pull-up and 3.3 kΩ pull-down. Both 5 V
sensor supplies are enabled.

#### `input_module_10ch_selftest`
Hardware self-test for a 10-channel input module on slot 1. Runs three phases
using the on-board pull resistors as a known load (no external wiring required):
pull-up only, pull-down only, and both active. Expected ADC counts are calculated
from the input circuit component values (R51=10kΩ, R52=3.3kΩ, R41=105kΩ,
R42=210kΩ, 3.3V reference). Each channel is checked against the expected range
and reported as PASS or FAIL. Returns exit code 0 on success, 1 on failure.

#### `input_module_10ch_freq`
Reads a frequency signal on CH1 of a 10-channel input module on slot 1 and
prints the measured frequency in Hz once per second. CH1 is configured as
`INPUTFUNC_FREQUENCY` without pull-up or pull-down (externally driven signal).
CH2–CH10 remain configured as analog mV inputs. Both 5 V sensor supplies are
enabled.

---

### input_module_6ch

#### `input_module_6ch`
Reads all 6 channels of a 6-channel input module on slot 1 and prints the measured
voltages in millivolts once per second. Demonstrates the 6-channel-specific
configuration: voltage range selection (5 V / 12 V / 24 V), per-channel analogue
filter depth (ADC averaging), and the three independent 5 V sensor supply outputs.
All channels are configured for analogue mV measurement with a 24 V range, 10 kΩ
pull-up and 3.3 kΩ pull-down. Sensor supply 1 is switched on.

---

### input_module_420ma

#### `input_module_420ma`
Reads all channels of a 4–20 mA input module and prints the measured current in
microamps once per second.

---

### output_module_6ch

#### `output_module_6ch`
Drives a 6-channel output module on slot 2 using two output functions simultaneously.
CH1–CH4 are configured as high-side boolean outputs and cycle on one at a time
(500 ms each). CH5–CH6 are configured as high-side PWM and the duty cycle ramps
continuously from 0 to 100 % and back, at a PWM frequency of 1 kHz. Demonstrates
per-channel current limits, PWM frequency configuration (`GO_module_output_configure_frequency`),
and reading per-channel current feedback and module temperature after each
`GO_module_output_send_values()` call.

---

### output_module_10ch

#### `output_module_10ch`
Drives a 10-channel output module on slot 2. All 10 channels are configured as
high-side boolean outputs and cycle on one at a time, switching each on for
500 ms before moving to the next.

---

### bridge_module

#### `bridge_module`
Drives a bridge module. Demonstrates configuring and controlling the bridge
output channels.
