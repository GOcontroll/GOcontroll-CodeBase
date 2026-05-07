# AGENTS.md — Index for AI coding tools

This repository is a C library for programming **GOcontroll Moduline controllers**.
It supports two build flavours sharing the same public API:

| Define              | Target                                  | Threading        | CAN backend     |
|---------------------|-----------------------------------------|------------------|-----------------|
| `GOCONTROLL_LINUX`  | Moduline L4 / Moduline M1 (IMX8 ARM64)| POSIX pthreads   | SocketCAN       |
| `GOCONTROLL_S1`     | Moduline S1 (STM32H5)                   | FreeRTOS / CMSIS | STM32 FDCAN HAL |

Pick exactly **one** of these defines per build. Never both.

> **Naming note:** the S1 platform is selected by the legacy build define
> `GOCONTROLL_IOT` and lives under `code/iot/`. The source identifiers are
> kept for compatibility with the previous product naming; all prose in
> this repo refers to the platform as **S1**.

Top-level layout:

```
code/        Public C library — board, communication, modules, XCP, S1 subtree
examples/    Self-contained main()s, one per topic (Linux only at present)
lib/         Vendored third-party (IIO, JSON-C, OAES) — do not modify
```

---

## Hard rules — violate these and code will misbehave

1. **`GO_board_get_hardware_version()` must be the first call** in `main()`, before
   any LED, module, or communication API. It populates `hardwareConfig` (module
   count, LED controller type, ADC type) which every other layer reads.
   See `code/GO_board.h:218` and any `examples/**/main*.c`.

2. **`GO_communication_modules_initialize(slot)` must run before
   `GO_module_*_set_module_slot(slot)`.** The initialise call populates
   `hardwareConfig.moduleOccupancy`; `set_module_slot` reads that data to verify
   the correct module type is physically present. Reversing the order produces a
   **"contested slot"** error at runtime.
   See `examples/input_module_6ch/input_module_6ch.c:32`.

3. **Sensor supply (`*_configure_supply`) must be set before `*_configuration()`.**
   Supply state is included in the first SPI configuration frame; configuring
   supplies after `*_configuration()` will not reach the module until the next
   reconfiguration.
   See `examples/input_module_6ch/input_module_6ch.c:108`.

4. **Channel config (`*_configure_channel`) must come before `*_configuration()`.**
   `*_configuration()` is the single commit that ships the gathered config to
   the module over SPI.

5. **Don't mix 6ch/10ch input macros.** Use `INPUTPULLUP6CH_*` and
   `INPUTPULLDOWN6CH_*` for the 6-channel module; `INPUTPULLUP10CH_*` /
   `INPUTPULLDOWN10CH_*` for the 10-channel module. The numeric values overlap
   but mean different resistor values, so silently picking the wrong macro
   gives a wrong but plausible reading.
   See `code/modules/GO_module_input.h:64-74`.

6. **The ADC thread must be started before reading supply voltages.** Call
   `GO_board_controller_power_start_adc_thread(sample_time_ms)` first; otherwise
   `GO_board_controller_power_voltage()` returns 0 mV.
   See `code/GO_board.h:150`.

7. **`GO_board_controller_info_task_start()` must run before the scheduler starts.**
   It creates the 10 Hz IMU task. On Linux this means before the application
   loop; on S1 this means before `osKernelStart()`.
   See `code/GO_board.h:226`.

8. **Module communication window: 10 ms ≤ period ≤ 400 ms.**
   - **Minimum 10 ms** between successive `*_receive_values()` /
     `*_send_values()` calls per module. Anything faster overdrives the
     module SPI bus and produces dropped/garbled frames.
   - **Maximum 400 ms** for *output modules* (output 6ch, output 10ch,
     bridge). The module runs its own communication-loss watchdog: if no
     valid frame arrives within 400 ms it disables all outputs as a fail-safe.
     This means a stalled or paused application loop will silently drop
     loads from the system.
   - Canonical period is **10 ms** (`usleep(10000)` on Linux,
     `osDelay(10)` on S1). Use 1 ms only for low-latency CAN RX polling
     in dedicated tasks — never as the module loop.

9. **MQTT setup order:** `sub_register` → `configure` → `subscribe` → `enable(true)`.
   Subscriptions must be registered before `enable(true)` so the network thread
   sees the buffers when it connects.
   See `code/GO_communication_mqtt.h:11`.

10. **CAN ISR-safety:** `GO_communication_can_bus_off_recovery`,
    `can_busload_count_frame`, and `GO_communication_can_rx_push` are designed
    to be called from FDCAN HAL ISR callbacks (S1 only). Anything else
    must run from task context.
    See `code/GO_communication_can.h:121,154,190`.

---

## Where to look

The source-of-truth lives in:

- File-level doc-blocks at the top of each `code/**/*.h` header.
- Function-level Doxygen blocks above each prototype.
- Inline comments in the canonical examples — especially
  `examples/input_module_6ch/input_module_6ch.c` (the most rigorous one;
  it documents the required initialisation order and rationale verbatim).
- `examples/examples.md` — one-line description per example.

---

## Common pitfalls

- **"Contested slot at runtime"** → `set_module_slot` was called before
  `GO_communication_modules_initialize`. See rule 2.
- **All voltages read 0 mV** → ADC thread not started. See rule 6.
- **First read returns zeros** → `*_configure_channel` ran after `*_configuration()`.
  See rule 4.
- **License verify killed the process** → `GO_board_verify_license()` calls
  `exit()` on failure; not recoverable from the caller.
- **Output module suddenly disables all outputs** → application loop period
  exceeded 400 ms; the module's own watchdog dropped to fail-safe. See rule 8.
- **CAN bus stops after one bus-off** → recovery only happens when
  `GO_communication_can_bus_off_recovery` is wired into the
  `HAL_FDCAN_ErrorStatusCallback` (S1). See rule 10.

---

## Conventions when generating code for this repo

- Use the canonical 10 ms loop with a cycle counter for sub-rate work
  (`if (++cycle >= PRINT_INTERVAL_CYCLES) { cycle = 0; ... }`). See any example.
- Always register a shutdown callback via `GO_board_exit_program(callback)` on
  Linux. The callback should turn outputs off and stop background threads.
- Module instances are static `_inputModule` / `_outputModule` / `_bridgeModule`
  structs at file scope — do not allocate on the stack inside `main()`.
- For new examples, mirror the file-level doc-block style in
  `examples/input_module_6ch/input_module_6ch.c` (purpose, configuration table,
  initialisation order, "this example demonstrates").
- Do **not** add header-only docstrings that duplicate `examples/examples.md`.
  Cross-link instead.
