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

11. **ESP frame-pacing (S1 only):** `GO_communication_esp.c::SendFrame()` is
    a single-buffer `HAL_UART_Transmit_IT` send with a `s_tx_busy` guard.
    Frames offered while the previous transmit is still in progress are
    **silently dropped** — no retry, no error return, no queue. Practical
    consequence: do **not** call two ESP send functions back-to-back from
    the same task context without spacing them out.

    Common trap: `GO_communication_esp_set_modem_config(...)` followed
    immediately by `GO_communication_esp_enable_lte(true)` looks like one
    "configure-and-enable" action but is two UART frames; the second one
    will not reach the ESP. Same applies to `mqtt_configure` +
    `mqtt_enable`, and to any pair of `_set_*` + `_enable_*` calls.

    Recommended pattern: drive the bring-up from a periodic task with a
    state-machine that sends one frame per tick (e.g. one frame per
    10–100 ms). One UART frame at 115200 baud takes ≤ 50 ms even at the
    512-byte payload limit, so a 100 ms task tick is always safe.
    See `code/GO_communication_esp.c:119–148` for the SendFrame source.

12. **The application must define `osThreadId_t model_step_thread`.** The
    controller-info task (started unconditionally by
    `GO_board_controller_info_task_start()` — see rule 7) reports the
    application's main task stack high-water mark through the global
    `extern osThreadId_t model_step_thread;` declared in `GO_board.c`. Because
    the info task always runs on S1, **every** application that links this
    build must define this symbol or the link fails with
    `undefined reference to model_step_thread`. Assign it the handle of your
    main model/application task for meaningful stack telemetry, or set it to
    `NULL` when there is no single model task (the field then reflects the
    info task itself — cosmetic only).
    See `code/GO_board.c:242`.

13. **A module is reset exactly ONCE during `GO_communication_modules_initialize`.**
    The reset (assert→release of the module's reset line) must be issued a single
    time, **before** the bootloader-escape retry loop — never inside it. A module's
    bootloader opens a short window after one reset and leaves it only on a *clean*
    escape command; a corrupt or failed escape keeps the module in the bootloader,
    so the handshake can simply be retried. Re-asserting reset on every retry
    iteration instead throws the module back into the bootloader and fights its own
    boot timing, which **intermittently prevents detection**: typically one slot's
    module never registers (all its channels read 0) while an identical module in
    another slot — one that happened to succeed on the first try — works fine.
    The retry loop must retry only the escape handshake, not the reset.
    See `code/GO_communication_modules.c` (`GO_communication_modules_initialize`).

14. **S1 module SPI (`hspi1`) needs BOTH `MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE`
    (AFCNTR=1) AND `NSSPMode = SPI_NSS_PULSE_DISABLE`.** With the CubeMX defaults
    (`..._DISABLE` / `..._ENABLE`) the STM32H5 SPI emits a **spurious clock edge at each
    `SPE` enable** (verified on a logic analyzer: a stray SCK pulse right after CS-assert,
    ~8 µs before the real burst — with AFCNTR=0 the peripheral releases SCK to GPIO
    between transfers). That single extra clock advances the module (slave) by one bit
    the STM32 DMA never captures, so **every module frame is read back shifted left by
    1 bit** — e.g. the bootloader signature `9,45,9` reads as `18,18,90`, module id
    `20,20,2` as `40,40,4`. Detection then fails for **every** module with a misleading
    **"contested slot"** error and nothing comes up, while the modules and wiring are
    perfectly fine. **Both settings are required and must be changed together**: with
    AFCNTR=1 but NSSP still enabled the frame is still shifted (a *different* offset,
    e.g. reads `1,1,43,2,...`); NSSP disabled alone (AFCNTR=0) does nothing. SPI clock
    speed and clock phase (CPHA/CPOL) are irrelevant — ruled out on the analyzer. Do
    not revert either one.
    See `code/iot/Core/Src/spi.c` (`MX_SPI1_Init`).

15. **A module is configured EXACTLY ONCE — never in a retry loop, never again at
    runtime.** `*_configuration()` is a single commit, and repeating it breaks the
    module's SPI synchronisation: measured result on a 6-channel output module was
    `errorCode = 0x30000000` (`INIT_FAILURE | NOT_COMMUNICATING`) with `supply` and
    `temperature` frozen at their last valid values. The Linux/Simulink reference
    (`GOcontroll_Linux_3_0.c`) configures each module once, at startup, and afterwards only
    calls `*_send_values()` / `*_receive_values()`. Easy to violate by accident: a helper
    that both *builds* and *uploads* the configuration, called just before an upload retry
    loop, ships the full burst up to six times within a few milliseconds. Keep build and
    upload separate. The corollary: there is no configuration-based recovery path — if a
    module stops answering mid-run, keep sending values (its own watchdog puts outputs in a
    safe state and the exchange resumes by itself). Never re-reset and re-configure a
    powered, running output module: a warm reset cannot return it to its bootloader, so the
    re-detect fails forever *and* drops the load.
    See `examples/output_module_6ch_s1/output_module_6ch_s1.md`.

16. **`GO_communication_modules_send_spi()` confirms nothing — do not treat a 0 return as
    proof of delivery.** It is transmit-only: no response, no checksum check. The return
    value means "the DMA clocked bytes out". Everything that ships configuration through it
    therefore reports success even when the module never received or applied anything,
    which is precisely the question you need answered when a module communicates perfectly
    but drives nothing. Where delivery matters, use `send_receive_spi()` instead: it clocks
    in what the module presents on MISO in the *same* SPI window (identical on the wire —
    SCK/MOSI/CS unchanged; only the master reads along), so a checksum-valid response proves
    the module was alive and correctly framed at that moment. The configuration paths are
    transmit-only, so verify delivery indirectly instead: a module that answers
    `send_values()` with a header-valid frame while `dutyCycle[]` stays 0 for a commanded
    channel was never configured. Note that a suspected non-delivery must not be turned
    into a retry of the upload — see rule 15.
    See `code/modules/GO_module_output.c` (`GO_module_output_configuration`).

17. **Check the return value of every `*_configure_channel()` / `*_configure_frequency()`
    call.** They validate their arguments **first** and return `-EINVAL` *without writing
    `configuration[]`*. So a single out-of-range parameter leaves that channel with function
    nibble 0 — a **floating output** — while `*_configuration()` afterwards happily ships
    the half-built configuration and returns 0. The only trace is one `err()` line, which is
    easy to miss in a busy RTT log, and the end result is indistinguishable from a hardware
    fault: the module communicates perfectly and drives nothing.
    The classic trigger is `currentMax > CURRENTMAXMAX` (4000). Note that the Linux
    reference application programs **4500** on real hardware by writing
    `outputModule.currentMax[ch]` directly and never calling the validating helper — so if
    you need a limit above the library's cap, configure the function with a passing value
    and set the field directly afterwards, and only when the module's rating allows it.
    See `examples/output_module_6ch_s1/output_module_6ch_s1.c` (`build_config`).

---

## Where to look

The source-of-truth lives in:

- File-level doc-blocks at the top of each `code/**/*.h` header.
- Function-level Doxygen blocks above each prototype.
- Inline comments in the canonical examples — especially
  `examples/input_module_6ch/input_module_6ch.c` (the most rigorous one;
  it documents the required initialisation order and rationale verbatim).
- `examples/output_module_6ch_s1/output_module_6ch_s1.{c,md}` — the canonical **S1**
  output-module bring-up procedure: the exact order, the settle delays, how to verify
  that a configuration actually reached the module, and the diagnosis table for
  "communicates fine but drives nothing". Read this before touching module output code
  on an S1.
- `examples/examples.md` — one-line description per example.

---

## Common pitfalls

- **"Contested slot at runtime"** → `set_module_slot` was called before
  `GO_communication_modules_initialize`. See rule 2.
- **All voltages read 0 mV** → ADC thread not started. See rule 6.
- **First read returns zeros** → `*_configure_channel` ran after `*_configuration()`.
  See rule 4.
- **Output module suddenly disables all outputs** → application loop period
  exceeded 400 ms; the module's own watchdog dropped to fail-safe. See rule 8.
- **CAN bus stops after one bus-off** → recovery only happens when
  `GO_communication_can_bus_off_recovery` is wired into the
  `HAL_FDCAN_ErrorStatusCallback` (S1). See rule 10.
- **ESP/modem/MQTT only receives the first of two back-to-back frames**
  (e.g. `MODEM_CONFIG` arrives but `LTE_ENABLE` does not) → caller
  invoked two ESP send functions in the same task tick; the second
  hit the `s_tx_busy` drop. See rule 11.
- **Link error `undefined reference to model_step_thread`** → the
  application did not define the `osThreadId_t model_step_thread` global
  that the controller-info task reads. See rule 12.
- **One module never detected (all its channels read 0) while an identical
  module in another slot works** → the reset was issued *inside* the
  bootloader-escape retry loop (resetting the module up to 5×) instead of once
  before it. See rule 13.
- **An output module communicates perfectly — valid frames, plausible temperature and
  supply, `errorCode = 0` — but no output ever switches** (`dutyCycle[]` reads 0 while a
  non-zero value is commanded). In order of likelihood:
  0. A `*_configure_channel()` call was **rejected** and its return value ignored, so that
     channel's function nibble is still 0 (floating). Look for a lone `err()` line about a
     parameter being "out of range" — `currentMax > 4000` is the usual one — and log the
     `configuration[]` bytes: a half bridge at frequency nibble 4 must read `0x24`, and
     anything with a high nibble of 0 was never configured. See rule 17.
  1. The configuration never reached the module. A transmit-only `cfg == 0` proves
     nothing — see rule 16 for how to establish this from the `send_values()` feedback
     instead of assuming it.
  2. The configuration was uploaded more than once, which desynchronised the module's SPI.
     See rule 15.
  3. The frequency nibble is a value the module firmware does not know. The CodeBase
     frequency table is off by one against the SPI spec (spec: `1 = 200 Hz` … `6 = 10 kHz`;
     there is no 100 Hz and `OUTPUTFREQ_6CH_10KHZ = 7` is out of range), so the macro names
     are not documentation. Use nibble **4** — the value the working Linux application
     programs for every channel: `configuration[ch] = 4 | (func << 4)`.
  4. The configuration landed in the gap where the module was still starting its own
     application (it needs ~300 ms after detection, not the 3 ms `initialize()` waits).
  5. Only then suspect the load, wiring or the module's power rail — and check `supply`,
     which the module measures itself.
  Full procedure and diagnosis table:
  `examples/output_module_6ch_s1/output_module_6ch_s1.md`.
- **A module reports `errorCode = 0x30000000` (`INIT_FAILURE | NOT_COMMUNICATING`) with
  `supply`/`temperature` frozen** → the configuration was uploaded repeatedly. See rule 15.
- **`send_values()` decodes plausible-looking but wrong values** (temperature/current/duty
  from the wrong bytes) → the response header was not validated, so an echo or the answer to
  a previous transaction passed as feedback on its checksum alone. Expected V2 header for
  the 6-channel output module: `[slot, 43, 2, 22, 4, 1]`.
- **ALL modules fail detection with "contested slot"; RTT frame dumps show the
  bootloader response shifted left by 1 bit** (signature `9,45,9` reads as
  `18,18,90`, ids doubled) → the STM32H5 SPI emits a spurious clock edge at each
  `SPE` enable because `MasterKeepIOState` is `DISABLE`. Set it to `ENABLE`
  (AFCNTR=1). The modules/wiring are fine; changing SPI speed/phase/NSSP does
  nothing. See rule 14. To diagnose: build with `-DDEBUG=1` so
  `GO_communication_modules_initialize` dumps the raw `dataRxBoot`/`dataRxFirm`
  frames, and/or scope SCK/MISO/MOSI/CS — a healthy 61-byte transfer is exactly
  488 clock pulses with no stray edge near CS-assert.

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
