# Output module (6 channel) on Moduline S1 — bring-up procedure

Companion to `output_module_6ch_s1.c`. That file is the executable version of this
document; this one explains *why*, and how to diagnose it when an output stays dead.

**The failure mode this exists for:** the module registers, answers every frame with a
valid checksum, reports a plausible temperature and supply voltage and `errorCode = 0` —
and **not one output ever switches**. Nothing in the API returns an error. Every step below
prevents a specific, measured version of that.

The reference implementation is the generated Simulink application for Moduline IV
(`GOcontroll_Linux_3_0.c`, block *"Start for M-S-Function: Output Module (6 channel)"*).
It drives four of these modules successfully and its sequence is: reset+detect once per
module → configure once → `send_values()` on a fixed tick, forever. Nothing else. Where
this procedure looks over-careful, it is because the reference does it this way.

---

## The procedure

| # | Step | Delay after | Why the delay |
|---|------|-------------|---------------|
| 1 | `set_module_type()` | — | Must precede everything; `set_module_slot` validates against it. |
| 2 | *(module powers up with the MCU on S1)* | **50 ms** | On Linux the module has been up for seconds before the app starts. On S1 it boots together with the MCU; resetting a still-booting module fights its own boot timing. |
| 3 | `GO_communication_modules_initialize(slot)` | — | Resets the module **once**, then retries only the bootloader-escape handshake. Registers the real module ID and firmware stamp. |
| 4 | `set_module_slot(slot)` | **300 ms** | The second escape makes the module *leave its bootloader and start its own application*; `initialize()` only waits 3 ms after that. A configuration landing in that gap is lost silently. |
| 5 | `configure_frequency()` × 3, `6ch_configure_channel()` × 6 | — | Memory writes only. No SPI. |
| 6 | `GO_module_output_configuration()` — **exactly once** | **100 ms** | Two back-to-back config messages; the module needs to digest them and re-arm its SPI DMA. Reads inside that window come back byte-shifted and fail their checksum. |
| 7 | `GO_module_output_send_values()` every **10 ms**, forever | — | 10 ms ≤ period ≤ 400 ms (AGENTS.md rule 8). Below 10 ms the module's SPI slave garbles frames; above 400 ms its own watchdog disables all outputs. |

Steps 1–4 must not run in a tight loop, and step 6 must not be repeated. See the traps
below for both.

---

## `configuration() == 0` is not proof that the configuration landed

`GO_communication_modules_send_spi()` is transmit-only: no response, no checksum check. Its
return value means "the DMA clocked bytes out", nothing more. So `configuration() == 0` is
**not** evidence that the module received or applied anything — and that is exactly the
question you need answered when outputs are dead.

There is no in-band acknowledgement of the configuration, so verify it indirectly, from the
feedback in the drive loop:

* The module answers `send_values()` with a header-valid frame, but `dutyCycle[]` stays 0
  for a channel you are commanding → the module is communicating fine but was never
  configured (or the configuration was rejected). Work down the diagnosis table below.
* `temperature` and `supply` are plausible and change over time → the module is alive; the
  problem is in the configuration, not in the bus.

Do **not** react to a suspected non-delivery by re-uploading the configuration: that is
itself harmful (see trap 2). Reset the module and run the bring-up once, cleanly.

---

## Diagnosing a dead output

Log these five values once per second from the drive loop. They partition the problem.

| Observation | Conclusion |
|---|---|
| `configuration[ch]` has a **high nibble of 0** | That channel was never configured — a `configure_channel()` call was rejected and its return value ignored. See trap 0. A half bridge at frequency nibble 4 must read `0x24`. |
| `res != 0` on `send_values` | No valid exchange. Everything else in the struct is **stale** — never act on it. Check the SPI settings (trap 6) and the loop period. |
| `duty` ≈ commanded value | The module **is** switching. The fault is downstream: wiring, connector, load, or the output stage. Software is exonerated. |
| `duty == 0` while commanding 1000 | The module is not switching. Either the channel configuration never landed (verify per above) or the function/frequency encoding is wrong (traps 1 and 3). |
| `supply` low or 0 | The module measures its own power rail. This is the classic "communicates fine, never drives". |
| `errorCode != 0` | The module reports a fault itself; decode against the `OUTPUTERR_*` macros in `GO_module_output.h`. |

`errorCode == 0x08000000` (`OUTPUTERR_COMMUNICATION_TIMEOUT`) deserves a special mention:
it is the module's own 400 ms watchdog. If you can make that bit appear by pausing the loop
and disappear by resuming it, you have **proved that the module parses and checksum-validates
the frames you send** — which settles the "does my MOSI even arrive?" question without a
logic analyzer.

Two more probes worth knowing:

* Build with `-DDEBUG=1`. `GO_communication_modules_initialize()` then dumps the raw
  bootloader/firmware frames. Healthy is `9, 45, 9, …`; a shifted frame means trap 6.
* A healthy 61-byte transfer is exactly 488 SCK pulses with **no stray edge near
  CS-assert**. That is the analyzer signature for trap 6.

---

## The traps

### 0. A rejected `configure_channel()` leaves the output floating — and nobody notices

`GO_module_output_6ch_configure_channel()` and `..._configure_frequency()` validate their
arguments **first** and return `-EINVAL` *without writing `configuration[]`*. The function
nibble then stays 0, which means **floating output**, and everything downstream still reports
success: `configuration()` ships the half-built config and returns 0, `send_values()`
exchanges valid frames, the module reports no error. The only trace is one `err()` line in a
busy RTT log. The result is indistinguishable from a hardware fault.

**Always check the return values, and log the resulting `configuration[]` bytes.** A half
bridge at frequency nibble 4 must read `0x24`; a high nibble of 0 means the call was
rejected.

The usual trigger is `currentMax > CURRENTMAXMAX` (4000). Note that the Linux reference
programs **4500** on this very hardware — by writing `outputModule.currentMax[ch]` directly
and never calling the validating helper at all. So the library cap is a library limitation,
not necessarily the module's rating. If you need a higher limit:

```c
/* configure the FUNCTION with a value that passes validation ... */
GO_module_output_6ch_configure_channel(&m, ch, OUTPUTFUNC_6CH_HALFBRIDGE,
                                       CURRENTMAXMAX, 0, 0);
/* ... then set the real limit directly, like the Linux reference does. */
m.currentMax[ch] = 4500u;
```

Only do that when the module's per-channel rating genuinely allows it.

### 1. The frequency and function tables do not match the specification

The working Linux application programs, for every channel of every 6-channel output module:

```c
outputModule.configuration[ch] = 4 | (func << 4);   /* half-bridge -> 0x24 */
```

Frequency nibble **4**, including for half-bridge channels. Use that value.

The CodeBase table is off by one against the SPI specification. Spec: `1 = 200 Hz`,
`2 = 500 Hz`, `3 = 1 kHz`, `4 = 2 kHz`, `5 = 5 kHz`, `6 = 10 kHz`. There is no 100 Hz, and
`OUTPUTFREQ_6CH_10KHZ = 7` is outside the spec range altogether. So `OUTPUTFREQ_100HZ`
does not give you 100 Hz, and the macro names cannot be used as documentation. A frequency
value the module firmware does not recognise can leave a duty-controlled channel silent
**without any error bit** — which is indistinguishable from a hardware fault at a glance.

The same suspicion applies to `OUTPUTFUNC_DISABLED = 1`: the Linux shutdown path writes
`configuration[ch] = 0` to make outputs float, which suggests 0 is the real "unused"
encoding and the function table may be shifted as well. Unconfirmed, but if channels behave
one function off from what you asked for, start here.

### 2. Configure exactly once — never in a retry loop, never again at runtime

Repeated reconfiguration breaks the module's SPI synchronisation. Measured result:
`errorCode` went to `0x30000000` (`INIT_FAILURE | NOT_COMMUNICATING`) and
`supply`/`temperature` froze at their last valid values.

This is easy to do by accident: a helper that both builds *and* uploads the configuration,
called just before an upload retry loop, sends the full two-message burst up to six times
within a few milliseconds. Keep build and upload separate, and call upload once.

The corollary is that there is no configuration-based recovery path. If the module stops
answering mid-run, just keep sending values: its own watchdog puts the outputs in a safe
state, and the exchange resumes by itself when it answers again. Do **not** re-reset and
re-configure — a powered, running output module cannot be driven back into its bootloader
by a warm reset anyway, so a mid-drive reset+detect fails forever *and* drops the load.

### 3. `peak_current` / `peak_time` belong to function 7 only

They are stored in `channelParameter1` / `channelParameter2`, which are **unions** that
mean `fastLoopGain` / `fastLoopBasicDuty` for other functions. Pass `0, 0` for anything
other than `OUTPUTFUNC_6CH_PEAKANDHOLD`. (The Linux reference does pass 1000/500 for
half-bridge channels and still works, so this is not the difference between driving and not
driving — but 0/0 is what the specification asks for.)

### 4. Validate the feedback header, not just the checksum

The 6-channel feedback parser historically checked only the checksum, unlike the 10-channel
one. Any frame with a valid checksum — an echo, or the answer to a *previous* transaction —
was then decoded as feedback, so temperature, current and duty were read from the wrong
bytes while everything looked healthy. Expected V2 header:

```
[0] slot   [1] 43 (length-1)   [2] 2 (MOD->COM)   [3] 22 (output 6ch)   [4] 4 (feedback)   [5] 1 (index)
```

`GO_module_output_send_values()` now enforces this for V2 and returns `-EBADMSG` otherwise.

### 5. The firmware stamp selects the protocol version

`configuration()` derives `sw_version` from the registered stamp
(`moduleOccupancy[slot][4..6]`), and `sw_version >= 2.0.0` selects the V2 SPI protocol.
A module registered with a *made-up* stamp — e.g. by an app-level "adopt" fallback that
bypasses bootloader validation — may leave you speaking the wrong protocol version at it
for the entire session, with no error anywhere. Prefer the cold bootloader detect, and treat
any hard-coded stamp as a diagnostic dead end.

### 6. S1 SPI needs both `MasterKeepIOState = ENABLE` and `NSSPMode = DISABLE`

With the CubeMX defaults the STM32H5 SPI emits a spurious clock edge at every `SPE` enable,
which shifts **every** module frame left by one bit — the bootloader signature `9,45,9`
reads back as `18,18,90`, module id `20,20,2` as `40,40,4` — and detection fails for every
module with a misleading "contested slot" error. Both settings are required together. See
AGENTS.md rule 14 and `code/iot/Core/Src/spi.c`.

### 7. Practical S1 details that cost time

* **Task stack ≥ 2048 bytes.** `printf`/`dbg()` frames in the detect path overflow 1024 and
  hard-fault the MCU in a way that looks like a module problem.
* **All module I/O runs after `osKernelStart()`.** Module SPI blocks on an RTOS semaphore,
  so `*_init()` may only spawn the task — never talk to a module.
* **One shared mutex per SPI bus.** The CodeBase module drivers have no internal bus lock
  and use shared static buffers. Additionally, let other slots wait until the first slot has
  finished its bootloader handshake, so that fragile exchange runs uninterrupted.
* **A module keeps its state across an MCU reflash.** A J-Link reflash resets the MCU, not
  the module supply, so a module can stay in its application and return frames that are not
  `9,45,9`. Power-cycle the module for a clean reset before concluding anything about
  detection.
* **`make clean` after changing `DEFINES`.** The object rules do not depend on the Makefile,
  so a `-DDEBUG=1` change alone does not trigger a rebuild.

---

## Related

* `output_module_6ch_s1.c` — the executable procedure.
* `../output_module_6ch/output_module_6ch.c` — the Linux example (boolean + PWM channels).
* `../../AGENTS.md` — hard rules 2, 4, 8, 12, 13, 14, 15, 16, 17 and the pitfalls list.
* `../../code/modules/GO_module_output.h` — struct, function/frequency macros, `OUTPUTERR_*` bits.
