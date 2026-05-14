# GOcontroll-CodeBase

C library for programming **GOcontroll Moduline controllers**. Two build
flavours share the same public API:

| Define              | Target                                   |
|---------------------|------------------------------------------|
| `GOCONTROLL_LINUX`  | Moduline L4 / Moduline M1 (IMX8 ARM64) |
| `GOCONTROLL_IOT`    | Moduline S1 (STM32H5 + FreeRTOS)         |

> The S1 platform is selected by the legacy `GOCONTROLL_IOT` define and lives
> under `code/iot/`. Source identifiers are kept for compatibility with the
> previous product naming.

## Repo layout

```
code/        Public C library — board, communication, modules, XCP, S1 subtree
examples/    Self-contained main()s, one per topic (Linux only at present)
lib/         Vendored third-party (IIO) — do not modify
AGENTS.md    Index for AI coding tools (hard rules + conventions)
```

## Quick start

```bash
make led_blink              # build the smallest example
sudo build/app              # run on target
```

For a working module program, see
[`examples/input_module_10ch/input_module_10ch.c`](examples/input_module_10ch/input_module_10ch.c)
or
[`examples/input_module_6ch/input_module_6ch.c`](examples/input_module_6ch/input_module_6ch.c)
(the latter has the most thorough inline documentation, including the
required initialisation order).

## Hardware

GOcontroll controllers expose pluggable modules in physical slots.
Supported module types:

- **Input modules** — 6-channel, 10-channel, and 4–20 mA variants.
- **Output modules** — 6-channel and 10-channel high-side drivers.
- **Bridge modules** — 2-channel H-bridge / half-bridge drivers.

Module communication runs at **10 ms cycle minimum**; output modules trip a
fail-safe watchdog if no frame arrives within **400 ms**.

## Documentation

- [`AGENTS.md`](AGENTS.md) — index for AI coding tools: hard rules,
  pitfalls, and conventions.
- [`examples/examples.md`](examples/examples.md) — one-line description
  per example.
- File-level doc-blocks at the top of each `code/**/*.h` header and each
  example `.c` file.

## License

Library code: see header doc-blocks (MIT-style). Vendored libraries under
`lib/` and `code/iot/Drivers/` retain their original licenses.
