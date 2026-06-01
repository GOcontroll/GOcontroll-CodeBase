# make/ — generieke S1 build-laag

Dit is de **platform-generieke** build-laag voor een GOcontroll Moduline S1
(STM32H573RI) applicatie. Hij leeft bewust in CodeBase, naast de HAL die hij
compileert: de paden in `s1.mk` verwijzen rechtstreeks naar de interne layout van
`code/iot/...` (linker-script, startup, HAL, FreeRTOS). Zo versioneren build-laag en
HAL samen — geen cross-repo drift.

## Inhoud

| Bestand | Doel |
|---|---|
| `s1.mk` | Alle generieke build-regels (toolchain, flags, sources, link, flash/debug-targets). |
| `jlink/flash.jlink` | `make flash` — reset, halt, `loadfile deploy/firmware.hex`, reset+run. |
| `jlink/erase.jlink` | `make erase` — mass-erase. |
| `jlink/reset.jlink` | `make reset` — reset + run zonder flashen. |

## Gebruik vanuit een applicatie-repo

Een applicatie-repo levert enkel een **dunne** root-`Makefile` die de project-
variabelen zet en deze laag include't:

```makefile
# === BEGIN GENERATED PROJECT (do not edit — application-builder) ===
PROJECT := <slug>
# === END GENERATED PROJECT ===
# === BEGIN GENERATED UNITS (do not edit — application-builder) ===
UNIT_DIRS := application/modem application/can1
# === END GENERATED UNITS ===
CODEBASE   := GOcontroll-CodeBase
APP_DIR    := application
BUILD_DIR  := build
DEPLOY_DIR := deploy
include $(CODEBASE)/make/s1.mk
```

De `application-builder` skill beheert alleen de twee marker-blokken
(`PROJECT`, `UNIT_DIRS`); de rest is stabiel.

## Vereisten

- `arm-none-eabi-gcc` (GNU Arm Embedded) in PATH.
- SEGGER J-Link CLI — pad via `JLINK_DIR` (default `C:/Program Files/SEGGER/JLink_V876`,
  override op de commandline: `make flash JLINK_DIR="C:/Program Files/SEGGER/JLink_V880"`).
- Windows: Git Bash (voor `mkdir -p` / `rm -rf` in de recepten).

## Doelen

`make` · `make build` · `make clean` · `make flash` · `make erase` · `make reset` ·
`make gdbserver` · `make debug` · `make rtt` · `make size` · `make help`
