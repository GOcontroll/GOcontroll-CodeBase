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
| `stack_check.py` | Statische stack-analyse op basis van de `.su`-bestanden van de compiler (zie hieronder). |
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
`make gdbserver` · `make debug` · `make rtt` · `make size` · `make stack-check` · `make help`

## Stack-analyse (`make stack-check`)

### Achtergrond

FreeRTOS-taken op de Cortex-M33 (STM32H573) kunnen crashen als de gedeclareerde
`stack_size` niet groot genoeg is. Dat leidt niet tot een compileerfout, maar tot een
stille geheugenkorruptie of een `vApplicationStackOverflowHook`-reset op de hardware.
`make stack-check` maakt dit probleem zichtbaar vóór het flashen.

### Hoe het werkt

`s1.mk` bevat al de compiler-flag `-fstack-usage`. Na elke compilatie schrijft GCC
een `.su`-bestand naast elk `.o`-bestand in `build/`. Elk `.su`-bestand bevat per
functie de grootte van het **lokale stack-frame** (alleen de lokale variabelen van die
functie — niet de functies die zij aanroept).

`stack_check.py` leest alle `.su`-bestanden in `build/`, sorteert op framegrootte en
controleert specifiek alle taak-functies (naam eindigt op `_task`).

### Drempelwaarden

Op de Cortex-M33 telt bovenop het eigen frame van een taak altijd extra overhead:

| Post | Bytes |
|---|---|
| FreeRTOS context-switch (registers opslaan) | ~68 |
| `snprintf` met `double`-argument via soft-float libgcc | ~400 |
| MQTT send-keten (`mqtt_pub` → `SendFrame`) | ~88 |

De chip heeft alleen een **single-precision FPU**. Elk `double`-argument in
`snprintf("%.6f", ...)` valt terug op software-drijvendepuntrekening in libgcc en
gebruikt daardoor ~400 bytes stack — veel meer dan verwacht.

| Verdict | Framegrootte | Betekenis |
|---|---|---|
| `ok` | ≤ 200 B | Veilig voor een 512-byte stack (zonder `snprintf double`) |
| `WARN` | 200–400 B | Controleer de gedeclareerde `stack_size` handmatig |
| `ERROR` | > 400 B | Vrijwel zeker overflow als `snprintf double` gebruikt wordt op een 1024-byte stack |

### Gebruik

```bash
make build        # genereer eerst de .su-bestanden
make stack-check  # analyseer ze
```

Drempelwaarden zijn aanpasbaar:

```bash
python3 GOcontroll-CodeBase/make/stack_check.py build --task-warn 150 --task-error 300
```

### Typische aanpak bij een WARN/ERROR

1. Controleer welke lokale variabelen het grote frame veroorzaken (`char payload[256]`,
   `char topic[64]`, etc.).
2. Declareer die buffers als `static` in de taakfunctie — ze verhuizen dan van de
   FreeRTOS-taskstack naar BSS en kosten geen heap meer.
3. Verklein de bijbehorende `stack_size` navenant.
4. Hercompileer en draai `make stack-check` opnieuw tot alle taken `ok` zijn.

> **Vereiste:** Python 3 in PATH. Alleen `make stack-check` heeft Python nodig;
> `make build`, `make flash` en alle overige doelen werken zonder Python.
