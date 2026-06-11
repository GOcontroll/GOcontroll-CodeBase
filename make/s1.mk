# =============================================================================
# GOcontroll Moduline S1 — generieke build-laag (include vanuit een dunne Makefile)
# =============================================================================
# Toolchain : arm-none-eabi-gcc 14.2 (GNU Arm Embedded)
# Programmer: SEGGER J-Link CLI (JLink.exe, JLinkGDBServerCL.exe)
# Doel-MCU  : STM32H573RI (Cortex-M33 + single-precision FPU) + FreeRTOS
#
# Dit bestand bevat ALLE generieke, stabiele build-regels voor de S1. Een
# applicatie-repo levert enkel een DUNNE root-Makefile die deze variabelen zet en
# dit bestand include't:
#
#   PROJECT     := <slug>                            (application-builder marker-blok)
#   UNIT_DIRS   := application/modem application/...  (application-builder marker-blok)
#   CODEBASE    := GOcontroll-CodeBase               (pad naar deze submodule op de repo-root)
#   APP_DIR     := application                       (gegenereerde C)
#   BUILD_DIR   := build                             (objecten/intermediates)
#   DEPLOY_DIR  := deploy                            (binaries)
#
#   include $(CODEBASE)/make/s1.mk
#
# Targets:
#   make            Build everything (default).
#   make build      Same as above.
#   make clean      Remove build/ and deploy/ directories.
#   make flash      Flash via J-Link (SWD).
#   make erase      Mass-erase the STM32 via J-Link.
#   make reset      Reset target via J-Link.
#   make gdbserver  Start JLinkGDBServer in foreground (port 2331).
#   make debug      Start arm-none-eabi-gdb attached to gdbserver and loaded.
#   make rtt        Open JLinkRTTClient (after a build/flash with RTT enabled).
#   make size       Print section sizes of the ELF.
# =============================================================================

# ----- Defaults (overschrijfbaar door de dunne Makefile) ---------------------
PROJECT    ?= app
CODEBASE   ?= GOcontroll-CodeBase
APP_DIR    ?= application
BUILD_DIR  ?= build
DEPLOY_DIR ?= deploy
UNIT_DIRS  ?=

# ----- Shell (Windows: use Git Bash so mkdir -p / rm -rf work) ---------------
SHELL := C:/PROGRA~1/Git/usr/bin/bash.exe

TARGET_MCU := STM32H573xx
CHIP_JLINK := STM32H573RI

# ----- Toolchain -------------------------------------------------------------
CROSS_COMPILE ?= arm-none-eabi-
CC      := $(CROSS_COMPILE)gcc
CXX     := $(CROSS_COMPILE)g++
AS      := $(CROSS_COMPILE)gcc -x assembler-with-cpp
LD      := $(CROSS_COMPILE)gcc
OBJCOPY := $(CROSS_COMPILE)objcopy
SIZE    := $(CROSS_COMPILE)size
GDB     := $(CROSS_COMPILE)gdb

# J-Link CLI - installed under "Program Files\SEGGER\JLink_Vxxx\".
# Override on the command line if your install path or version differs:
#   make flash JLINK_DIR="C:/Program Files/SEGGER/JLink_V880"
JLINK_DIR    ?= C:/Program Files/SEGGER/JLink_V876
JLINK        := "$(JLINK_DIR)/JLink.exe"
JLINK_GDB    := "$(JLINK_DIR)/JLinkGDBServerCL.exe"
JLINK_RTT    := "$(JLINK_DIR)/JLinkRTTClient.exe"
JLINK_IF     ?= SWD
JLINK_SPEED  ?= 4000

# ----- Linker script & startup ----------------------------------------------
LINKER_SCRIPT := $(CODEBASE)/code/iot/STM32H573RITX_FLASH.ld
STARTUP_SRC   := $(CODEBASE)/code/iot/Core/Startup/startup_stm32h573ritx.s

# ----- CPU / FPU flags -------------------------------------------------------
# STM32H573 = Cortex-M33 + single-precision FPU (fpv5-sp-d16).
# We use the FreeRTOS ARM_CM33_NTZ (non-TrustZone) port, so no -mcmse.
CPU_FLAGS := -mcpu=cortex-m33 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard

# ----- Defines ---------------------------------------------------------------
# GOCONTROLL_IOT selects the S1 platform inside the codebase.
DEFINES := \
	-D$(TARGET_MCU) \
	-DUSE_HAL_DRIVER \
	-DGOCONTROLL_IOT \
	-DUSE_FULL_LL_DRIVER=0

# ----- Include paths ---------------------------------------------------------
# Per-unit -I entries worden afgeleid uit UNIT_DIRS (door de dunne Makefile gezet).
UNIT_INCLUDES := $(foreach d,$(UNIT_DIRS),-I$(d))
INCLUDES = \
	-I$(APP_DIR)/app/src \
	$(UNIT_INCLUDES) \
	-I$(CODEBASE)/code \
	-I$(CODEBASE)/code/modules \
	-I$(CODEBASE)/code/iot/Core/Inc \
	-I$(CODEBASE)/code/iot/Drivers/CMSIS/Include \
	-I$(CODEBASE)/code/iot/Drivers/CMSIS/Device/ST/STM32H5xx/Include \
	-I$(CODEBASE)/code/iot/Drivers/STM32H5xx_HAL_Driver/Inc \
	-I$(CODEBASE)/code/iot/Drivers/STM32H5xx_HAL_Driver/Inc/Legacy \
	-I$(CODEBASE)/code/iot/Drivers/segger/Inc \
	-I$(CODEBASE)/code/iot/Middlewares/Third_Party/FreeRTOS/Source/include \
	-I$(CODEBASE)/code/iot/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 \
	-I$(CODEBASE)/code/iot/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure

# ----- Sources ---------------------------------------------------------------
# Application sources (every .c under $(APP_DIR)/app/src/).
APP_SOURCES := $(wildcard $(APP_DIR)/app/src/*.c)

# Unit sources - elke unit-map (uit UNIT_DIRS) levert zijn src/*.c aan.
UNIT_SOURCES := $(foreach d,$(UNIT_DIRS),$(wildcard $(d)/src/*.c))

# GOcontroll public codebase - root .c files. We compile them all; each has an
# internal GOCONTROLL_IOT branch (or is platform-agnostic).
CODEBASE_ROOT_SOURCES := \
	$(CODEBASE)/code/GO_board.c \
	$(CODEBASE)/code/GO_communication_can.c \
	$(CODEBASE)/code/GO_communication_esp.c \
	$(CODEBASE)/code/GO_communication_modules.c \
	$(CODEBASE)/code/GO_communication_mqtt.c \
	$(CODEBASE)/code/GO_controller_info.c \
	$(CODEBASE)/code/GO_fault.c \
	$(CODEBASE)/code/GO_gps.c \
	$(CODEBASE)/code/GO_memory.c \
	$(CODEBASE)/code/modules/GO_module_input.c \
	$(CODEBASE)/code/print.c

# STM32H5 platform glue (peripheral inits, IRQ vectors, system clock, etc.).
IOT_CORE_SOURCES := $(wildcard $(CODEBASE)/code/iot/Core/Src/*.c)

# STM32H5 HAL drivers - every Src/*.c is compiled; each is internally gated
# by HAL_<MODULE>_MODULE_ENABLED defines in stm32h5xx_hal_conf.h, so unused
# modules emit nothing.
HAL_SOURCES := $(wildcard $(CODEBASE)/code/iot/Drivers/STM32H5xx_HAL_Driver/Src/*.c)

# SEGGER RTT for SEGGER_RTT_printf() / info() output over J-Link.
SEGGER_SOURCES := $(wildcard $(CODEBASE)/code/iot/Drivers/segger/Src/*.c)

# FreeRTOS V10.6.2 - kernel + ARM_CM33_NTZ port + heap_4 + CMSIS_RTOS_V2.
FREERTOS_DIR := $(CODEBASE)/code/iot/Middlewares/Third_Party/FreeRTOS/Source
FREERTOS_SOURCES := \
	$(FREERTOS_DIR)/tasks.c \
	$(FREERTOS_DIR)/list.c \
	$(FREERTOS_DIR)/queue.c \
	$(FREERTOS_DIR)/timers.c \
	$(FREERTOS_DIR)/event_groups.c \
	$(FREERTOS_DIR)/stream_buffer.c \
	$(FREERTOS_DIR)/croutine.c \
	$(FREERTOS_DIR)/CMSIS_RTOS_V2/cmsis_os2.c \
	$(FREERTOS_DIR)/portable/GCC/ARM_CM33_NTZ/non_secure/port.c \
	$(FREERTOS_DIR)/portable/GCC/ARM_CM33_NTZ/non_secure/portasm.c \
	$(FREERTOS_DIR)/portable/MemMang/heap_4.c

C_SOURCES := \
	$(APP_SOURCES) \
	$(UNIT_SOURCES) \
	$(CODEBASE_ROOT_SOURCES) \
	$(IOT_CORE_SOURCES) \
	$(HAL_SOURCES) \
	$(SEGGER_SOURCES) \
	$(FREERTOS_SOURCES)

ASM_SOURCES := $(STARTUP_SRC)

# ----- Object files (mirror source tree under build/) ------------------------
OBJECTS := \
	$(addprefix $(BUILD_DIR)/, $(C_SOURCES:.c=.o)) \
	$(addprefix $(BUILD_DIR)/, $(ASM_SOURCES:.s=.o))

DEPS := $(OBJECTS:.o=.d)

# ----- Compiler flags --------------------------------------------------------
OPT      ?= -Og
CSTD     := -std=gnu11
WARNINGS := -Wall -Wextra -Wshadow -Wundef
COMMON_CFLAGS := \
	$(CPU_FLAGS) \
	$(DEFINES) \
	$(INCLUDES) \
	$(OPT) \
	-g3 -gdwarf-2 \
	-ffunction-sections -fdata-sections \
	-fno-common \
	-fstack-usage \
	-MMD -MP

CFLAGS   := $(COMMON_CFLAGS) $(CSTD) $(WARNINGS)
ASFLAGS  := $(CPU_FLAGS) -g3 -gdwarf-2 -MMD -MP

# ----- Linker flags ----------------------------------------------------------
LDFLAGS := \
	$(CPU_FLAGS) \
	-T$(LINKER_SCRIPT) \
	-specs=nano.specs \
	-specs=nosys.specs \
	-u _printf_float \
	-Wl,--gc-sections \
	-Wl,--print-memory-usage \
	-Wl,-Map=$(DEPLOY_DIR)/$(PROJECT).map \
	-Wl,--no-warn-rwx-segments

LDLIBS := -lc -lm -lnosys

# =============================================================================
# Build rules
# =============================================================================
ELF := $(DEPLOY_DIR)/$(PROJECT).elf
HEX := $(DEPLOY_DIR)/$(PROJECT).hex
BIN := $(DEPLOY_DIR)/$(PROJECT).bin

# Project-independent flash artifact. The J-Link script in $(CODEBASE)/make/jlink/
# loads this fixed name, so a PROJECT rename never breaks `make flash`. Always a
# copy of the current $(HEX).
FIRMWARE := $(DEPLOY_DIR)/firmware.hex

.PHONY: all build clean flash erase reset gdbserver debug rtt size stack-check help
.DEFAULT_GOAL := all

all: build
build: $(ELF) $(HEX) $(BIN) $(FIRMWARE) size

# Compile C
$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo "  CC   $<"
	@$(CC) $(CFLAGS) -c $< -o $@

# Assemble startup
$(BUILD_DIR)/%.o: %.s
	@mkdir -p $(dir $@)
	@echo "  AS   $<"
	@$(AS) $(ASFLAGS) -c $< -o $@

# Link
$(ELF): $(OBJECTS) $(LINKER_SCRIPT)
	@mkdir -p $(dir $@)
	@echo "  LD   $@"
	@$(LD) $(OBJECTS) $(LDFLAGS) $(LDLIBS) -o $@

# Binary outputs
$(HEX): $(ELF)
	@echo "  HEX  $@"
	@$(OBJCOPY) -O ihex $< $@

$(BIN): $(ELF)
	@echo "  BIN  $@"
	@$(OBJCOPY) -O binary $< $@

# Stable-named copy for the J-Link flash script.
$(FIRMWARE): $(HEX)
	@echo "  CP   $@"
	@cp $< $@

size: $(ELF)
	@echo "---"
	@$(SIZE) $(ELF)

clean:
	@echo "  RM   $(BUILD_DIR) $(DEPLOY_DIR)"
	@rm -rf $(BUILD_DIR) $(DEPLOY_DIR)

# =============================================================================
# Flash / debug via J-Link CLI
# =============================================================================
JLINK_BASE_OPTS := -Device $(CHIP_JLINK) -If $(JLINK_IF) -Speed $(JLINK_SPEED) -AutoConnect 1

flash: $(FIRMWARE)
	@echo "  FLASH $(FIRMWARE)  (via J-Link, $(CHIP_JLINK))"
	@$(JLINK) $(JLINK_BASE_OPTS) -CommanderScript $(CODEBASE)/make/jlink/flash.jlink

erase:
	@echo "  ERASE $(CHIP_JLINK)"
	@$(JLINK) $(JLINK_BASE_OPTS) -CommanderScript $(CODEBASE)/make/jlink/erase.jlink

reset:
	@echo "  RESET $(CHIP_JLINK)"
	@$(JLINK) $(JLINK_BASE_OPTS) -CommanderScript $(CODEBASE)/make/jlink/reset.jlink

gdbserver:
	@echo "  GDB-SERVER  $(CHIP_JLINK)  (port 2331, RTT on)"
	@$(JLINK_GDB) -select USB -device $(CHIP_JLINK) -if $(JLINK_IF) -speed $(JLINK_SPEED) -port 2331 -rtos GDBServer/RTOSPlugin_FreeRTOS

# Interactive GDB session. Run `make gdbserver` in a second terminal first.
debug: $(ELF)
	@$(GDB) -ex "target remote :2331" -ex "monitor reset" -ex "load" -ex "monitor reset" $(ELF)

rtt:
	@echo "  RTT  client (connect to running JLinkGDBServer or after flash)"
	@$(JLINK_RTT)

# Analyse .su files from the last build. Run 'make build' first.
stack-check:
	@python3 $(CODEBASE)/make/stack_check.py $(BUILD_DIR)

help:
	@echo "Targets:  all  clean  flash  erase  reset  gdbserver  debug  rtt  size  stack-check"
	@echo "Vars:     OPT=-Og|-O2|-O0   JLINK_DIR=<path>   JLINK_SPEED=<kHz>"

-include $(DEPS)
