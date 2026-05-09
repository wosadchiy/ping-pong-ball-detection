# ============================================================================
# Makefile — ADuC841 firmware build & flash for macOS / Linux
#
# Replaces the Keil µVision project + WSD.exe pair on Windows.
# Toolchain: SDCC (compiler) + python aduc_flash.py (uploader).
#
# Usage:
#   make                            compile src/ → build/firmware.ihx (default)
#   make all                        same as just `make`
#   make flash                      build (if needed) + serial upload
#   make probe                      just talk to bootloader, print its ID
#   make monitor                    open `screen` to the UART (Ctrl-A K to exit)
#   make ports                      list candidate /dev/cu.* devices
#   make clean                      remove build/
#
# Override on command line, e.g.:
#   make flash PORT=/dev/cu.usbserial-A100VKSF BAUD=9600
# ============================================================================

# ─── tools ──────────────────────────────────────────────────────────────────
SDCC      ?= sdcc
PACKIHX   ?= packihx

# Prefer the project's virtualenv python (it has pyserial). Fall back to a
# system python3 — user is then expected to `pip install pyserial` themselves.
VENV_PY   := $(abspath $(CURDIR)/../../venv/bin/python)
PYTHON    ?= $(if $(wildcard $(VENV_PY)),$(VENV_PY),python3)
FLASHER   ?= $(PYTHON) tools/aduc_flash.py

# ─── target ─────────────────────────────────────────────────────────────────
NAME      ?= firmware
SRC_DIR   := src
BUILD_DIR := build

SOURCES   := $(SRC_DIR)/main.c
HEADERS   := $(SRC_DIR)/aduc841_sfr.h
HEX       := $(BUILD_DIR)/$(NAME).ihx

# ─── ADuC841 memory map ─────────────────────────────────────────────────────
# 62 KB code Flash (0x0000..0xF7FF), 256 + 2048 RAM, code starts at 0x0000.
CODE_SIZE := 0xF800
XRAM_SIZE := 2048
SDCC_FLAGS := -mmcs51 \
              --model-small \
              --code-size $(CODE_SIZE) \
              --xram-size $(XRAM_SIZE) \
              --xram-loc  0x0000 \
              --opt-code-size

# ─── serial port autodetection (with manual override) ──────────────────────
# Ищем ТОЛЬКО FTDI-плату ADuC841 (серийники начинаются с A/F/B), а не любой
# usbserial — иначе на машине с подключённой Arduino Nano (CH340 с числовым
# серийником вроде /dev/cu.usbserial-14230) glob `usbserial-*` зацепил бы её
# первой и `make flash` залил бы прошивку 8052 в ATmega328 → bootloader
# молча не отвечает. Префиксы A/F/B соответствуют FT232R / FT232H / FT2232
# заводским серийникам.
#
# Override priority:  ADUC_PORT env var → PORT=... arg → glob.
PORT      ?= $(or $(ADUC_PORT),$(shell ls /dev/cu.usbserial-A* /dev/cu.usbserial-F* /dev/cu.usbserial-B* 2>/dev/null | head -1))

# BAUD     = baud для общения с БУТЛОАДЕРОМ (flash/probe/listen/scan).
#            Жёстко зашит в маске — НЕ менять.
# APP_BAUD = baud для общения с НАШЕЙ ПРОШИВКОЙ (monitor/sine). Должен
#            совпадать с UART_BAUD в src/main.c.
BAUD      ?= 9600
APP_BAUD  ?= 115200

# ─── targets ────────────────────────────────────────────────────────────────

.DEFAULT_GOAL := all
.PHONY: all flash probe listen scan monitor monitor-screen ports sine clean help

help:
	@echo "Targets:"
	@echo "  all        compile firmware → $(HEX)  (default)"
	@echo "  flash      build + serial upload to ADuC841"
	@echo "  probe      interrogate bootloader, print ID"
	@echo "  listen     passive: dump RX bytes for 5 s — press RESET while it runs"
	@echo "  scan       try common baud rates (11.0592 / 16 / 20 MHz xtals)"
	@echo "  monitor    open serial terminal (miniterm, with local echo)"
	@echo "  monitor-screen  same but via 'screen' (no local echo)"
	@echo "  ports      list candidate /dev/cu.* serial devices"
	@echo "  sine       host-side sine sweep on dx,dy → see two channels on scope"
	@echo "  clean      rm -rf build/"
	@echo ""
	@echo "Vars (override on command line):"
	@echo "  PORT     = $(PORT)"
	@echo "  BAUD     = $(BAUD)        (bootloader: flash/probe/listen/scan)"
	@echo "  APP_BAUD = $(APP_BAUD)   (firmware:   monitor/sine — = UART_BAUD in main.c)"
	@echo "  SDCC     = $(SDCC)"
	@echo "  PYTHON   = $(PYTHON)"

all: $(HEX)

$(HEX): $(SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	@echo "→ compiling $(SOURCES)"
	$(SDCC) $(SDCC_FLAGS) -o $(BUILD_DIR)/ $(SOURCES)
	@echo "→ packing Intel-HEX"
	$(PACKIHX) $(BUILD_DIR)/main.ihx > $(HEX)
	@echo "→ done: $(HEX) ($$(wc -c < $(HEX)) bytes)"

flash: all _check_port
	@echo "→ flashing $(HEX) to $(PORT) @ $(BAUD) baud"
	$(FLASHER) --port $(PORT) --baud $(BAUD) --hex $(HEX)

probe: _check_port
	$(FLASHER) --port $(PORT) --baud $(BAUD) --probe

listen: _check_port
	$(FLASHER) --port $(PORT) --baud $(BAUD) --listen 5

scan: _check_port
	$(FLASHER) --port $(PORT) --scan-baud

monitor: _check_port
	@echo "→ opening serial monitor on $(PORT) @ $(APP_BAUD) baud (firmware)"
	@echo "  exit: Ctrl-]   |   toggle echo: Ctrl-T E   |   help: Ctrl-T Ctrl-H"
	$(PYTHON) -m serial.tools.miniterm \
		--echo \
		--eol LF \
		--rts 0 --dtr 0 \
		$(PORT) $(APP_BAUD)

# Старый screen-вариант — если по какой-то причине miniterm не подходит.
# `screen` НЕ показывает локальный ввод (нет echo) и шлёт только CR на Enter
# — наша прошивка теперь это нормально обрабатывает (см. feed() в main.c).
monitor-screen: _check_port
	@echo "→ opening screen on $(PORT) @ $(APP_BAUD) baud (firmware)"
	@echo "  exit: Ctrl-A K Y"
	screen $(PORT) $(APP_BAUD)

# Sine-sweep host generator — Python feeds 30 Hz of dx,dy values to the
# already-flashed firmware. Override RATE/AMP/FREQ on the command line:
#   make sine RATE=60 AMP=512 FREQ=1.0
RATE ?= 30
AMP  ?= 320
FREQ ?= 0.5
sine: _check_port
	@echo "→ streaming sine to $(PORT) @ $(APP_BAUD) baud (rate=$(RATE)Hz amp=$(AMP)px freq=$(FREQ)Hz)"
	$(PYTHON) tools/aduc_send_test.py --port $(PORT) --baud $(APP_BAUD) \
		--rate $(RATE) --amp $(AMP) --freq $(FREQ)

ports:
	@echo "Available serial devices:"
	@ls -1 /dev/cu.* 2>/dev/null | grep -v Bluetooth || echo "  (none found)"

clean:
	rm -rf $(BUILD_DIR)

# ─── internal sanity checks ────────────────────────────────────────────────

_check_port:
	@if [ -z "$(PORT)" ]; then \
		echo "error: no serial port found. Specify with PORT=/dev/cu.xxx"; \
		echo "       (run 'make ports' to see candidates)"; \
		exit 1; \
	fi
