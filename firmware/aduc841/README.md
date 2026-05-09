# ADuC841 firmware — macOS / Linux toolchain

Замена Windows-инструментов **Keil µVision** + **WSD.exe** на нативный
open-source стек для macOS:

| Было (Windows) | Стало (macOS / Linux) |
| --- | --- |
| Keil µVision C51 | [SDCC](https://sdcc.sourceforge.net/) (open-source) |
| Keil A51 ассемблер | SDAS (входит в SDCC) |
| WSD.exe | `tools/aduc_flash.py` (Python, ~270 строк, реализует AN-1074) |
| ProgrammerStudio (отладка) | UART `printf` + `screen` |

Никакого Windows / VM / Wine не нужно — только Cursor + терминал.

## Установка

```bash
# 1. Один раз — поставить SDCC и Python-зависимости
brew install sdcc          # ~20 МБ, всё включено: cc + as + ld + packihx
pip install pyserial       # уже есть в venv проекта
```

Если `brew install sdcc` ругается на права `/opt/homebrew`:

```bash
sudo chown -R $(whoami) /opt/homebrew
brew install sdcc
```

Проверить:

```bash
sdcc --version       # должна быть SDCC 4.x
packihx --help       # должен запуститься без ошибки
```

## Структура

```
firmware/aduc841/
├── Makefile               build/flash/monitor/sine цели
├── README.md              этот файл
├── src/
│   ├── main.c             прошивка: парсер 'dx,dy\n' → DAC0/DAC1
│   └── aduc841_sfr.h      SFR-определения (DAC/ADC/T3/PLL) для SDCC
├── examples/
│   └── smoke_main.c       старая smoke-версия (UART echo + DAC ramp + LED)
└── tools/
    ├── aduc_flash.py      загрузчик по протоколу AN-1074
    └── aduc_send_test.py  host-генератор синусоидального dx,dy → проверка канала
```

Чтобы вернуться на smoke-test (например, после ремонта платы):

```bash
cp examples/smoke_main.c src/main.c
make clean && make flash
```

## Workflow

### 1. Собрать

```bash
cd firmware/aduc841
make build
```

Результат: `build/firmware.ihx` — Intel HEX, готовый к заливке.

### 2. Перевести плату в режим загрузчика

ADuC841 заходит в bootloader при сбросе с `/PSEN`-пином, подтянутым к земле:

1. Зажать кнопку **BOOT** (или замкнуть джампер на `/PSEN`)
2. Нажать **RESET** (или передёрнуть питание)
3. Отпустить **BOOT**

Чип теперь молчит и слушает UART на 9600 baud (при кварце 11.0592 MHz).

### 3. Прошить

```bash
make flash
```

Что произойдёт:
- Makefile найдёт `/dev/cu.usbserial-*` автоматически.
- `aduc_flash.py` сначала **interrogate** — пошлёт `!Z\0¦` и прочтёт 25-байтный ID-пакет; если плата ответила — увидите её строку идентификации.
- Затем **erase** → **write** → **run**.
- Прошивка стартует, и сразу из неё в UART должна полететь строка `ADuC841 ALIVE\r\n`.

Если порт надо указать вручную (несколько USB-устройств):

```bash
make flash PORT=/dev/cu.usbserial-A100VKSF
```

### 4. Смотреть UART

```bash
make monitor
# выйти: Ctrl-A K Y
```

После ресета чип должен поздороваться:

```
ADuC841 LATENCY MIRROR ready
- format: 'dx,dy\n' signed decimal pixels (+/- 1024)
- DAC0=dx, DAC1=dy, midscale=center (~AVdd/2)
- LED toggles per valid packet
```

Можно прямо в `screen` напечатать `100,-50<Enter>` — DAC0 уйдёт примерно
на **(2048+200) / 4096 × AVdd**, DAC1 — на **(2048−100) / 4096 × AVdd**,
а LED моргнёт. Это значит парсер работает.

### 5. Проверить канал Python → ADuC → DAC

После того как прошивка зашита и стоит на нормальном рантайме (не bootloader),
закройте `screen` (Ctrl-A K Y) и запустите host-генератор:

```bash
make sine
# или с переопределением параметров:
make sine RATE=60 AMP=512 FREQ=1.0
```

На осциллографе должны появиться **две синусоиды** — DAC0 и DAC1, в квадратуре
(сдвиг на четверть периода). Если они есть — путь Python → FTDI → UART → парсер
→ DAC полностью рабочий, и можно подключать к нему детектор шарика.

### 6. Что-то не так?

Самые частые проблемы:

| Симптом | Причина | Что делать |
| --- | --- | --- |
| `make flash` пишет «no serial port found» | плата не подключена / не опозналась | `make ports` — посмотреть список; убедиться что плата в USB |
| `flasher: No response from bootloader` | плата не в режиме загрузчика | повторить «BOOT + RESET» (см. шаг 2) |
| Тот же таймаут, кварц ≠ 11.0592 MHz | bootloader работает на baud, отличном от 9600 | `make flash BAUD=...` (формула: `9600 × XTAL / 11.0592e6`) |
| `make flash` подключился, но NAK на write | прошивка > 62 КБ или адреса вне Flash | проверить `.ihx` — есть ли запись выше 0xF7FF |
| `make monitor` пишет «Cannot exclusive open» | порт занят (Python `main.py` запущен) | закрыть другую программу, использующую порт |
| После flash чип молчит | RESET без `/PSEN` низкого = должен работать. Если не работает — проверить тактирование | замерить кварц, проверить декаплинг |

## Подгонка под вашу плату

Прямо сейчас прошивка предполагает:

- **Кварц 11.0592 MHz** → если у вас 16 MHz / 20 MHz — поправьте `XTAL_HZ` в `src/main.c` и пересчитайте `TH1_RELOAD` (формула в комментарии). Заодно поменяйте `--baud` для флешера.
- **LED на P3.4** → если у вас на другом пине, поменяйте `__sbit __at(0xB4) LED;` в `main.c` (адрес = 0xB0 + bit_number).
- **DAC0 на пине AOUT0** (по даташиту) → у ADuC841 это фиксированный пин, не настраивается.

## Что дальше

Текущий статус (✅ сделано):

- ✅ Smoke-test железа (UART echo + DAC ramp + LED) — `examples/smoke_main.c`.
- ✅ Парсер `dx,dy\n` → DAC0/DAC1 — `src/main.c` (текущая прошивка).
- ✅ Раздельные handler'ы в Python: `ArduinoHandler` (CH340) и `AducHandler`
  (FTDI) — теперь обе платы можно держать в USB одновременно. См.
  `hardware.py` и `platform_utils.py`.
- ✅ Host-генератор синусоиды — `make sine` (= `tools/aduc_send_test.py`).

Ещё впереди:

1. **Замер FTDI latency timer** — loopback TX↔RX тест, чтобы вычесть из
   общей задержки чисто USB-Serial вклад (типично 1-16 ms на FT232R, его
   можно уменьшить через `system_profiler` / `ioreg` → kext-параметры).
2. **Интеграция в `main.py`** — параллельно с отправкой пакета на Arduino
   зеркалить `nx, ny` в `AducHandler.send_dx_dy()`. Смотреть на сцоупе
   DAC0 (камерный сигнал) против стробоскопа тарелки → реальная latency.

## Ссылки

- [AN-1074 — Serial Download Protocol](https://www.analog.com/media/en/technical-documentation/application-notes/AN-1074.pdf) — документация протокола.
- [ADuC841 datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ADUC841_842_843.pdf) — карта SFR, DAC, ADC.
- [SDCC manual](https://sdcc.sourceforge.net/doc/sdccman.pdf) — компилятор.
