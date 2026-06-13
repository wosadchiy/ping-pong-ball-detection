# STM32 Nucleo-F103RB firmware

Замена `camera_control_v2.ino` (Arduino Nano) на STM32 Nucleo-F103RB.

> **Текущий статус — Step 3.0 (SPI).** Прошивка работает SPI-slave: Pi 4
> присылает «ручную» команду (флаг + скорость) бинарным пакетом, STM32
> печатает её в Serial и крутит мотор. Детали и распиновка — в разделе
> [Step 3.0](#step-30--spi-канал-pi-4--stm32-тест-ручного-управления) ниже.
> (Прежний Step 1 — jog по двум кнопкам — описан исторически.)

## Зачем мигрируем

Оригинальный скетч на ATmega328P упирается в:

* 16 МГц 8-битное ядро — мало запаса CPU для параллельных задач (камера/SPI/UART/таймеры);
* регистровый код Timer1/Timer2, который намертво приколочен к AVR;
* UART-only канал к Pi через CDC-VCP — большая latency и не full-duplex.

STM32F103RB на 72 МГц с DMA, hardware-SPI/I²C и продвинутыми таймерами
закрывает все три проблемы и даёт запас лет на 5 вперёд.

## Соответствие пинов Arduino Nano → Nucleo-F103RB

Имена `D5`, `D7`, `D9`, `D10`, `D12` сохранены — Arduino-разъём на Nucleo
размечен ровно теми же номерами, что Uno/Nano. Меняется только реальная
STM32-нога за этим именем. Под `framework=arduino` (stm32duino) маппинг
делает сам core, поэтому код из `.ino` переносится с константами 1-в-1.

| Назначение | Arduino-имя | Nucleo разъём | STM32 пин | На AVR Nano было |
|---|---|---|---|---|
| `stepPin`    (выход импульсов на драйвер) | **D9**  | **CN5-2** | **PC7** *(TIM3_CH2)* | PB1 / OC1A (Timer1) |
| `dirPin`     (направление) | **D5**  | **CN9-6** | **PB4** *(TIM3_CH1)* | PD5 |
| `enPin`      (active-low enable) | **D10** | **CN5-3** | **PB6** | PB2 |
| `buttonPin1` (jog влево) | **D12** | **CN5-5** | **PA6** *(SPI1_MISO в будущем)* | PB4 |
| `buttonPin2` (jog вправо) | **D7**  | **CN9-8** | **PA8** | PD7 |
| LED          (онбордовый LD2) | `LED_BUILTIN` | онборд | **PA5** *(SPI1_SCK в будущем)* | D13 / PB5 |

> **Нумерация контактов разъёмов на MB1136 идёт снизу вверх** (со стороны
> ARDUINO-края платы). На CN5 пин 1 — это D8 (нижний), пин 10 — D15
> (верхний). На CN9 пин 1 — это D0/RX (нижний), пин 8 — D7 (верхний).
> Самый надёжный ориентир при втыкании провода — **надпись на силкскрине
> рядом с ногой** (`PWM/D9`, `PWM/D5`, `RX/D0` и т.п.), а не подсчёт пинов.

### Важные нюансы Nucleo

* **PA5 (LED_BUILTIN) и PA6 (D12) совпадают с SPI1.** Поэтому для SPI-канала
  к Pi мы используем **SPI2** (PB12-15 на Morpho CN10), а не SPI1 — так LED
  и пины мотора остаются свободны (см. раздел Step 3.0).
* **PC7 (D9 / step) умеет hardware-toggle через TIM3_CH2** — точный аналог
  AVR'овской OC1A. Это важно для Step 2.
* **`Serial`** под stm32duino выходит на USART2 (PA2/PA3), которые
  замкнуты на ST-Link/V2-1 → mini-USB → `/dev/ttyACM0` на Pi. Никакого
  отдельного UART-USB-конвертера не нужно.

## Питание Nucleo

Заводской вариант — джампер **JP5 = U5V**, плата запитана от ST-Link
USB-кабеля. Для Step 1 этого достаточно — втыкаем в USB Pi (или ноутбука),
плата живёт.

Если потом захочется автономии (без USB-кабеля к компьютеру) — JP5 в
**E5V** и подавать 5 В на пин `E5V` (CN7-pin 6). Потребление самой Nucleo:
~50 мА типично, ≤120 мА пиково.

> ⚠ Если JP5 стоит на **E5V**, но 5 В на пин E5V **не подаётся** — плата
> выглядит «мёртвой»: VTREF проседает до ~2.5 В, HSE не стартует, openocd
> пишет `Halt timed out`, LD2 тускло горит без миганий. Первым делом при
> таких симптомах проверяй JP5 и питание (см. шапку `platformio.ini`).

**Мотор и его драйвер запитывать ТОЛЬКО от своего БП** — бортовой LDO
Nucleo (LD1117S50) не предназначен для индуктивных нагрузок.

## Как собрать и прошить

PlatformIO ставится в venv проекта (`pip install platformio`) или системно
(`pipx install platformio`). На Pi 4 первый build качает stm32-toolchain
~150 МБ, дальше всё локально.

```bash
# из корня репо:
task fw_build       # собрать
task fw_flash       # залить через ST-Link
task fw_serial      # открыть /dev/ttyACM0 и слушать heartbeat
```

Эквивалент без taskipy (если уже стоишь в корне репо):

```bash
pio run -d firmware/stm32_nucleo_f103rb
pio run -d firmware/stm32_nucleo_f103rb -t upload
pio device monitor -d firmware/stm32_nucleo_f103rb
```

…или из самой папки прошивки:

```bash
cd firmware/stm32_nucleo_f103rb
pio run
pio run -t upload
pio device monitor
```

После `task fw_flash` и открытия `task fw_serial` должно появиться:

```
[nucleo-f103rb] SPI slave manual-drive test (Step 3.0)
  SPI2 slave: MOSI=PB15 MISO=PB14 SCK=PB13 NSS=PB12 (CN10)
  waiting for 6-byte packets from the Pi (spidev0.0) ...
[spi] active=0 omega=0  good=0 bad=0
```

Как только Pi начнёт слать пакеты (`task fw_spi_test ...`), строки `[spi]`
начнут показывать принятые `active`/`omega`, а `good` — расти. Подробный
сценарий проверки — в разделе [Step 3.0 → Как проверить](#как-проверить).

## Step 3.0 — SPI-канал Pi 4 → STM32 (тест ручного управления)

> **Текущая прошивка (`src/main.cpp`) — именно этот шаг.** STM32 работает
> SPI-**slave**, Pi 4 — SPI-**master** (`spidev0.0`). Pi шлёт бинарный кадр с
> «ручной» командой (флаг `manual_active` + скорость `omega`), STM32 печатает
> принятое в Serial и крутит мотор. Кнопки D7/D12 больше не опрашиваются.

### Почему SPI2, а не SPI1

SPI1 на Nucleo занят: `SPI1_SCK = PA5` — это онбордовый LED (LD2), а
`SPI1_MISO = PA6` мы использовали под кнопку. **SPI2 целиком висит на
свободных пинах Morpho-разъёма CN10**, поэтому LED и все пины мотора целы.

### Распиновка (Pi 4 SPI0 master → Nucleo CN10 slave)

Оба конца 3.3 В — соединяем одноимённые сигналы напрямую, без сдвига уровней.
**Общий GND между Pi и Nucleo обязателен.** Питание плат — раздельное (между
платами провод 3V3 НЕ тянуть).

| Сигнал | Pi 4 (BCM / пин хедера) | направление | STM32 пин | Nucleo CN10 |
|---|---|:---:|---|---|
| MOSI | GPIO10 / **pin 19** | Pi → STM32 | PB15 (SPI2_MOSI) | **CN10-26** |
| MISO | GPIO9  / **pin 21** | STM32 → Pi | PB14 (SPI2_MISO) | **CN10-28** |
| SCLK | GPIO11 / **pin 23** | Pi → STM32 | PB13 (SPI2_SCK)  | **CN10-30** |
| CE0  | GPIO8  / **pin 24** | Pi → STM32 | PB12 (SPI2_NSS)  | **CN10-16** |
| GND  | pin 20 или 25       | —          | GND              | **CN10-20** |

> CN10 — правый Morpho-разъём (если смотреть на плату с USB ST-Link вверху).
> MISO для теста можно даже не подключать (STM32 отдаёт счётчик принятых
> пакетов — приятно для проверки, но команда едет по MOSI).

### Формат пакета (6 байт, MSB-first, SPI mode 0)

```
[0]=0xAA  [1]=0x55  [2]=flags  [3]=omega_lo  [4]=omega_hi  [5]=xor
   flags bit0 = manual_active
   omega      = int16 little-endian (знаковая скорость, |omega| ≤ 200)
   xor        = XOR байтов [0..4]
```

Прошивка ищет преамбулу `AA 55`, проверяет XOR и при успехе обновляет
`manual_active`/`omega`. Бьётся контрольная сумма → кадр отбрасывается
(счётчик `bad` в Serial), на следующей преамбуле приём ресинхронизируется.

### Как проверить

1. **Включить SPI на Pi (один раз):**
   ```bash
   sudo raspi-config      # Interface Options → SPI → Enable → reboot
   # либо: echo 'dtparam=spi=on' | sudo tee -a /boot/firmware/config.txt && sudo reboot
   ```
   После перезагрузки должен появиться `/dev/spidev0.0`.
2. **Соединить провода** по таблице выше (+ общий GND).
3. **Залить прошивку и открыть Serial:**
   ```bash
   task fw_flash
   task fw_serial      # отдельный терминал — увидишь "[spi] active=.. omega=.. good=.. bad=.."
   ```
4. **Послать команду с Pi:**
   ```bash
   task fw_spi_test -- --mode 1 --omega 40     # ручной режим, мотор крутится
   task fw_spi_test -- --sweep                 # плавный свип скорости ±120
   task fw_spi_test -- --once --omega 25       # один пакет + эхо MISO
   ```
   В `task fw_serial` `good` должен расти, `bad` оставаться 0, `omega`
   повторять отправленное. Мотор крутится по знаку omega (если не в ту
   сторону — поменяй тернарник `omega > 0 ? HIGH : LOW` в `runStepper`).

## Что дальше

* **Step 3.1**: заменить `ArduinoHandler` (UART/CSV) в `hardware.py` на
  `Stm32SpiHandler` (`spidev`) и слать по этому же кадру рассогласование
  камеры (`normX`) вместо `omega` — расширить пакет полями err/derr + CRC16.
* **Step 2 (параллельно)**: HardwareTimer @1 кГц на TIM3_CH2 для velocity
  ramp вместо bit-bang в `loop()` — нужно для высоких частот шага.
* Долгосрочно — возможно переход с stm32duino на CubeMX+HAL/LL для
  предельной latency на ISR-ах.
