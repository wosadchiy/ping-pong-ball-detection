#!/usr/bin/env python3
"""
spi_test.py — проверка канала Pi 4 (SPI master) → STM32 Nucleo (SPI slave)
==========================================================================

Шлёт STM32 тот самый «ручной» drive-команд, что раньше уходил по UART как
``M<0/1>`` + ``O<float>`` (см. hardware.py / camera_control_v2.ino), но теперь
бинарным 6-байтовым пакетом по SPI:

    [0]=0xAA  [1]=0x55  [2]=flags  [3]=omega_lo  [4]=omega_hi  [5]=xor
      flags bit0 = manual_active (1 = ручной режим включён)
      omega      = int16 little-endian, знаковая скорость в user-units
      xor        = XOR байтов [0..4] — контроль целостности на стороне STM32

Прошивка (firmware/stm32_nucleo_f103rb/src/main.cpp) ищет преамбулу
``AA 55``, проверяет контрольную сумму и печатает каждую принятую команду в
USART2 VCP — то есть в ``task fw_serial`` ты ВИДИШЬ принятые байты, а мотор
крутится по знаку/величине omega.

Назначение
----------
1) Подтвердить, что spidev на Pi открывается и шина физически работает.
2) Убедиться, что STM32 ловит кадр (good++ в serial), а битые кадры
   ловятся контрольной суммой (bad++).
3) Прогнать реальную ручную команду приводом — основа для следующего шага
   (передача рассогласования камеры по этому же каналу).

Распиновка (Pi 4 SPI0 → Nucleo CN10, оба 3.3 В, общий GND обязателен!)
---------------------------------------------------------------------
    MOSI  Pi GPIO10 / hdr-pin 19  ──►  PB15  CN10-26
    MISO  Pi GPIO9  / hdr-pin 21  ◄──  PB14  CN10-28
    SCLK  Pi GPIO11 / hdr-pin 23  ──►  PB13  CN10-30
    CE0   Pi GPIO8  / hdr-pin 24  ──►  PB12  CN10-16
    GND   Pi hdr-pin 20 или 25    ───  GND   CN10-20

Перед запуском включи SPI на Pi (один раз):
    sudo raspi-config  →  Interface Options  →  SPI  →  Enable     (затем reboot)
  или добавь `dtparam=spi=on` в /boot/firmware/config.txt и перезагрузись.
Появится /dev/spidev0.0 — это CE0.

Примеры
-------
  # включить ручной режим, постоянная скорость +40 (мотор крутится в одну сторону)
  python firmware/stm32_nucleo_f103rb/tools/spi_test.py --mode 1 --omega 40

  # выключить привод (manual_active=0)
  python firmware/stm32_nucleo_f103rb/tools/spi_test.py --mode 0

  # демонстрация: плавный свип скорости от -120 до +120 и обратно
  python firmware/stm32_nucleo_f103rb/tools/spi_test.py --sweep

  # послать ровно один пакет и показать, что вернул STM32 по MISO
  python firmware/stm32_nucleo_f103rb/tools/spi_test.py --mode 1 --omega 25 --once
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import time

SYNC0 = 0xAA
SYNC1 = 0x55
OMEGA_LIMIT = 200  # совпадает с клампом err_raw в прошивке/легаси


def build_packet(manual_active: bool, omega: float) -> list[int]:
    """Собрать 6-байтовый кадр {sync, sync, flags, omega_lo, omega_hi, xor}."""
    o = int(round(omega))
    o = max(-OMEGA_LIMIT, min(OMEGA_LIMIT, o))
    lo, hi = struct.pack("<h", o)  # int16 little-endian -> два байта
    flags = 0x01 if manual_active else 0x00
    body = [SYNC0, SYNC1, flags, lo, hi]
    checksum = 0
    for b in body:
        checksum ^= b
    return body + [checksum & 0xFF]


def open_spi(bus: int, dev: int, speed_hz: int):
    """Открыть spidev с понятной ошибкой, если SPI не включён / нет модуля."""
    try:
        import spidev  # noqa: WPS433 (локальный импорт — чтобы скрипт грузился и без spidev)
    except ImportError:
        print(
            "ERROR: модуль 'spidev' не установлен.\n"
            "       pip install spidev   (внутри venv проекта),\n"
            "       либо: task install",
            file=sys.stderr,
        )
        raise SystemExit(2)

    spi = spidev.SpiDev()
    try:
        spi.open(bus, dev)
    except FileNotFoundError:
        print(
            f"ERROR: /dev/spidev{bus}.{dev} не найден.\n"
            "       Включи SPI: sudo raspi-config → Interface Options → SPI → Enable,\n"
            "       или 'dtparam=spi=on' в /boot/firmware/config.txt, затем reboot.",
            file=sys.stderr,
        )
        raise SystemExit(2)

    spi.max_speed_hz = speed_hz
    spi.mode = 0  # CPOL=0, CPHA=0 — совпадает с прошивкой (SPI mode 0)
    spi.bits_per_word = 8
    return spi


def send_packet(spi, packet: list[int], echo: bool) -> list[int]:
    """Передать кадр и вернуть то, что STM32 одновременно отдал по MISO."""
    returned = spi.xfer2(list(packet))
    if echo:
        tx = " ".join(f"{b:02X}" for b in packet)
        rx = " ".join(f"{b:02X}" for b in returned)
        print(f"TX [{tx}]   MISO [{rx}]")
    return returned


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--bus", type=int, default=0, help="SPI bus (default 0)")
    ap.add_argument("--dev", type=int, default=0,
                    help="SPI chip-select / device (default 0 → /dev/spidev0.0, CE0)")
    ap.add_argument("--speed", type=int, default=500_000,
                    help="SPI clock in Hz (default 500000 — безопасно для slave-ISR)")
    ap.add_argument("--mode", type=int, choices=(0, 1), default=1,
                    help="manual_active flag: 1=ручной режим вкл, 0=выкл (default 1)")
    ap.add_argument("--omega", type=float, default=40.0,
                    help="скорость (user units, знак=направление), |omega|<=200")
    ap.add_argument("--rate", type=float, default=50.0,
                    help="пакетов в секунду в continuous/sweep (default 50)")
    ap.add_argument("--once", action="store_true",
                    help="послать ровно один пакет и выйти (с эхом MISO)")
    ap.add_argument("--sweep", action="store_true",
                    help="демо: непрерывный синус-свип omega ±|omega| (mode форсится в 1)")
    ap.add_argument("--sweep-amp", type=float, default=120.0,
                    help="амплитуда свипа (default 120)")
    ap.add_argument("--sweep-freq", type=float, default=0.25,
                    help="частота свипа в Гц (default 0.25 → период 4 с)")
    ap.add_argument("--echo", action="store_true",
                    help="печатать каждый отправленный пакет и ответ MISO")
    args = ap.parse_args()

    spi = open_spi(args.bus, args.dev, args.speed)
    print(f"opened /dev/spidev{args.bus}.{args.dev} @ {args.speed} Hz, mode 0")

    try:
        if args.once:
            pkt = build_packet(bool(args.mode), args.omega)
            send_packet(spi, pkt, echo=True)
            print(f"sent one packet: active={args.mode} omega={int(round(args.omega))}")
            return 0

        if args.sweep:
            period = 1.0 / args.rate
            w = 2.0 * math.pi * args.sweep_freq
            print(
                f"sweep: ±{args.sweep_amp:.0f} @ {args.sweep_freq:.3f} Hz, "
                f"{args.rate:.0f} pkt/s — Ctrl+C to stop"
            )
            t0 = time.monotonic()
            n = 0
            while True:
                t = time.monotonic() - t0
                omega = args.sweep_amp * math.sin(w * t)
                pkt = build_packet(True, omega)
                send_packet(spi, pkt, echo=args.echo and (n % 25 == 0))
                n += 1
                time.sleep(period)

        # default: continuous fixed command at --rate (держим STM32 в курсе)
        period = 1.0 / args.rate
        print(
            f"streaming active={args.mode} omega={int(round(args.omega))} "
            f"@ {args.rate:.0f} pkt/s — Ctrl+C to stop"
        )
        n = 0
        while True:
            pkt = build_packet(bool(args.mode), args.omega)
            send_packet(spi, pkt, echo=args.echo and (n % 25 == 0))
            n += 1
            time.sleep(period)

    except KeyboardInterrupt:
        print("\nstopping — sending safe packet (active=0, omega=0)")
    finally:
        try:
            send_packet(spi, build_packet(False, 0.0), echo=False)
        except Exception:
            pass
        spi.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
