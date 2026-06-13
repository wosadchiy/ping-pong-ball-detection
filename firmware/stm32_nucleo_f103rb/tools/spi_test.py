#!/usr/bin/env python3
"""
spi_test.py — проверка канала Pi 4 (SPI master) → STM32 Nucleo (SPI slave)
==========================================================================

Шлёт STM32 drive-команду 16-байтовым бинарным пакетом по SPI. Прошивка сама
считает закон управления (ручной режим > слежение > покой), здесь мы лишь
эмулируем то, что в бою отправляет hardware.py из данных камеры:

    [0]  0xAA      преамбула
    [1]  0x55      преамбула
    [2]  flags     bit0 = manual_active, bit1 = tracking
    [3..4]   manual_omega  int16 LE  (знаковая скорость, user units)
    [5..6]   err_n         int16 LE  (норм. ошибка ×10000, ±1.0)
    [7..8]   derr_n        int16 LE  (норм. производная ×1000, в сек)
    [9..10]  kp_x100       int16 LE  (Kp × 100)
    [11..12] max_omega     int16 LE  (ограничение скорости)
    [13..14] td_x1000      int16 LE  (Td × 1000, сек)
    [15] xor   XOR байтов [0..14]

Прошивка (firmware/stm32_nucleo_f103rb/src/main.cpp) ищет преамбулу
``AA 55``, проверяет контрольную сумму и печатает каждую принятую команду в
USART2 VCP — то есть в ``task fw_serial`` ты ВИДИШЬ принятые байты, а мотор
крутится по вычисленной omega.

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
  # ручной режим, постоянная скорость +40 (мотор крутится в одну сторону)
  python firmware/stm32_nucleo_f103rb/tools/spi_test.py --mode 1 --omega 40

  # выключить привод (manual_active=0, tracking=0)
  python firmware/stm32_nucleo_f103rb/tools/spi_test.py --mode 0

  # демонстрация ручного режима: плавный свип скорости ±120
  python firmware/stm32_nucleo_f103rb/tools/spi_test.py --sweep

  # эмуляция слежения: tracking=1, ошибка err=0.5 (полкадра), Kp=1, max_omega=60
  python firmware/stm32_nucleo_f103rb/tools/spi_test.py --track --err 0.5 --kp 1 --max-omega 60

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
OMEGA_LIMIT = 200  # совпадает с клампом в прошивке/легаси
ERR_N_SCALE = 10000.0
DERR_N_SCALE = 1000.0
KP_SCALE = 100.0
TD_SCALE = 1000.0
INT16_LIMIT = 32767


def _i16(v: float) -> int:
    iv = int(round(v))
    return max(-INT16_LIMIT, min(INT16_LIMIT, iv))


def build_packet(manual_active: bool, omega: float, *, tracking: bool = False,
                 err_n: float = 0.0, derr_n: float = 0.0, kp: float = 1.0,
                 max_omega: float = 40.0, td: float = 0.0) -> list[int]:
    """Собрать 16-байтовый control-кадр (см. модульный docstring)."""
    o = max(-OMEGA_LIMIT, min(OMEGA_LIMIT, int(round(omega))))
    flags = (0x01 if manual_active else 0x00) | (0x02 if tracking else 0x00)
    body = [SYNC0, SYNC1, flags]
    body += list(struct.pack("<h", o))
    body += list(struct.pack("<h", _i16(max(-1.0, min(1.0, err_n)) * ERR_N_SCALE)))
    body += list(struct.pack("<h", _i16(max(-10.0, min(10.0, derr_n)) * DERR_N_SCALE)))
    body += list(struct.pack("<h", _i16(kp * KP_SCALE)))
    body += list(struct.pack("<h", _i16(max_omega)))
    body += list(struct.pack("<h", _i16(td * TD_SCALE)))
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
    # --- эмуляция режима слежения камеры (tracking) ---
    ap.add_argument("--track", action="store_true",
                    help="tracking=1: STM32 считает PD-закон по err/derr/Kp/max-omega")
    ap.add_argument("--err", type=float, default=0.0,
                    help="нормированная ошибка камеры ±1.0 (для --track)")
    ap.add_argument("--derr", type=float, default=0.0,
                    help="нормированная производная ошибки, в сек (для --track)")
    ap.add_argument("--kp", type=float, default=1.0, help="Kp (для --track)")
    ap.add_argument("--max-omega", type=float, default=40.0,
                    help="ограничение скорости (для --track)")
    ap.add_argument("--td", type=float, default=0.0,
                    help="Td — производная составляющая, сек (для --track)")
    ap.add_argument("--self-test", action="store_true",
                    help="проверить сборку пакета без железа и выйти")
    args = ap.parse_args()

    if args.self_test:
        _self_test()
        return 0

    spi = open_spi(args.bus, args.dev, args.speed)
    print(f"opened /dev/spidev{args.bus}.{args.dev} @ {args.speed} Hz, mode 0")

    def track_kwargs() -> dict:
        return dict(tracking=True, err_n=args.err, derr_n=args.derr,
                    kp=args.kp, max_omega=args.max_omega, td=args.td)

    try:
        if args.once:
            if args.track:
                pkt = build_packet(False, 0.0, **track_kwargs())
                send_packet(spi, pkt, echo=True)
                print(f"sent one tracking packet: err={args.err} kp={args.kp} "
                      f"max_omega={args.max_omega} td={args.td}")
            else:
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

        # default: continuous command at --rate (держим STM32 в курсе)
        period = 1.0 / args.rate
        if args.track:
            print(
                f"streaming tracking err={args.err} derr={args.derr} kp={args.kp} "
                f"max_omega={args.max_omega} td={args.td} @ {args.rate:.0f} pkt/s "
                f"— Ctrl+C to stop"
            )
        else:
            print(
                f"streaming active={args.mode} omega={int(round(args.omega))} "
                f"@ {args.rate:.0f} pkt/s — Ctrl+C to stop"
            )
        n = 0
        while True:
            if args.track:
                pkt = build_packet(False, 0.0, **track_kwargs())
            else:
                pkt = build_packet(bool(args.mode), args.omega)
            send_packet(spi, pkt, echo=args.echo and (n % 25 == 0))
            n += 1
            time.sleep(period)

    except KeyboardInterrupt:
        print("\nstopping — sending safe packet (all zero/off)")
    finally:
        try:
            send_packet(spi, build_packet(False, 0.0), echo=False)
        except Exception:
            pass
        spi.close()
    return 0


def _self_test() -> None:
    """Sanity-check the packet builder without hardware (CI / `--self-test`)."""
    import struct as _struct
    cases = [
        dict(manual_active=True, omega=40),
        dict(manual_active=False, omega=0, tracking=True, err_n=0.5, kp=1.0,
             max_omega=60, td=0.0),
    ]
    for kw in cases:
        p = build_packet(kw.pop("manual_active"), kw.pop("omega"), **kw)
        assert len(p) == 16, f"len {len(p)} != 16"
        assert p[0] == SYNC0 and p[1] == SYNC1, p
        xor = 0
        for b in p[:15]:
            xor ^= b
        assert p[15] == xor, ("checksum", p)
        _ = _struct.unpack("<h", bytes(p[3:5]))[0]
    print("self-test OK: 16-byte packets, checksums valid")


if __name__ == "__main__":
    raise SystemExit(main())
