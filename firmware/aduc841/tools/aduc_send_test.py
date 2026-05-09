#!/usr/bin/env python3
"""
aduc_send_test.py — проверка канала Python → ADuC841 → DAC0/DAC1
================================================================

Гоняет синусоидальный паттерн `dx,dy\\n` в ADuC841 на нормальном (не
бутлоадерном!) рантайме. На осциллографе это должно выглядеть как
две синусоиды в квадратуре (dx — в фазе sin, dy — со сдвигом π/2).

Назначение
----------
Проверить, что:
  1) FTDI-порт открывается, и ADuC принимает байты в нормальном режиме
     (БЕЗ нажатия PSEN/RESET — прошивка main.c уже прошита и работает).
  2) Парсер `dx,dy\\n` в прошивке корректно мапит знаковые значения
     в 12-bit DAC code (0V — полный левый край, AVdd — полный правый,
     AVdd/2 — центр).
  3) End-to-end джиттер канала Python→FTDI→UART (на наших 9600 baud)
     стабилен — это база для интерпретации результатов измерения
     latency камерного конвейера.

Пример
------
  ./venv/bin/python firmware/aduc841/tools/aduc_send_test.py
  ./venv/bin/python firmware/aduc841/tools/aduc_send_test.py --rate 30 --amp 320

Подсказки
---------
- На осциллографе ставьте AC-coupling и развёртку ~ 100 ms/div, чтобы
  увидеть весь период (по умолчанию 0.5 Hz → период 2 секунды).
- На X1 (DAC0) триггер по rising edge у середины (1.65 V на AVdd=3.3),
  X2 (DAC1) выводите вторым каналом — должна быть та же синусоида,
  сдвинутая на четверть периода вправо.
- `Ctrl+C` чтобы остановить.
"""

import argparse
import math
import sys
import time

import serial

# Чтобы можно было запускать как отдельный скрипт И из проектного venv.
sys.path.insert(0, "../../..")
try:
    from hardware import AducHandler  # noqa: E402
except Exception:
    AducHandler = None


def autodetect_port() -> str | None:
    """Найти ADuC порт через тот же механизм, что и в проектном hardware.py."""
    if AducHandler is not None:
        return AducHandler.find_aduc()
    # Fallback: ручной перебор, если запускают вне venv.
    import serial.tools.list_ports
    for p in serial.tools.list_ports.comports():
        if "usbserial-A" in p.device or "ftdi" in (p.manufacturer or "").lower():
            return p.device
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None,
                    help="ADuC FTDI port (auto if omitted)")
    ap.add_argument("--baud", type=int, default=9600,
                    help="UART baud (must match firmware UART_BAUD)")
    ap.add_argument("--rate", type=float, default=30.0,
                    help="packets per second (default 30 — like camera fps)")
    ap.add_argument("--amp", type=int, default=320,
                    help="sine amplitude in pixels (default 320 = ±half VGA)")
    ap.add_argument("--freq", type=float, default=0.5,
                    help="sine frequency in Hz (default 0.5 — period 2 s)")
    ap.add_argument("--quadrature", action="store_true", default=True,
                    help="dy = cos(2π f t) (quadrature with dx — default)")
    ap.add_argument("--echo", action="store_true",
                    help="also print every packet to stdout")
    args = ap.parse_args()

    port = args.port or autodetect_port()
    if not port:
        print("ERROR: ADuC port not found. Pass --port /dev/cu.usbserial-... explicitly.",
              file=sys.stderr)
        return 2

    print(f"opening {port} @ {args.baud} baud …")
    ser = serial.Serial(port, args.baud, timeout=0.05)
    # FTDI: НЕ дёргать DTR/RTS чтобы не сбросить чип.
    ser.dtr = False
    ser.rts = False
    time.sleep(0.1)

    period = 1.0 / args.rate
    omega = 2.0 * math.pi * args.freq

    print(f"streaming sine: amp={args.amp}px  freq={args.freq:.3f}Hz  "
          f"rate={args.rate:.1f}Hz  Ctrl+C to stop")
    t0 = time.monotonic()
    next_tick = t0
    n = 0

    try:
        while True:
            now = time.monotonic()
            t = now - t0
            dx = int(round(args.amp * math.sin(omega * t)))
            dy = int(round(args.amp * math.cos(omega * t))) if args.quadrature \
                 else int(round(args.amp * math.sin(omega * t)))

            line = f"{dx},{dy}\n".encode()
            ser.write(line)

            if args.echo and (n % 10 == 0):
                # каждый 10-й пакет — иначе stdout засоряется
                print(f"t={t:6.2f}s  dx={dx:+5d}  dy={dy:+5d}")

            n += 1
            next_tick += period
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                # отстали — догоняем, не накапливаем долг
                next_tick = time.monotonic()

    except KeyboardInterrupt:
        print(f"\nstopped after {n} packets ({n / (time.monotonic() - t0):.1f} pkt/s)")
    finally:
        # Прокинем "стоп": dx=dy=0 → DAC возвращается в центр.
        try:
            ser.write(b"0,0\n")
            time.sleep(0.05)
        except Exception:
            pass
        ser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
