import os
import struct
import time
from threading import Event, Lock, Thread

import serial
import serial.tools.list_ports

from platform_utils import ADUC_MATCH_KEYWORDS, ARDUINO_MATCH_KEYWORDS


def _find_serial_port(
    keywords: tuple[str, ...],
    env_var: str | None = None,
    prefer_path_substr: str | None = None,
) -> str | None:
    """Return a serial port whose description/manufacturer/device path matches
    any of `keywords` (case-insensitive).

    Override priority:
      1. If `env_var` is set in the environment — return it verbatim.
      2. If `prefer_path_substr` is given AND a matching port contains it in
         its path — return that one (e.g. prefer `wchusbserial14230` over
         the Apple-driver double `usbserial-14230` for the same CH340 chip;
         the Apple node sometimes returns EINVAL on `serial.Serial(...)`).
      3. Otherwise, return the first matching port in pyserial's enumeration
         order.
    """
    forced = os.environ.get(env_var) if env_var else None
    if forced:
        return forced

    matches: list[str] = []
    for p in serial.tools.list_ports.comports():
        haystack = " ".join(
            filter(None, [p.description, p.manufacturer, p.device])
        ).lower()
        if any(kw in haystack for kw in keywords):
            matches.append(p.device)

    if prefer_path_substr:
        for m in matches:
            if prefer_path_substr in m:
                return m

    return matches[0] if matches else None


class ArduinoHandler:
    def __init__(self, baudrate=115200, disabled: bool = False):
        self.ser = None
        self.enabled = False

        # Caches for the out-of-band A/M/O commands. We only emit a new
        # serial line when the cached value differs from the current store
        # value, so the bandwidth cost of these knobs is roughly zero
        # during steady state — they only fire on user interaction.
        # Sentinels chosen so the FIRST send_data() call always pushes a
        # full A+M+O snapshot to the firmware (initial sync).
        self._last_accel: float | None = None
        self._last_manual_active: bool | None = None
        self._last_manual_omega: float | None = None
        # Td — derivative time. Sentinel = None forces a first-send.
        self._last_td: float | None = None

        # Явный dev-режим: пропускаем поиск порта и остаёмся выключенными.
        # Все send/receive уже защищены гейтом self.enabled, так что
        # остальной pipeline (camera/detector/recorder/UI) работает как есть.
        if disabled:
            print("Arduino disabled via --no-arduino. Running in dev mode (no serial I/O).")
            return

        port = self.find_arduino()
        if port:
            try:
                self.ser = serial.Serial(port, baudrate, timeout=0.001)
                time.sleep(2)  # Arduino boot delay after DTR reset
                self.enabled = True
                print(f"Connected to Arduino: {port}")
            except Exception as e:
                # Печатаем и порт, и тип исключения — без этого было видно
                # только "Connection error: (22, 'Invalid argument')",
                # неясно к какому устройству он пробовал постучаться.
                print(
                    f"Arduino connection error on {port}: "
                    f"{type(e).__name__}: {e}"
                )
        else:
            print("Arduino not found. Tracking will run without serial output.")

    @staticmethod
    def find_arduino():
        """Locate the first Arduino-compatible USB-serial port across OSes.

        Matches Arduino-specific bridges only (CH340 / CP210x / native-USB
        Arduinos). FTDI-based devices (like the ADuC841 latency-test board)
        are intentionally NOT matched here — see `AducHandler.find_aduc()`.

        Override with `ARDUINO_PORT=/dev/cu.foo` if autodetection picks the
        wrong device.

        Note: macOS often exposes a CH340 dongle under TWO node names —
        `/dev/cu.usbserial-XXXX` (Apple's generic driver, occasionally
        returns EINVAL on open) and `/dev/cu.wchusbserial-XXXX` (the WCH
        official driver, reliable). We tell `_find_serial_port` to PREFER
        the WCH node when both are present.
        """
        return _find_serial_port(
            ARDUINO_MATCH_KEYWORDS,
            env_var="ARDUINO_PORT",
            prefer_path_substr="wchusbserial",
        )

    def _write_line(self, msg: str) -> None:
        """Low-level: append a newline (if missing) and write to serial.

        Swallows expected disconnect errors so the logic loop keeps
        running when the Arduino is unplugged mid-session.
        """
        if not (self.enabled and self.ser and self.ser.is_open):
            return
        if not msg.endswith("\n"):
            msg += "\n"
        try:
            self.ser.write(msg.encode())
        except (serial.SerialException, OSError):
            pass

    def _push_drive_tuning(self, store) -> None:
        """Emit A/M/O updates only when the corresponding store field changed.

        The firmware caches each of these values internally, so re-sending
        the same value is harmless but wasteful. Doing the diff in Python
        keeps the serial line quiet (~0 bytes/sec while no slider moves)
        and makes scope captures readable when debugging the protocol.

        Order: A first (so the firmware knows the new accel BEFORE it
        starts ramping toward a new manual_omega), then M, then O. The
        Arduino loop is fast enough that the order rarely matters in
        practice but it's still nice to think about cause-and-effect.
        """
        accel = float(getattr(store, "accel", 100.0))
        if self._last_accel is None or abs(self._last_accel - accel) > 0.01:
            self._write_line(f"A{accel:.2f}")
            self._last_accel = accel

        manual_active = bool(getattr(store, "manual_omega_active", False))
        if self._last_manual_active != manual_active:
            self._write_line(f"M{1 if manual_active else 0}")
            self._last_manual_active = manual_active

        manual_omega = float(getattr(store, "manual_omega", 0.0))
        # 0.05 user-unit deadband matches the slider step (1) divided by 20:
        # any deliberate slider drag triggers a send, but the OS-driven
        # focus jitter on dpg.add_input_int won't.
        if (self._last_manual_omega is None
                or abs(self._last_manual_omega - manual_omega) > 0.05):
            self._write_line(f"O{manual_omega:.2f}")
            self._last_manual_omega = manual_omega

        # Td — derivative time (sec) for the PD regulator on Arduino.
        # 0.0005 deadband ≈ half of slider step (0.005); меньшая
        # детализация вряд ли вообще различима в поведении мотора.
        td = float(getattr(store, "td", 0.0))
        if self._last_td is None or abs(self._last_td - td) > 0.0005:
            self._write_line(f"D{td:.4f}")
            self._last_td = td

    def send_data(self, ax, ay, nx, ny, dnx, dny, store):
        """Push a single control packet to Arduino. Field layout:

            ax, ay     — degrees (FOV-based), kept for diagnostics
            nx, ny     — *pixels* in (-w/2..+w/2), error term for P-part
            kp         — proportional gain (float)
            tracking   — 0/1
            max_omega  — speed cap (user units)
            dnx, dny   — derivative of nx/ny in pixels/sec, computed in
                         `detector.process` from EMA-smoothed values with
                         the actual `dt` between detector iterations.
                         Used by the Arduino as the D-part of a PD law:
                             omega = (err + Td * derr) * max_omega * Kp.

        Backwards-compat note: the legacy 7-field CSV (without dnx, dny)
        is a strict prefix of the new 9-field CSV. Old firmware that calls
        `getValue(7)` will simply not see the new tail. New firmware reads
        all 9 fields. Td itself is sent OUT-OF-BAND as `D<float>` (see
        `_push_drive_tuning`) — same pattern as A/M/O — so changing it
        never disrupts the steady-state CSV stream.
        """
        if not (self.enabled and self.ser and self.ser.is_open):
            return

        # Out-of-band drive-tuning updates first — they're cheap and only
        # fire on actual changes. Doing them BEFORE the CSV means an
        # accel/Td/mode change reaches the firmware in the same TX burst
        # as the next regular packet, so the visible motor response is
        # in sync with the slider movement.
        self._push_drive_tuning(store)

        try:
            # Передаем в поле Kp не коэффициент, а 64000.0 / Kp (или 0 если Kp=0)
            kp_coeff = 0.0
            if getattr(store, 'kp', 0.0) > 0.0:
                kp_coeff = 64000.0 / float(store.kp)
            msg = (
                f"{ax:.2f},{ay:.2f},{nx:.2f},{ny:.2f},"
                f"{kp_coeff:.2f},{int(store.is_tracking)},{store.max_omega:.1f},"
                f"{dnx:.2f},{dny:.2f}\n"
            )
            self.ser.write(msg.encode())
        except (serial.SerialException, OSError):
            pass

    def receive_data(self):
        if self.enabled and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode(errors="ignore").strip()
                    if line:
                        print(f"ARDUINO: {line}")
            except Exception:
                pass

    def close(self):
        if self.ser:
            self.ser.close()
            print("Serial port closed.")


class AducHandler:
    """Serial link to the ADuC841 latency-test board.

    Pushes `dx,dy\\n` packets to the ADuC841, whose firmware mirrors them onto
    its on-chip 12-bit DACs (DAC0 = dx, DAC1 = dy). The DAC outputs go to an
    oscilloscope alongside a turntable strobe to measure end-to-end pipeline
    latency: camera → OpenCV → EMA → serial → MCU → DAC.

    Independent of `ArduinoHandler`: the Arduino Nano (CH340) drives the motor
    as before; the ADuC841 (FTDI) only listens and writes to its DAC.

    DESIGN — async writer thread (non-blocking from caller's POV)
    ------------------------------------------------------------
    На 9600 baud один пакет ~12 байт уходит ~12 ms по проводу. С FTDI
    latency timer ~16 ms на macOS суммарная задержка одного `serial.write()`
    может доходить до ~30 ms — а logic thread шлёт нам каждые ~16-33 ms
    (60-30 fps камеры). Если хост-буфер FTDI заполняется, блокирующий
    `write()` подвешивает logic thread → камера фризит на секунды (это и
    наблюдалось до этой ревизии).

    Решение — `latest-wins` буфер на 1 элемент + отдельный writer-thread:
      * `send_dx_dy(dx, dy)` — атомарно записывает свежую пару под Lock,
        пробуждает writer событием. Возвращается мгновенно (микросекунды).
      * Writer-thread в своём темпе берёт последнее значение и шлёт его.
        Если writer не успевает (медленный baud / latency timer) — старые
        значения просто перетираются новыми. Это **именно то, что нужно**
        для замера latency: DAC должен отслеживать ТЕКУЩИЙ nx/ny, а не
        полную историю. Лишние сэмплы между ~30 Hz обновлениями DAC
        нерепрезентативны для измерения задержки.

    Bootloader baud (9600) — см. `firmware/aduc841/src/main.c`.
    Override port: env var `ADUC_PORT=/dev/cu.bar`.
    """

    def __init__(self, baudrate: int = 115200):
        # ⚠ baud 115200 ДОЛЖЕН совпадать с UART_BAUD в
        # firmware/aduc841/src/main.c — иначе парсер увидит мусор.
        # Бутлоадер при этом всё равно работает на 9600 (это в нём
        # прошито), так что `make flash` НЕ ломается.
        self.ser: serial.Serial | None = None
        self.enabled = False
        self._latest: tuple[float, float] | None = None
        self._latest_lock = Lock()
        self._wakeup = Event()
        self._stop = Event()
        self._writer_thread: Thread | None = None
        self._sec_per_byte = 10.0 / float(baudrate)  # 8N1, 10 бит на байт

        port = self.find_aduc()
        if not port:
            print("ADuC841 not found. Latency-test DAC mirror disabled.")
            return

        try:
            self.ser = serial.Serial(
                port, baudrate,
                timeout=0.001,
                # write_timeout — страховка: даже если буфер драйвера
                # как-то намертво заклинит, writer-thread не залипнет
                # навсегда, а кинет SerialTimeoutException и пропустит
                # этот сэмпл. Logic thread про это не знает.
                write_timeout=0.05,
            )
            # FTDI does NOT auto-reset on DTR like an Arduino — keep both
            # lines deasserted so the chip stays running on open.
            self.ser.dtr = False
            self.ser.rts = False
            self.enabled = True
            print(f"Connected to ADuC841: {port} @ {baudrate} baud")
        except Exception as e:
            print(f"ADuC841 connection error: {e}")
            return

        self._writer_thread = Thread(
            target=self._writer_loop, name="aduc-writer", daemon=True
        )
        self._writer_thread.start()

    @staticmethod
    def find_aduc() -> str | None:
        """Locate the ADuC841 board's FTDI USB-serial port. Override with
        `ADUC_PORT=/dev/cu.bar` if autodetection picks the wrong device."""
        return _find_serial_port(ADUC_MATCH_KEYWORDS, env_var="ADUC_PORT")

    def send_dx_dy(self, dx: float, dy: float) -> None:
        """NON-BLOCKING: stash latest (dx, dy) for the writer thread.

        Safe to call from logic-thread on every frame at any rate. Returns
        in microseconds — no serial I/O happens in the calling thread.
        Stale values are silently overwritten if the writer is behind.
        """
        if not self.enabled:
            return
        with self._latest_lock:
            self._latest = (dx, dy)
        self._wakeup.set()

    def _writer_loop(self) -> None:
        """Drain the latest-wins slot and push it on the wire as fast as
        the link allows. Two layers of backpressure prevent OS-buffer
        накопление (та самая 5-8-секундная задержка, что мы наблюдали):

        1. `out_waiting` check ДО write — если в TX-буфере драйвера ещё
           не прокачался прошлый пакет, дропаем этот sample, ждём
           следующего wakeup. Свежее значение всегда придёт.
        2. `time.sleep(N * sec_per_byte)` ПОСЛЕ write — даём UART
           буквально опустошиться, чтобы следующий write не лёг
           поверх предыдущего в драйверный буфер.
        """
        OUT_BUFFER_BUDGET = 32   # bytes; ~3 пакета на 115200 baud (~3 ms)

        while not self._stop.is_set():
            self._wakeup.wait(timeout=0.1)
            if self._stop.is_set():
                break
            self._wakeup.clear()

            with self._latest_lock:
                pair = self._latest
                self._latest = None

            if pair is None:
                continue
            if not (self.ser and self.ser.is_open):
                continue

            try:
                if self.ser.out_waiting > OUT_BUFFER_BUDGET:
                    continue
            except OSError:
                pass  # not all platforms expose out_waiting reliably

            dx, dy = pair
            # Прошивочный парсер принимает только целые (см. firmware/aduc841/
            # src/main.c, feed()). Округляем до int — потеря субпиксельной
            # точности тут несущественна: 1 px ≈ 5 mV на DAC при AVdd=3.3 В
            # и DX_MAX=320 (полный размах AVdd), что много ниже шума.
            msg = f"{int(round(dx))},{int(round(dy))}\n".encode()
            try:
                self.ser.write(msg)
            except (serial.SerialException, serial.SerialTimeoutException, OSError):
                # Дроп пакета — следующий тик logic-thread'а перезапишет
                # _latest и разбудит нас снова. Не страшно.
                continue

            # Адаптивный rate limit: спим ровно столько, сколько UART
            # будет прокачивать этот пакет. На 115200 это ~1 ms на 11 байт.
            # Множитель 0.9 — чуть-чуть undersleep, чтобы writer всегда
            # был готов отправить следующий sample сразу как UART
            # освободится, без зазора.
            time.sleep(len(msg) * self._sec_per_byte * 0.9)

    def close(self) -> None:
        self._stop.set()
        self._wakeup.set()
        if self._writer_thread and self._writer_thread.is_alive():
            self._writer_thread.join(timeout=1.0)
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            print("ADuC841 serial port closed.")


class Stm32SpiHandler:
    """SPI master link Pi 4 → STM32 Nucleo-F103RB (SPI slave on SPI2).

    Replaces the UART `ArduinoHandler` path for *manual drive*: instead of
    sending `M<0/1>` + `O<float>` text lines over a CH340 serial port (which
    no longer exists — the motor moved to the STM32), we push a compact
    binary frame over /dev/spidev0.0 to the firmware in
    `firmware/stm32_nucleo_f103rb/src/main.cpp`.

    The STM32 runs the control law (like the old Arduino did); Python only
    supplies normalised camera inputs + tuning. Frame (16 bytes, MSB-first,
    SPI mode 0) — MUST match the firmware decoder and `tools/spi_test.py`:

        [0]  0xAA      preamble
        [1]  0x55      preamble
        [2]  flags     bit0 = manual_active, bit1 = tracking
        [3..4]   manual_omega  int16 LE  (user units, signed)
        [5..6]   err_n         int16 LE  (normalised error ×10000, ±1.0)
        [7..8]   derr_n        int16 LE  (normalised d/dt   ×1000, per sec)
        [9..10]  kp_x100       int16 LE  (Kp × 100)
        [11..12] max_omega     int16 LE  (speed cap, user units)
        [13..14] td_x1000      int16 LE  (derivative time Td × 1000, sec)
        [15] xor   XOR of bytes [0..14]

    Independent of the `--no-arduino` flag on purpose: that flag only skips
    the legacy UART search. `task dev_pi` runs with `--no-arduino`, yet still
    needs this SPI path to drive the motor. Use `--no-spi` to disable.
    """

    SYNC0 = 0xAA
    SYNC1 = 0x55
    PKT_LEN = 16
    OMEGA_LIMIT = 200       # matches the firmware/legacy err_raw clamp
    ERR_N_SCALE = 10000.0   # err_n  int16  → [-1.0, +1.0]
    DERR_N_SCALE = 1000.0   # derr_n int16  → per-second
    DERR_N_LIMIT = 10.0     # clamp normalised derivative before scaling
    KP_SCALE = 100.0
    TD_SCALE = 1000.0
    INT16_LIMIT = 32767

    def __init__(self, disabled: bool = False, bus: int = 0, dev: int = 0,
                 speed_hz: int = 500_000):
        self.spi = None
        self.enabled = False

        if disabled:
            print("STM32 SPI disabled via --no-spi. Manual drive over SPI is off.")
            return

        try:
            import spidev  # noqa: WPS433 — optional, Linux-only dependency
        except ImportError:
            print(
                "spidev not installed — STM32 SPI link disabled. "
                "Install with `pip install spidev` (Linux/Pi only)."
            )
            return

        try:
            self.spi = spidev.SpiDev()
            self.spi.open(bus, dev)
            self.spi.max_speed_hz = speed_hz
            self.spi.mode = 0  # CPOL=0, CPHA=0 — matches the firmware
            self.spi.bits_per_word = 8
            self.enabled = True
            print(f"Connected to STM32 over SPI: /dev/spidev{bus}.{dev} @ {speed_hz} Hz")
        except FileNotFoundError:
            print(
                f"/dev/spidev{bus}.{dev} not found — STM32 SPI link disabled. "
                "Enable SPI: raspi-config → Interface Options → SPI, then reboot."
            )
            self.spi = None
        except Exception as e:
            print(f"STM32 SPI open error: {type(e).__name__}: {e}")
            self.spi = None

    @staticmethod
    def _clamp(v: float, limit: float) -> float:
        return max(-limit, min(limit, v))

    def _i16(self, v: float) -> int:
        iv = int(round(v))
        return max(-self.INT16_LIMIT, min(self.INT16_LIMIT, iv))

    def _build_packet(self, *, manual_active: bool, tracking: bool,
                      manual_omega: float, err_n: float, derr_n: float,
                      kp: float, max_omega: float, td: float) -> list[int]:
        """Pack the 16-byte control frame. err_n/derr_n are already normalised
        to ±1 (and per-second); everything is fixed-point int16 LE."""
        flags = (0x01 if manual_active else 0x00) | (0x02 if tracking else 0x00)
        body = [self.SYNC0, self.SYNC1, flags]
        body += list(struct.pack("<h", self._i16(
            self._clamp(manual_omega, self.OMEGA_LIMIT))))
        body += list(struct.pack("<h", self._i16(
            self._clamp(err_n, 1.0) * self.ERR_N_SCALE)))
        body += list(struct.pack("<h", self._i16(
            self._clamp(derr_n, self.DERR_N_LIMIT) * self.DERR_N_SCALE)))
        body += list(struct.pack("<h", self._i16(kp * self.KP_SCALE)))
        body += list(struct.pack("<h", self._i16(max_omega)))
        body += list(struct.pack("<h", self._i16(td * self.TD_SCALE)))
        checksum = 0
        for b in body:
            checksum ^= b
        return body + [checksum & 0xFF]

    def send_state(self, store, nx: float, dnx: float, half_width: float) -> None:
        """Push one control frame: manual override + camera-tracking inputs.

        Called every logic-thread iteration. The 16-byte transfer at 500 kHz
        takes ~320 µs; re-sending is harmless (the firmware has no packet
        timeout) and lets a one-off corrupted frame self-heal next call.

        `nx`/`dnx` are pixel error + its time derivative (px/sec) from the
        detector; `half_width` = frame_width/2. We normalise here so the
        firmware law stays resolution-independent.
        """
        if not (self.enabled and self.spi):
            return
        hw = half_width if half_width > 1.0 else 1.0
        err_n = nx / hw
        derr_n = dnx / hw
        try:
            self.spi.xfer2(self._build_packet(
                manual_active=bool(getattr(store, "manual_omega_active", False)),
                tracking=bool(getattr(store, "is_tracking", False)),
                manual_omega=float(getattr(store, "manual_omega", 0.0)),
                err_n=err_n,
                derr_n=derr_n,
                kp=float(getattr(store, "kp", 1.0)),
                max_omega=float(getattr(store, "max_omega", 40.0)),
                td=float(getattr(store, "td", 0.0)),
            ))
        except OSError:
            pass

    def close(self) -> None:
        if self.spi:
            try:
                # Leave the motor disabled on exit (everything zero/off).
                self.spi.xfer2(self._build_packet(
                    manual_active=False, tracking=False, manual_omega=0.0,
                    err_n=0.0, derr_n=0.0, kp=0.0, max_omega=0.0, td=0.0))
            except Exception:
                pass
            try:
                self.spi.close()
            except Exception:
                pass
            print("STM32 SPI port closed.")
