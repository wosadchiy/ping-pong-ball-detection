import time

import serial
import serial.tools.list_ports

from platform_utils import SERIAL_MATCH_KEYWORDS


class ArduinoHandler:
    def __init__(self, baudrate=115200):
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

        port = self.find_arduino()
        if port:
            try:
                self.ser = serial.Serial(port, baudrate, timeout=0.001)
                time.sleep(2)  # Arduino boot delay after DTR reset
                self.enabled = True
                print(f"Connected to Arduino: {port}")
            except Exception as e:
                print(f"Connection error: {e}")
        else:
            print("Arduino not found. Tracking will run without serial output.")

    @staticmethod
    def find_arduino():
        """Locate the first Arduino-compatible USB-serial port across OSes.

        Windows -> COMx with "Arduino" / "CH340" in description.
        macOS   -> /dev/cu.usbmodem*, /dev/cu.wchusbserial*, /dev/cu.SLAB_*.
        Linux   -> /dev/ttyACM*, /dev/ttyUSB*.
        """
        for p in serial.tools.list_ports.comports():
            haystack = " ".join(
                filter(None, [p.description, p.manufacturer, p.device])
            ).lower()
            if any(kw in haystack for kw in SERIAL_MATCH_KEYWORDS):
                return p.device
        return None

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

    def send_data(self, ax, ay, nx, ny, store):
        """Push a single control packet to Arduino. Field layout:

            ax, ay   — degrees (FOV-based), kept for diagnostics / future use
            nx, ny   — *pixels* in (-w/2..+w/2), what the firmware actually
                       uses as `normX`/`normY` to drive the motor. We send
                       them as floats with two decimals so the motor sees
                       sub-pixel error coming from the EMA smoother.
            kp       — proportional gain (float)
            tracking — 0/1
            max_omega — speed cap

        NOTE: switching nx/ny from int (-100..100) to float pixels requires
        the Arduino sketch to declare `normX/normY` as `float` and parse
        with `toFloat()` (was `toInt()`). See README / commit message.

        Drive-tuning state (acceleration, manual override toggle, manual
        omega target) is sent OUT-OF-BAND via three separate prefixed
        commands (A/M/O). The 7-field CSV format below is unchanged so
        old firmware keeps working: it just won't see the new commands.
        """
        if not (self.enabled and self.ser and self.ser.is_open):
            return

        # Out-of-band drive-tuning updates first — they're cheap and only
        # fire on actual changes. Doing them BEFORE the CSV means an
        # accel/mode change reaches the firmware in the same TX burst as
        # the next regular packet, so the visible motor response is in
        # sync with the slider movement.
        self._push_drive_tuning(store)

        try:
            msg = (
                f"{ax:.2f},{ay:.2f},{nx:.2f},{ny:.2f},"
                f"{store.kp:.2f},{int(store.is_tracking)},{store.max_omega:.1f}\n"
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
