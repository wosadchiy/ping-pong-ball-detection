import time

import serial
import serial.tools.list_ports

from platform_utils import SERIAL_MATCH_KEYWORDS


class ArduinoHandler:
    def __init__(self, baudrate=115200):
        self.ser = None
        self.enabled = False
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
        the Arduino sketch to declare `normX/normY` as `float` and parse with
        `toFloat()` (was `toInt()`). See README / commit message.
        """
        if self.enabled and self.ser and self.ser.is_open:
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
