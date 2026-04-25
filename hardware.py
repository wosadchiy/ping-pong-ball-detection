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
        if self.enabled and self.ser and self.ser.is_open:
            try:
                msg = (
                    f"{ax:.2f},{ay:.2f},{int(nx)},{int(ny)},"
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
