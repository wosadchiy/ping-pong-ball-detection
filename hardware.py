import serial
import serial.tools.list_ports
import time

class ArduinoHandler:
    def __init__(self, baudrate=115200):
        self.ser = None
        self.enabled = False
        port = self.find_arduino()
        if port:
            try:
                self.ser = serial.Serial(port, baudrate, timeout=0.001)
                time.sleep(2)
                self.enabled = True
                print(f"Connected to Arduino: {port}")
            except: pass

    def find_arduino(self):
        ports = serial.tools.list_ports.comports()
        for p in ports:
            if "Arduino" in p.description or "CH340" in p.description:
                return p.device
        return None

    def send_data(self, ax, ay, nx, ny, store):
        if self.enabled:
            msg = f"{ax:.2f},{ay:.2f},{int(nx)},{int(ny)},{store.kp:.2f},{int(store.is_tracking)},{store.max_omega:.1f}\n"
            self.ser.write(msg.encode())

    def receive_data(self):
        if self.enabled and self.ser.in_waiting:
            line = self.ser.readline().decode(errors="ignore").strip()
            if line: print(f"ARDUINO: {line}")
            
    def close(self):
        if self.ser: self.ser.close()