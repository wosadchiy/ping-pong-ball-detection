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
                # Открываем порт с коротким таймаутом
                self.ser = serial.Serial(port, baudrate, timeout=0.001)
                time.sleep(2)  # Ждем инициализации Arduino
                self.enabled = True
                print(f"Connected to Arduino: {port}")
            except Exception as e:
                print(f"Connection error: {e}")

    def find_arduino(self):
        ports = serial.tools.list_ports.comports()
        for p in ports:
            if "Arduino" in p.description or "CH340" in p.description:
                return p.device
        return None

    def send_data(self, ax, ay, nx, ny, store):
        # Проверяем, открыт ли порт физически
        if self.enabled and self.ser and self.ser.is_open:
            try:
                # Пакет: angleX, angleY, normX, normY, Kp, isTracking, MaxOmega
                msg = f"{ax:.2f},{ay:.2f},{int(nx)},{int(ny)},{store.kp:.2f},{int(store.is_tracking)},{store.max_omega:.1f}\n"
                self.ser.write(msg.encode())
            except (serial.SerialException, OSError):
                # Если порт закрылся во время записи, просто игнорируем ошибку
                pass

    def receive_data(self):
        if self.enabled and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode(errors="ignore").strip()
                    if line: print(f"ARDUINO: {line}")
            except:
                pass
            
    def close(self):
        if self.ser:
            self.ser.close()
            print("Serial port closed.")