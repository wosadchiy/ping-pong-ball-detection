import cv2
import numpy as np
import math
import serial
import serial.tools.list_ports
import time
import json
import os
from threading import Thread

# -----------------------------
# CONFIG STORE
# -----------------------------
class ConfigStore:
    def __init__(self, filepath="settings.json"):
        self.filepath = filepath
        # Default State
        self.camera_id = 0
        self.h_min, self.h_max = 20, 50
        self.s_min, self.s_max = 100, 255
        self.v_min, self.v_max = 50, 255
        self.exposure = -5
        self.gain = 100
        self.brightness = 30
        self.kp = 1.0
        self.is_tracking = False
        self.max_omega = 40.0  # New motor speed limit
        
        self.hw_changed = False
        self.cam_id_changed = False
        self.load_from_json()

    def save_to_json(self, *args):
        data = {
            "camera_id": self.camera_id,
            "h_min": self.h_min, "h_max": self.h_max,
            "s_min": self.s_min, "s_max": self.s_max,
            "v_min": self.v_min, "v_max": self.v_max,
            "exposure": self.exposure, "gain": self.gain,
            "brightness": self.brightness, "kp": self.kp,
            "is_tracking": self.is_tracking,
            "max_omega": self.max_omega
        }
        try:
            with open(self.filepath, 'w') as f:
                json.dump(data, f, indent=4)
            print(f"--- Settings saved to {self.filepath} ---")
        except Exception as e:
            print(f"Save error: {e}")

    def load_from_json(self):
        if os.path.exists(self.filepath):
            try:
                with open(self.filepath, 'r') as f:
                    data = json.load(f)
                    for key, value in data.items():
                        if hasattr(self, key): setattr(self, key, value)
                print(f"--- Settings loaded from {self.filepath} ---")
            except: 
                print("Error parsing JSON, using defaults.")

    def update_hw(self, key, value):
        setattr(self, key, value)
        self.hw_changed = True

    def update_cam_id(self, val):
        if self.camera_id != val:
            self.camera_id = val
            self.cam_id_changed = True

store = ConfigStore()

# -----------------------------
# MULTITHREADED VIDEO STREAM
# -----------------------------
class VideoStream:
    def __init__(self, src=0):
        self.src = src
        self.cap = cv2.VideoCapture(self.src)
        self.setup_camera()
        self.ret, self.frame = self.cap.read()
        self.stopped = False

    def setup_camera(self):
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FPS, 120)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.apply_hw_settings()

    def apply_hw_settings(self):
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, store.exposure)
        self.cap.set(cv2.CAP_PROP_GAIN, store.gain)
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, store.brightness)

    def change_source(self, new_src):
        self.stopped = True
        time.sleep(0.1)
        self.cap.release()
        self.src = new_src
        self.cap = cv2.VideoCapture(self.src)
        self.setup_camera()
        self.stopped = False
        Thread(target=self.update, args=(), daemon=True).start()

    def start(self):
        Thread(target=self.update, args=(), daemon=True).start()
        return self

    def update(self):
        while not self.stopped:
            self.ret, self.frame = self.cap.read()

    def read(self):
        return self.frame if self.ret else None

    def stop(self):
        self.stopped = True
        self.cap.release()

# -----------------------------
# SERIAL CONNECTION
# -----------------------------
def find_arduino():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if "Arduino" in p.description or "CH340" in p.description: return p.device
    return None

SERIAL_ENABLED = False
ser = None
port = find_arduino()
if port:
    try:
        ser = serial.Serial(port, 115200, timeout=0.001)
        time.sleep(2)
        SERIAL_ENABLED = True
        print(f"Connected to Arduino: {port}")
    except: pass

def ema(prev, new, a): return a * new + (1 - a) * prev

def send_to_arduino(ax, ay, nx, ny):
    if SERIAL_ENABLED:
        # Packet: angleX, angleY, normX, normY, Kp, isTracking, MaxOmega
        msg = f"{ax:.2f},{ay:.2f},{int(nx)},{int(ny)},{store.kp:.2f},{int(store.is_tracking)},{store.max_omega:.1f}\n"
        ser.write(msg.encode())

# -----------------------------
# SETTINGS UI (Refactored)
# -----------------------------
def create_settings_ui():
    cv2.namedWindow("Settings", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Settings", 450, 850)
    
    # --- GENERAL SECTION ---
    cv2.createTrackbar("[GENERAL]", "Settings", 0, 0, lambda x: None)
    cv2.createTrackbar("SAVE SETTINGS", "Settings", 0, 1, lambda v: (store.save_to_json() if v==1 else None, cv2.setTrackbarPos("SAVE SETTINGS", "Settings", 0)))
    cv2.createTrackbar("CAM ID", "Settings", store.camera_id, 4, store.update_cam_id)
    cv2.createTrackbar("FOLLOW BALL", "Settings", int(store.is_tracking), 1, lambda v: setattr(store, 'is_tracking', bool(v)))
    
    # --- CAMERA SETTINGS ---
    cv2.createTrackbar("[CAMERA]", "Settings", 0, 0, lambda x: None)
    cv2.createTrackbar("H Min", "Settings", store.h_min, 179, lambda v: setattr(store, 'h_min', v))
    cv2.createTrackbar("H Max", "Settings", store.h_max, 179, lambda v: setattr(store, 'h_max', v))
    cv2.createTrackbar("S Min", "Settings", store.s_min, 255, lambda v: setattr(store, 's_min', v))
    cv2.createTrackbar("V Min", "Settings", store.v_min, 255, lambda v: setattr(store, 'v_min', v))
    cv2.createTrackbar("Exposure", "Settings", abs(store.exposure), 13, lambda v: store.update_hw('exposure', -v))
    cv2.createTrackbar("Gain", "Settings", store.gain, 255, lambda v: store.update_hw('gain', v))
    
    # --- MOTOR SETTINGS ---
    cv2.createTrackbar("[MOTOR]", "Settings", 0, 0, lambda x: None)
    # Kp with 0.01 step
    cv2.createTrackbar("Kp x100", "Settings", int(store.kp * 100), 500, lambda v: setattr(store, 'kp', v / 100.0))
    # Max Omega (Range 30 to 100)
    cv2.createTrackbar("Max Speed", "Settings", int(store.max_omega), 100, lambda v: setattr(store, 'max_omega', float(max(30, v))))

# -----------------------------
# MAIN LOOP
# -----------------------------
vs = VideoStream(src=store.camera_id).start()
create_settings_ui()
cv2.namedWindow("Tracking")

FOV_X, FOV_Y = 68.0, 46.0
alpha = 0.25
f_ax, f_ay, f_nx, f_ny = 0, 0, 0, 0
last_data = (0.0, 0.0, 0, 0)
prev_time = time.time()
fps_smoothed = 0

while True:
    t_loop = time.time()
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'): break

    if store.hw_changed:
        vs.apply_hw_settings()
        store.hw_changed = False
    
    if store.cam_id_changed:
        vs.change_source(store.camera_id)
        store.cam_id_changed = False

    frame = vs.read()
    if frame is None:
        placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(placeholder, "Camera Search...", (200, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imshow("Tracking", placeholder)
        continue

    dt = t_loop - prev_time
    if dt > 0: fps_smoothed = ema(fps_smoothed, 1.0/dt, 0.01)
    prev_time = t_loop

    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    lower = np.array([store.h_min, store.s_min, store.v_min])
    upper = np.array([store.h_max, 255, 255])
    
    mask = cv2.inRange(hsv, lower, upper)
    kernel = np.ones((7,7), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    h, w = frame.shape[:2]
    cx_f, cy_f = w//2, h//2
    detected = False
    best_cnt = None
    max_area = 0

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 500 and area > max_area:
            perimeter = cv2.arcLength(cnt, True)
            circularity = 4*math.pi*area/(perimeter*perimeter) if perimeter > 0 else 0
            if circularity > 0.5:
                max_area = area; best_cnt = cnt

    if best_cnt is not None:
        (cx, cy), radius = cv2.minEnclosingCircle(best_cnt)
        detected = True
        dx, dy = cx - cx_f, cy_f - cy
        f_ax = ema(f_ax, dx * (FOV_X / w), alpha)
        f_ay = ema(f_ay, dy * (FOV_Y / h), alpha)
        f_nx = ema(f_nx, max(-100, min(100, dx / cx_f * 100)), alpha)
        f_ny = ema(f_ny, max(-100, min(100, dy / cy_f * 100)), alpha)
        last_data = (f_ax, f_ay, f_nx, f_ny)
        cv2.circle(frame, (int(cx), int(cy)), int(radius), (0, 255, 255), 2)

    if SERIAL_ENABLED and ser.in_waiting:
        line = ser.readline().decode(errors="ignore").strip()
        if line: print(f"ARDUINO: {line}")

    send_to_arduino(*last_data)

    cv2.putText(frame, f"FPS: {int(fps_smoothed)} | Kp: {store.kp:.2f} | MaxV: {int(store.max_omega)}", (10, h-20), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    cv2.imshow("Tracking", frame)
    cv2.imshow("Mask", mask)

vs.stop()
if SERIAL_ENABLED: ser.close()
cv2.destroyAllWindows()
###