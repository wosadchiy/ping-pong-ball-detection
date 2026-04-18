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
# CONFIG STORE (The "State")
# -----------------------------
class ConfigStore:
    def __init__(self, filepath="settings.json"):
        self.filepath = filepath
        
        # Default values (Initial State)
        self.h_min, self.h_max = 20, 50
        self.s_min, self.s_max = 100, 255
        self.v_min, self.v_max = 50, 255
        self.exposure = -5
        self.gain = 100
        self.brightness = 30
        self.kp = 1.5
        
        # Runtime flags
        self.hw_changed = False
        
        # Load saved data on initialization
        self.load_from_json()

    def save_to_json(self, *args):
        """Dumps current state to a JSON file."""
        # We only save detection, hardware, and logic parameters
        data = {
            "h_min": self.h_min, "h_max": self.h_max,
            "s_min": self.s_min, "s_max": self.s_max,
            "v_min": self.v_min, "v_max": self.v_max,
            "exposure": self.exposure,
            "gain": self.gain,
            "brightness": self.brightness,
            "kp": self.kp
        }
        try:
            with open(self.filepath, 'w') as f:
                json.dump(data, f, indent=4)
            print(f"Settings successfully saved to {self.filepath}")
        except Exception as e:
            print(f"Error saving settings: {e}")

    def load_from_json(self):
        """Loads data from JSON and updates the instance attributes."""
        if os.path.exists(self.filepath):
            try:
                with open(self.filepath, 'r') as f:
                    data = json.load(f)
                    # Update attributes using dict.update logic
                    for key, value in data.items():
                        if hasattr(self, key):
                            setattr(self, key, value)
                print(f"Settings loaded from {self.filepath}")
            except Exception as e:
                print(f"Error loading settings: {e}")
        else:
            print("No settings file found. Using default values.")

    def update_hw(self, key, value):
        """Internal setter for hardware params to flag hardware update."""
        setattr(self, key, value)
        self.hw_changed = True

# Initialize store
store = ConfigStore()

# -----------------------------
# SETTINGS UI (The "View")
# -----------------------------
def on_save_trigger(val):
    """Callback for the 'Save' trackbar."""
    if val == 1:
        store.save_to_json()
        # Reset trackbar to 0 after saving
        cv2.setTrackbarPos("SAVE SETTINGS", "Settings", 0)

def create_settings_ui():
    cv2.namedWindow("Settings", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Settings", 400, 700)
    
    # 1. Save Trigger (Acts as a button)
    cv2.createTrackbar("SAVE SETTINGS", "Settings", 0, 1, on_save_trigger)
    
    # 2. HSV Sliders
    cv2.createTrackbar("H Min", "Settings", store.h_min, 179, lambda v: setattr(store, 'h_min', v))
    cv2.createTrackbar("H Max", "Settings", store.h_max, 179, lambda v: setattr(store, 'h_max', v))
    cv2.createTrackbar("S Min", "Settings", store.s_min, 255, lambda v: setattr(store, 's_min', v))
    cv2.createTrackbar("V Min", "Settings", store.v_min, 255, lambda v: setattr(store, 'v_min', v))
    
    # 3. Hardware Sliders (using update_hw to notify dispatcher)
    cv2.createTrackbar("Exposure", "Settings", abs(store.exposure), 13, lambda v: store.update_hw('exposure', -v))
    cv2.createTrackbar("Gain", "Settings", store.gain, 255, lambda v: store.update_hw('gain', v))
    
    # 4. PID Controller
    cv2.createTrackbar("Kp x10", "Settings", int(store.kp * 10), 50, lambda v: setattr(store, 'kp', v / 10.0))

# -----------------------------
# MULTITHREADED CAMERA CLASS
# -----------------------------
class VideoStream:
    def __init__(self, src=0):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FPS, 120)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.apply_hw_settings()
        
        (self.ret, self.frame) = self.cap.read()
        self.stopped = False

    def apply_hw_settings(self):
        """Syncs hardware registers with store state."""
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, store.exposure)
        self.cap.set(cv2.CAP_PROP_GAIN, store.gain)
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, store.brightness)
        print(f"Hardware updated: Exp={store.exposure}, Gain={store.gain}")

    def start(self):
        Thread(target=self.update, args=(), daemon=True).start()
        return self

    def update(self):
        while not self.stopped:
            (self.ret, self.frame) = self.cap.read()

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True
        self.cap.release()

# -----------------------------
# HELPERS (Serial, EMA, etc.)
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
    except: pass

def ema(prev, new, a): return a * new + (1 - a) * prev

def send_to_arduino(ax, ay, nx, ny):
    if SERIAL_ENABLED:
        msg = f"{ax:.2f},{ay:.2f},{int(nx)},{int(ny)}\n"
        ser.write(msg.encode())

# -----------------------------
# EXECUTION
# -----------------------------
vs = VideoStream(src=0).start()
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
    
    # 1. Dispatcher: Apply HW changes if needed
    if store.hw_changed:
        vs.apply_hw_settings()
        store.hw_changed = False

    frame = vs.read()
    if frame is None: continue

    # 2. FPS
    dt = t_loop - prev_time
    if dt > 0:
        fps_smoothed = ema(fps_smoothed, 1.0/dt, 0.01)
    prev_time = t_loop

    # 3. Processing
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
                max_area = area
                best_cnt = cnt

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
    else:
        # Buffer logic
        if SERIAL_ENABLED:
            # Gradually reset coordinates if ball is lost for more than 3 frames
            pass

    # 4. Sync
    send_to_arduino(*last_data)

    # 5. UI Rendering
    cv2.putText(frame, f"FPS: {int(fps_smoothed)} | Kp: {store.kp}", (10, h-20), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.imshow("Tracking", frame)
    cv2.imshow("Mask", mask)

    if cv2.waitKey(1) & 0xFF == ord('q'): break

vs.stop()
cv2.destroyAllWindows()