import cv2
import numpy as np
import math
import serial
import serial.tools.list_ports
import time
from threading import Thread

# -----------------------------
# SETTINGS
# -----------------------------
dev_mode = True  
fps_alpha = 0.01  # Ultra-stable FPS filtering for high speeds
mask_window_name = "Mask"

# -----------------------------
# MOUSE CALLBACK (CHECKBOX)
# -----------------------------
def mouse_callback(event, x, y, flags, param):
    global dev_mode
    if event == cv2.EVENT_LBUTTONDOWN:
        # Toggle dev_mode if clicked in the top-left area
        if 10 < x < 160 and 10 < y < 50:
            dev_mode = not dev_mode
            if not dev_mode:
                # Close Mask window immediately when switching to OFF
                try:
                    cv2.destroyWindow(mask_window_name)
                except:
                    pass
            print(f"Dev Mode changed to: {dev_mode}")

# -----------------------------
# MULTITHREADED CAMERA CLASS
# -----------------------------
class VideoStream:
    """Separate thread for high-speed frame capturing."""
    def __init__(self, src=0):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FPS, 120)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Disable auto-exposure for manual control
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25) 
        
        # Exposure balance: lower value = sharper movement, less light
        self.cap.set(cv2.CAP_PROP_EXPOSURE, -5) 
        
        # Gain: increase brightness without adding motion blur
        self.cap.set(cv2.CAP_PROP_GAIN, 50) 
        
        # Soft-level brightness adjustment
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 30)
        
        (self.ret, self.frame) = self.cap.read()
        self.stopped = False

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
# SERIAL CONNECTION
# -----------------------------
def find_arduino():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if "Arduino" in p.description or "CH340" in p.description:
            return p.device
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
    except:
        print("Serial connection failed")

# -----------------------------
# CONSTANTS & FILTERS
# -----------------------------
# Camera FOV based on ELP-USBGS1200P02-LC1100 lens specs
FOV_X = 68.0  
FOV_Y = 46.0

# Broadened HSV range to capture shadowed parts of the ball (lower Saturation and Value)
yellow_lower = np.array([20, 100, 50]) 
yellow_upper = np.array([50, 255, 255])

alpha = 0.25  # Smoothing for coordinates
f_ax, f_ay, f_nx, f_ny = 0, 0, 0, 0
last_data = (0.0, 0.0, 0, 0)
lost_frames_cnt = 0
MAX_LOST = 3  # Frames to buffer last known position

fps_smoothed = 0
prev_time = time.time()

def ema(prev, new, a):
    return a * new + (1 - a) * prev

def send_to_arduino(ax, ay, nx, ny):
    if SERIAL_ENABLED:
        msg = f"{ax:.2f},{ay:.2f},{int(nx)},{int(ny)}\n"
        ser.write(msg.encode())

# -----------------------------
# START STREAM
# -----------------------------
vs = VideoStream(src=0).start()
cv2.namedWindow("Tracking")
cv2.setMouseCallback("Tracking", mouse_callback)
time.sleep(1.0)

# -----------------------------
# MAIN LOOP
# -----------------------------
while True:
    t_start = time.time()
    frame = vs.read()
    if frame is None: continue

    # FPS Calculation with EMA filtering
    dt = t_start - prev_time
    if dt > 0:
        fps_instant = 1.0 / dt
        fps_smoothed = ema(fps_smoothed, fps_instant, fps_alpha)
    prev_time = t_start

    h, w = frame.shape[:2]
    cx_f, cy_f = w//2, h//2

    # PRE-PROCESSING: Blur to remove texture/noise and unify the ball surface
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    # MASKING
    mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
    mask[hsv[:,:,2] > 240] = 0  # Ignore extreme glares
    
    # MORPHOLOGY: Open to remove noise, Close to fill internal holes
    kernel = np.ones((7,7), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    detected = False
    max_area = 0
    best_cnt = None

    # FIND LARGEST VALID CONTOUR
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 500: continue # Minimum size threshold
        
        if area > max_area:
            # Basic circularity check
            perimeter = cv2.arcLength(cnt, True)
            circularity = 4 * math.pi * area / (perimeter * perimeter) if perimeter > 0 else 0
            
            if circularity > 0.5:
                max_area = area
                best_cnt = cnt

    # DATA PROCESSING FOR THE LARGEST OBJECT
    if best_cnt is not None:
        (cx, cy), radius = cv2.minEnclosingCircle(best_cnt)
        detected = True
        dx, dy = cx - cx_f, cy_f - cy
        
        # Coordinate EMA Filtering
        f_ax = ema(f_ax, dx * (FOV_X / w), alpha)
        f_ay = ema(f_ay, dy * (FOV_Y / h), alpha)
        f_nx = ema(f_nx, max(-100, min(100, dx / cx_f * 100)), alpha)
        f_ny = ema(f_ny, max(-100, min(100, dy / cy_f * 100)), alpha)

        last_data = (f_ax, f_ay, f_nx, f_ny)
        lost_frames_cnt = 0
        
        if dev_mode:
            cv2.circle(frame, (int(cx), int(cy)), int(radius), (0, 255, 255), 2)
            cv2.putText(frame, f"D: {f_ax:.1f}, {f_ay:.1f}", (int(cx)-40, int(cy)+int(radius)+20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    # TRANSMISSION WITH BUFFER LOGIC
    if detected:
        send_to_arduino(*last_data)
    else:
        lost_frames_cnt += 1
        if lost_frames_cnt <= MAX_LOST:
            send_to_arduino(*last_data)
        else:
            last_data = (0.0, 0.0, 0, 0)
            send_to_arduino(0, 0, 0, 0)

    # SERIAL FEEDBACK
    if SERIAL_ENABLED and ser.in_waiting:
        try:
            line = ser.readline().decode(errors="ignore").strip()
            if line: print(f"ARDUINO: {line}")
        except: pass

    # UI RENDERING
    color = (0, 255, 0) if dev_mode else (0, 0, 255)
    cv2.rectangle(frame, (10, 10), (160, 45), color, -1)
    label = "DEV: ON" if dev_mode else "DEV: OFF"
    cv2.putText(frame, label, (25, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    if dev_mode:
        cv2.imshow(mask_window_name, mask)
        cv2.line(frame, (cx_f, 0), (cx_f, h), (0, 255, 0), 1)
        cv2.line(frame, (0, cy_f), (w, cy_f), (0, 255, 0), 1)
        cv2.putText(frame, f"FPS: {int(fps_smoothed)}", (w - 120, 35), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    cv2.imshow("Tracking", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

vs.stop()
if SERIAL_ENABLED: ser.close()
cv2.destroyAllWindows()