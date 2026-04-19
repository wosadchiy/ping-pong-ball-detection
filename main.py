import cv2
import numpy as np
import time

from config import ConfigStore
from camera import VideoStream
from hardware import ArduinoHandler
from ui import create_settings_ui
from detector import BallDetector
from utils import ema

def main():
    store = ConfigStore()
    arduino = ArduinoHandler()
    vs = VideoStream(src=store.camera_id, store=store).start()
    detector = BallDetector()
    
    create_settings_ui(store)
    cv2.namedWindow("Tracking")

    prev_time = time.time()
    fps_smoothed = 0

    try:
        while True:
            t_loop = time.time()
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'): break

            # Реакция на изменения в UI
            if store.hw_changed:
                vs.apply_hw_settings()
                store.hw_changed = False
            
            if store.cam_id_changed:
                vs.change_source(store.camera_id)
                store.cam_id_changed = False

            frame = vs.read()
            if frame is None:
                placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(placeholder, "Camera Search...", (200, 240), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.imshow("Tracking", placeholder)
                continue

            # Расчет FPS
            dt = t_loop - prev_time
            if dt > 0: 
                fps_smoothed = ema(fps_smoothed, 1.0/dt, 0.01)
            prev_time = t_loop

            # --- ОБРАБОТКА (Детектор) ---
            processed_frame, mask, tracking_data = detector.process_frame(frame, store)

            # --- СВЯЗЬ (Arduino) ---
            arduino.receive_data()
            arduino.send_data(*tracking_data, store)

            # --- ВИЗУАЛИЗАЦИЯ ---
            h = processed_frame.shape[0]
            cv2.putText(processed_frame, 
                        f"FPS: {int(fps_smoothed)} | Kp: {store.kp:.2f} | MaxV: {int(store.max_omega)}", 
                        (10, h-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            cv2.imshow("Tracking", processed_frame)
            cv2.imshow("Mask", mask)

    finally:
        vs.stop()
        arduino.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()