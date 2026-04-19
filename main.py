import cv2
import time
import numpy as np

from config import ConfigStore
from camera import VideoStream
from hardware import ArduinoHandler
from detector import BallDetector
from ui import create_settings_ui, is_window_closed, draw_info_overlay
from utils import ema

def main():
    store = ConfigStore()
    arduino = ArduinoHandler()
    detector = BallDetector()
    vs = VideoStream(src=store.camera_id, store=store).start()
    
    # Инициализация интерфейса
    main_win = "Tracking"
    settings_win = "Settings"
    mask_win = "Mask"
    
    create_settings_ui(store)
    cv2.namedWindow(main_win)
    
    show_mask = False
    prev_time = time.time()
    fps_smoothed = 0

    try:
        while True:
            t_loop = time.time()
            
            # 1. Управление событиями
            key = cv2.waitKey(1) & 0xFF
            
            # Выход (Q или крестик)
            if key == ord('q') or is_window_closed(main_win) or is_window_closed(settings_win):
                break
            
            # Переключение маски (клавиша M)
            if key == ord('m'):
                show_mask = not show_mask
                if not show_mask: cv2.destroyWindow(mask_win)

            # 2. Обновление железа, если крутили ползунки
            if store.hw_changed:
                vs.apply_hw_settings()
                store.hw_changed = False
            if store.cam_id_changed:
                vs.change_source(store.camera_id)
                store.cam_id_changed = False

            # 3. Получение кадра из фонового потока
            frame = vs.read()
            if frame is None:
                placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(placeholder, "SEARCHING CAMERA...", (180, 240), 1, 1.5, (255,255,255), 2)
                cv2.imshow(main_win, placeholder)
                continue

            # 4. Обработка изображения
            processed_frame, mask, tracking_data = detector.process(frame, store)

            # 5. Расчет FPS и отрисовка текста
            dt = t_loop - prev_time
            if dt > 0: fps_smoothed = ema(fps_smoothed, 1.0/dt, 0.01)
            prev_time = t_loop
            
            draw_info_overlay(processed_frame, fps_smoothed, store)

            # 6. Связь с Arduino
            arduino.receive_data()
            arduino.send_data(*tracking_data, store)

            # 7. Вывод на экран
            cv2.imshow(main_win, processed_frame)
            if show_mask:
                cv2.imshow(mask_win, mask)

    finally:
        print("Stopping application...")
        vs.stop()
        arduino.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()