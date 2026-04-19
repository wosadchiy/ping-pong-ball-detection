import cv2
import dearpygui.dearpygui as dpg
import time
import numpy as np
from threading import Thread, Lock

from config import ConfigStore
from camera import VideoStream
from hardware import ArduinoHandler
from detector import BallDetector
from ui import create_ui, update_texture
from utils import ema

# Контейнер для данных между потоками
class SharedBuffer:
    def __init__(self):
        self.frame = None
        self.mask = None
        self.logic_fps = 0
        self.tracking_data = (0, 0, 0, 0)
        self.lock = Lock()

shared = SharedBuffer()

def logic_thread_func(store, detector, arduino, vs):
    """Поток вычислений: Детекция + Arduino"""
    prev_time = time.time()
    fps_ema = 0
    
    while True:
        t_start = time.perf_counter()
        
        frame = vs.read()
        if frame is not None:
            # 1. Основные расчеты (без UI задержек)
            res_frame, res_mask, data = detector.process(frame, store)
            
            # 2. Передача в Arduino
            arduino.send_data(*data, store)
            arduino.receive_data()

            # 3. Расчет Logic FPS
            curr_time = time.time()
            fps_ema = ema(fps_ema, 1.0 / (curr_time - prev_time), 0.1)
            prev_time = curr_time

            # 4. Копируем результат для UI (только ссылки, чтобы быстро)
            with shared.lock:
                shared.frame = res_frame.copy()
                shared.mask = res_mask.copy()
                shared.logic_fps = int(fps_ema)
        
        # Небольшая пауза, чтобы не раскалять CPU до 100% впустую
        time.sleep(0.001)

def main():
    store = ConfigStore()
    arduino = ArduinoHandler()
    detector = BallDetector()
    vs = VideoStream(src=store.camera_id, store=store).start()
    
    create_ui(store)
    
    # Запускаем "Мозги" в отдельном потоке
    logic_thread = Thread(target=logic_thread_func, args=(store, detector, arduino, vs), daemon=True)
    logic_thread.start()

    show_mask = False
    
    # Главный поток: ТОЛЬКО отрисовка интерфейса (Render Loop)
    while dpg.is_dearpygui_running():
        if dpg.is_key_pressed(dpg.mvKey_M):
            show_mask = not show_mask
            dpg.configure_item("mask_window", show=show_mask)
        
        if dpg.is_key_pressed(dpg.mvKey_Q): break

        # Копируем данные из буфера логики для отрисовки
        with shared.lock:
            local_frame = shared.frame
            local_mask = shared.mask
            logic_fps = shared.logic_fps

        if local_frame is not None:
            # Обновляем текст
            dpg.set_value("ui_render_fps", f"Render FPS: {dpg.get_frame_rate()}")
            dpg.set_value("ui_logic_fps", f"Logic FPS: {logic_fps}")
            dpg.set_value("ui_status", f"Status: {'TRACKING' if store.is_tracking else 'IDLE'}")

            # Конвертация в RGBA (делаем в Main потоке, чтобы не тормозить Logic)
            rgba = cv2.cvtColor(local_frame, cv2.COLOR_BGR2RGBA)
            update_texture("camera_texture", rgba)
            
            if show_mask and local_mask is not None:
                m_rgba = cv2.cvtColor(local_mask, cv2.COLOR_GRAY2RGBA)
                update_texture("mask_texture", m_rgba)

        dpg.render_dearpygui_frame()

    vs.stop()
    arduino.close()
    dpg.destroy_context()

if __name__ == "__main__":
    main()