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

class SharedBuffer:
    def __init__(self):
        self.frame = None
        self.mask = None
        self.logic_fps = 0
        self.lock = Lock()
        self.running = True

shared = SharedBuffer()

def logic_thread_func(store, detector, arduino, vs):
    prev_time = time.perf_counter()
    fps_ema = 0
    
    while shared.running:
        frame = vs.read()
        if frame is not None:
            # 1. Быстрая детекция
            res_frame, res_mask, data = detector.process(frame, store)
            
            # 2. Связь с Arduino
            arduino.send_data(*data, store)
            arduino.receive_data()

            # 3. Статистика скорости
            t_now = time.perf_counter()
            fps_ema = ema(fps_ema, 1.0 / (t_now - prev_time), 0.1)
            prev_time = t_now

            with shared.lock:
                shared.frame = res_frame
                shared.mask = res_mask
                shared.logic_fps = int(fps_ema)
        
        time.sleep(0.001)

def main():
    store = ConfigStore()
    arduino = ArduinoHandler()
    detector = BallDetector()
    vs = VideoStream(src=store.camera_id, store=store).start()
    
    create_ui(store)
    
    # Запуск logic-потока
    Thread(target=logic_thread_func, args=(store, detector, arduino, vs), daemon=True).start()

    while dpg.is_dearpygui_running():
        # Клавиша M - маска
        if dpg.is_key_pressed(dpg.mvKey_M):
            is_shown = dpg.is_item_shown("mask_window")
            dpg.configure_item("mask_window", show=not is_shown)
        
        # Клавиша Q - выход
        if dpg.is_key_pressed(dpg.mvKey_Q): break

        with shared.lock:
            local_frame = shared.frame
            local_mask = shared.mask
            logic_fps = shared.logic_fps

        if local_frame is not None:
            dpg.set_value("ui_render_fps", f"Render FPS: {dpg.get_frame_rate():.0f}")
            dpg.set_value("ui_logic_fps", f"Logic FPS: {logic_fps}")

            rgba = cv2.cvtColor(local_frame, cv2.COLOR_BGR2RGBA)
            update_texture("camera_texture", rgba)
            
            if dpg.is_item_shown("mask_window") and local_mask is not None:
                m_rgba = cv2.cvtColor(local_mask, cv2.COLOR_GRAY2RGBA)
                update_texture("mask_texture", m_rgba)

        dpg.render_dearpygui_frame()

    # Завершение
    shared.running = False
    time.sleep(0.1)
    vs.stop()
    arduino.close()
    dpg.destroy_context()

if __name__ == "__main__":
    main()