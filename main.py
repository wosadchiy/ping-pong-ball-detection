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
        self.lock = Lock()
        self.running = True  # Флаг для безопасного завершения потока

shared = SharedBuffer()

def logic_thread_func(store, detector, arduino, vs):
    """Поток вычислений: Детекция + Arduino"""
    prev_time = time.perf_counter()
    fps_ema = 0
    
    # Цикл работает, пока поднят флаг running
    while shared.running:
        frame = vs.read()
        if frame is not None:
            # 1. Основные расчеты
            res_frame, res_mask, data = detector.process(frame, store)
            
            # 2. Передача в Arduino (безопасно)
            arduino.send_data(*data, store)
            arduino.receive_data()

            # 3. Расчет Logic FPS
            curr_time = time.perf_counter()
            fps_ema = ema(fps_ema, 1.0 / (curr_time - prev_time), 0.1)
            prev_time = curr_time

            # 4. Передаем результаты в буфер для отрисовки
            with shared.lock:
                shared.frame = res_frame
                shared.mask = res_mask
                shared.logic_fps = int(fps_ema)
        
        # Минимальная пауза, чтобы не блокировать другие потоки
        time.sleep(0.001)
    
    print("Logic thread gracefully stopped.")

def main():
    store = ConfigStore()
    arduino = ArduinoHandler()
    detector = BallDetector()
    vs = VideoStream(src=store.camera_id, store=store).start()
    
    # Создаем интерфейс (vsync=False для максимальной производительности)
    create_ui(store)
    
    # Запускаем поток логики
    logic_thread = Thread(target=logic_thread_func, args=(store, detector, arduino, vs), daemon=True)
    logic_thread.start()

    show_mask = False
    
    # Главный поток: Рендеринг интерфейса
    while dpg.is_dearpygui_running():
        # Переключение маски клавишей M
        if dpg.is_key_pressed(dpg.mvKey_M):
            show_mask = not show_mask
            dpg.configure_item("mask_window", show=show_mask)
        
        # Выход клавишей Q
        if dpg.is_key_pressed(dpg.mvKey_Q):
            break

        # Копируем данные из буфера логики
        with shared.lock:
            local_frame = shared.frame
            local_mask = shared.mask
            logic_fps = shared.logic_fps

        if local_frame is not None:
            # Обновляем статистику в UI
            dpg.set_value("ui_render_fps", f"Render FPS: {dpg.get_frame_rate():.0f}")
            dpg.set_value("ui_logic_fps", f"Logic FPS: {logic_fps}")

            # Отрисовываем основную камеру
            rgba = cv2.cvtColor(local_frame, cv2.COLOR_BGR2RGBA)
            update_texture("camera_texture", rgba)
            
            # Отрисовываем маску только если окно открыто
            if show_mask and local_mask is not None:
                m_rgba = cv2.cvtColor(local_mask, cv2.COLOR_GRAY2RGBA)
                update_texture("mask_texture", m_rgba)

        dpg.render_dearpygui_frame()

    # --- ЗАВЕРШЕНИЕ РАБОТЫ ---
    print("Initiating shutdown...")
    shared.running = False  # Останавливаем поток логики
    time.sleep(0.1)        # Даем время на выход из цикла
    
    vs.stop()
    arduino.close()
    dpg.destroy_context()
    print("Application closed.")

if __name__ == "__main__":
    main()