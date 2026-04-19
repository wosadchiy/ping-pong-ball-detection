import cv2
import dearpygui.dearpygui as dpg
import time
from threading import Thread, Lock
from config import ConfigStore
from camera import VideoStream, list_available_cameras
from hardware import ArduinoHandler
from detector import BallDetector
from ui import create_ui, update_texture
from utils import ema

class SharedBuffer:
    def __init__(self):
        self.frame = None
        self.mask = None
        self.logic_fps = 0
        self.status = "IDLE"
        self.lock = Lock()
        self.running = True

shared = SharedBuffer()

def logic_thread_func(store, detector, arduino, vs_container):
    prev_time = time.perf_counter()
    fps_ema = 0
    lost_frames = 0
    
    while shared.running:
        vs = vs_container[0]
        frame = vs.read()
        if frame is not None:
            # Получаем данные и статус
            res_frame, res_mask, data, found = detector.process(frame, store)
            
            # Шлем данные всегда (при потере там будет плавное затухание до 0)
            arduino.send_data(*data, store)
            arduino.receive_data()
            
            # Логика статуса (с гистерезисом 15 кадров, чтобы не мелькало)
            if found:
                lost_frames = 0
                st = "TRACKING" if store.is_tracking else "IDLE"
            else:
                lost_frames += 1
                st = "LOST" if lost_frames > 15 and store.is_tracking else ("TRACKING" if store.is_tracking else "IDLE")

            t_now = time.perf_counter()
            fps_ema = ema(fps_ema, 1.0 / (t_now - prev_time), 0.1)
            prev_time = t_now

            with shared.lock:
                shared.frame = res_frame
                shared.mask = res_mask
                shared.logic_fps = int(fps_ema)
                shared.status = st
        time.sleep(0.001)

def main():
    store = ConfigStore()
    arduino = ArduinoHandler()
    detector = BallDetector()
    available_cams = list_available_cameras()
    
    create_ui(store, available_cams)
    
    vs = VideoStream(src=store.camera_id, store=store).start()
    vs_container = [vs]
    
    Thread(target=logic_thread_func, args=(store, detector, arduino, vs_container), daemon=True).start()

    while dpg.is_dearpygui_running():
        if store.cam_id_changed:
            vs_container[0].stop()
            time.sleep(0.4)
            vs_container[0] = VideoStream(src=store.camera_id, store=store).start()
            store.cam_id_changed = False
            store.save_to_json()

        if store.hw_changed:
            vs_container[0].apply_hw_settings()
            store.hw_changed = False

        with shared.lock:
            local_frame = shared.frame
            local_mask = shared.mask
            logic_fps = shared.logic_fps
            local_status = shared.status

        if local_frame is not None:
            dpg.set_value("ui_render_fps", f"Render FPS: {dpg.get_frame_rate():.0f}")
            dpg.set_value("ui_logic_fps", f"Logic FPS: {logic_fps}")
            dpg.set_value("ui_status", f"Status: {local_status}")
            
            rgba = cv2.cvtColor(local_frame, cv2.COLOR_BGR2RGBA)
            update_texture("camera_texture", rgba)
            
            if dpg.is_item_shown("mask_window") and local_mask is not None:
                m_rgba = cv2.cvtColor(local_mask, cv2.COLOR_GRAY2RGBA)
                update_texture("mask_texture", m_rgba)

        dpg.render_dearpygui_frame()

    shared.running = False
    time.sleep(0.1)
    vs_container[0].stop()
    arduino.close()
    dpg.destroy_context()

if __name__ == "__main__":
    main()