import cv2
import dearpygui.dearpygui as dpg
import time
from collections import deque
from threading import Thread, Lock
from config import ConfigStore
from camera import VideoStream, list_available_cameras # Импорт сканера
from hardware import ArduinoHandler
from detector import BallDetector
from platform_utils import IS_MACOS
from ui import create_ui, update_texture
from utils import ema

# How long the trajectory plot remembers (seconds) and how often we push a new
# point into it from the render thread. 60 Hz is overkill for a 1-Hz pendulum
# (Nyquist needs 2 Hz) but keeps the curve visually smooth without bloating
# the deque. 10 s × 60 Hz = ~600 points — trivial for DPG to render.
PLOT_WINDOW_SEC = 10.0
PLOT_SAMPLE_HZ = 60.0

class SharedBuffer:
    def __init__(self):
        self.frame = None
        self.mask = None
        self.logic_fps = 0
        # Latest X-axis ball delta in PIXELS (-w/2..+w/2), EMA-smoothed —
        # the same float the Arduino reads as `normX` and turns into
        # `omega = normX * Kp`. Sampled by the render thread to feed the
        # trajectory plot.
        self.nx = 0.0
        self.lock = Lock()
        self.running = True

shared = SharedBuffer()

def logic_thread_func(store, detector, arduino, vs_container):
    prev_time = time.perf_counter()
    fps_ema = 0
    while shared.running:
        vs = vs_container[0]
        frame = vs.read()
        if frame is not None:
            res_frame, res_mask, data = detector.process(frame, store)
            arduino.send_data(*data, store)
            arduino.receive_data()
            t_now = time.perf_counter()
            fps_ema = ema(fps_ema, 1.0 / (t_now - prev_time), 0.1)
            prev_time = t_now
            with shared.lock:
                shared.frame = res_frame
                shared.mask = res_mask
                shared.logic_fps = int(fps_ema)
                # data == (ax, ay, nx, ny); we plot nx — the percent-scale
                # X delta that drives the motor.
                shared.nx = float(data[2])
        time.sleep(0.001)

def main():
    store = ConfigStore()
    arduino = ArduinoHandler()
    detector = BallDetector()
    
    # 1. СНАЧАЛА сканируем камеры (пока никто их не занял)
    available_cams = list_available_cameras()
    print(f"Found cameras: {available_cams}")

    # 2. Инициализируем UI (передаем список камер)
    create_ui(store, available_cams)

    # 3. На macOS дать AVFoundation полностью отпустить ручки CoreMedia,
    #    освобождённые в list_available_cameras(), прежде чем переоткрывать
    #    тот же индекс в основном потоке.
    if IS_MACOS:
        time.sleep(0.5)

    # 4. ТОЛЬКО ТЕПЕРЬ открываем основной поток видео
    vs = VideoStream(src=store.camera_id, store=store).start()
    vs_container = [vs]
    
    # Запуск логики
    Thread(target=logic_thread_func, args=(store, detector, arduino, vs_container), daemon=True).start()

    # Trajectory plot bookkeeping. We sample `shared.nx` at PLOT_SAMPLE_HZ
    # (not at render rate) so the deque doesn't bloat when render FPS spikes
    # to 200+. The deque size cap is a safety net only — the time-based
    # popleft below is what actually defines the window.
    plot_t0 = time.perf_counter()
    plot_period = 1.0 / PLOT_SAMPLE_HZ
    plot_buf: deque = deque(maxlen=int(PLOT_WINDOW_SEC * PLOT_SAMPLE_HZ * 2))
    last_plot_sample = -1.0

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
            current_nx = shared.nx

        if local_frame is not None:
            dpg.set_value("ui_render_fps", f"Render FPS: {dpg.get_frame_rate():.0f}")
            dpg.set_value("ui_logic_fps", f"Logic FPS: {logic_fps}")
            # Honest camera FPS (read directly from the capture thread). This
            # is what AVFoundation/V4L2/DShow actually delivers, ignoring how
            # often we re-process the same frame in the logic loop.
            dpg.set_value("ui_cam_fps", f"Camera FPS: {vs_container[0].cam_fps:.1f}")
            rgba = cv2.cvtColor(local_frame, cv2.COLOR_BGR2RGBA)
            update_texture("camera_texture", rgba)
            if dpg.is_item_shown("mask_window") and local_mask is not None:
                m_rgba = cv2.cvtColor(local_mask, cv2.COLOR_GRAY2RGBA)
                update_texture("mask_texture", m_rgba)

            # Trajectory plot: throttle sampling to PLOT_SAMPLE_HZ and only
            # rebuild the line series when a new sample was actually added.
            # The X-axis still slides every render frame so the curve appears
            # to scroll smoothly even between samples.
            now_rel = time.perf_counter() - plot_t0
            if now_rel - last_plot_sample >= plot_period:
                plot_buf.append((now_rel, current_nx))
                cutoff = now_rel - PLOT_WINDOW_SEC
                while plot_buf and plot_buf[0][0] < cutoff:
                    plot_buf.popleft()
                last_plot_sample = now_rel
                xs = [p[0] for p in plot_buf]
                ys = [p[1] for p in plot_buf]
                dpg.set_value("plot_nx_series", [xs, ys])
            dpg.set_axis_limits(
                "plot_x_axis", now_rel - PLOT_WINDOW_SEC, now_rel
            )

        dpg.render_dearpygui_frame()

    shared.running = False
    time.sleep(0.1)
    vs_container[0].stop()
    arduino.close()
    dpg.destroy_context()

if __name__ == "__main__":
    main()