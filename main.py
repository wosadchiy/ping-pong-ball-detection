import argparse
import os
import sys

# Pi-friendly DISPLAY fallback. DearPyGui's GLFW backend reads DISPLAY at
# import time on Linux — without one, every dpg.* call in the import chain
# blows up with "Glfw Error 65544: X11: The DISPLAY environment variable
# is missing". On Pi running labwc (Wayland) the X11 socket :0 is exposed
# by Xwayland, and DPG opens its window there. We set the fallback BEFORE
# the dpg import so it takes effect.
if (
    sys.platform.startswith("linux")
    and not os.environ.get("DISPLAY")
    and not os.environ.get("WAYLAND_DISPLAY")
):
    os.environ["DISPLAY"] = ":0"
    os.environ.setdefault("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}")
    print(
        f"[main] no DISPLAY/WAYLAND_DISPLAY found — falling back to DISPLAY=:0 "
        f"(window will appear on Pi's connected monitor / VNC session, not in this SSH terminal)"
    )

import cv2
import dearpygui.dearpygui as dpg
import time
from collections import deque
from threading import Thread, Lock
from config import ConfigStore
from camera import (
    VideoStream,
    list_available_cameras,
    DEFAULT_CAPTURE_W,
    DEFAULT_CAPTURE_H,
    DEFAULT_CAPTURE_FPS,
)
from hardware import ArduinoHandler
# ADuC841 latency-mirror отключён по запросу — pipeline отлажен,
# измеренная задержка ~24-25 мс на α=0.5 / 120 fps. Чтобы вернуть
# зеркалирование на DAC0/DAC1 — раскомментируйте эту строку и
# 4 связанных блока ниже (помечены "ADuC OFF").
# from hardware import ArduinoHandler, AducHandler
from detector import BallDetector
from platform_utils import IS_MACOS, IS_RPI, apply_pi_tuning
from recorder import Recorder
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

def logic_thread_func(store, detector, arduino, vs_container, recorder):
    prev_time = time.perf_counter()
    fps_ema = 0
    # New-frame gating. Capture-thread updates `vs.frame` whenever V4L2 hands
    # us a fresh buffer; here we compare ndarray identity to skip redundant
    # detector.process calls on the SAME buffer. Result: logic_fps tracks
    # camera_fps almost 1:1 instead of being throttled by a fixed sleep.
    # The brief sleep below is just a "give the GIL away" yield while we
    # wait for the next frame; on Pi 4 with cap_fps=120 we hit it ~once
    # before each new frame → negligible CPU overhead.
    last_frame_id = None
    yield_sleep = 0.0005  # 0.5 ms — tight enough to never miss a frame, loose enough not to spin
    while shared.running:
        vs = vs_container[0]
        frame = vs.read()
        if frame is None or frame is last_frame_id:
            time.sleep(yield_sleep)
            continue
        last_frame_id = frame

        res_frame, res_mask, data = detector.process(frame, store)
        arduino.send_data(*data, store)
        arduino.receive_data()
        # ADuC OFF — было: aduc.send_dx_dy(data[2], data[3])
        # (зеркалирование nx,ny на DAC0/DAC1 для замера latency)
        t_now = time.perf_counter()
        fps_ema = ema(fps_ema, 1.0 / (t_now - prev_time), 0.1)
        prev_time = t_now
        # Record raw pixel deltas (slots 2,3 of `data`) at the full
        # logic-thread cadence — that's the most honest sample of what
        # the motor sees, no render-side downsampling. Recorder is
        # cheap when off (single boolean check, no lock).
        recorder.add_sample(float(data[2]), float(data[3]))
        with shared.lock:
            shared.frame = res_frame
            shared.mask = res_mask
            shared.logic_fps = int(fps_ema)
            # data == (ax, ay, nx, ny, dnx, dny); we plot nx — the
            # pixel-scale X delta that drives the P-part of the PD law.
            shared.nx = float(data[2])

def _parse_args():
    parser = argparse.ArgumentParser(description="Ball tracker")
    parser.add_argument(
        "--no-arduino",
        action="store_true",
        help="Не искать и не открывать serial-порт Arduino (dev-режим без железа).",
    )
    parser.add_argument(
        "--low-res",
        action="store_true",
        help=(
            "Захват камеры 320x240 вместо 640x480. Детектор работает в ~4 раза "
            "быстрее (полезно на Raspberry Pi). При прочих равных f_nx, f_ny и "
            "их производные становятся в 2 раза меньше — может потребоваться "
            "поднять Kp в UI ~×2, чтобы реакция мотора не упала."
        ),
    )
    parser.add_argument(
        "--cam-fps",
        type=int,
        default=DEFAULT_CAPTURE_FPS,
        help=f"Запрашиваемый FPS у камеры (default {DEFAULT_CAPTURE_FPS}).",
    )
    parser.add_argument(
        "--ui-fps",
        type=int,
        default=15,
        help=(
            "Cap render-loop FPS. Lower frees GIL for logic-thread. "
            "Pi 4: 15 is sweet spot (logic ~110, UI still responsive). "
            "Desktop: 60 is fine."
        ),
    )
    parser.add_argument(
        "--texture-fps",
        type=int,
        default=12,
        help=(
            "Cap camera/mask texture refresh rate (Hz). dpg.set_value with the "
            "float32 array holds GIL ~5-10 ms per call on Pi 4, so we update "
            "the picture less often than the rest of the UI. 12 Hz is enough "
            "for visual ball tracking; the detector still runs at full FPS."
        ),
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help=(
            "Не создавать UI. Поднимаются только capture + detector + serial + "
            "recorder. Полезно для прода на Pi 24/7 без монитора. Все настройки "
            "берутся из settings.json. Stop по Ctrl-C."
        ),
    )
    return parser.parse_args()


def run_headless(args, store, arduino, detector, *, capture_w: int, capture_h: int) -> None:
    """No-UI control loop. Capture + detector + serial + recorder.

    Keeps everything else identical to the GUI path: the same logic_thread,
    the same VideoStream, the same shared.lock for thread safety. Only
    difference: instead of dpg.render_dearpygui_frame() we sit in a plain
    print-loop that sleeps and shows stats once per second.

    Use case: production deployment on Pi 4 24/7, no monitor needed. All
    settings are loaded from settings.json (HSV, exposure, Kp, Td, etc.);
    they cannot be tweaked at runtime in this mode — edit the file and
    restart.
    """
    print("[headless] starting (no UI). Ctrl-C to stop.")
    if IS_MACOS:
        time.sleep(0.5)

    vs = VideoStream(
        src=store.camera_id,
        store=store,
        width=capture_w,
        height=capture_h,
        fps=args.cam_fps,
    ).start()
    recorder = Recorder()

    Thread(
        target=logic_thread_func,
        args=(store, detector, arduino, [vs], recorder),
        daemon=True,
    ).start()

    last_print = 0.0
    print_period = 1.0
    try:
        while shared.running:
            now = time.perf_counter()
            if now - last_print >= print_period:
                with shared.lock:
                    nx = shared.nx
                    logic_fps = shared.logic_fps
                rec_status = recorder.status()
                rec = (
                    f"  rec={rec_status['duration_sec']:.1f}s/{rec_status['samples']}pts"
                    if rec_status["recording"] else ""
                )
                print(
                    f"[headless] cam={vs.cam_fps:.0f}fps  logic={logic_fps}fps  "
                    f"nx={nx:+.1f}px  arduino={'on' if arduino.enabled else 'off'}"
                    f"  tracking={'on' if store.is_tracking else 'off'}{rec}"
                )
                last_print = now
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n[headless] Ctrl-C received, shutting down...")

    shared.running = False
    time.sleep(0.1)
    if recorder.is_recording:
        recorder.stop()
    vs.stop()
    arduino.close()


def main():
    args = _parse_args()
    apply_pi_tuning()  # no-op off Pi; on Pi tries to switch governor → performance

    capture_w, capture_h = (
        (320, 240) if args.low_res else (DEFAULT_CAPTURE_W, DEFAULT_CAPTURE_H)
    )
    if args.low_res:
        print(f"[main] --low-res: capture {capture_w}x{capture_h}")

    # UI texture size — decoupled from capture size. On Pi 4 the float32
    # RGBA copy + GL upload of a 640x480 texture eats ~80 ms per render
    # frame and starves the logic thread of GIL (~60 instead of ~115 FPS).
    # Halving the on-screen texture cuts that to ~20 ms; logic thread keeps
    # up with the camera. Detector still works at full capture resolution.
    ui_downsample = 2 if IS_RPI else 1
    ui_w = capture_w // ui_downsample
    ui_h = capture_h // ui_downsample
    if ui_downsample != 1:
        print(f"[main] UI texture {ui_w}x{ui_h} (downscaled from capture {capture_w}x{capture_h})")

    if IS_RPI:
        print("[main] running on Raspberry Pi — Pi-tuned defaults active")

    store = ConfigStore()
    arduino = ArduinoHandler(disabled=args.no_arduino)
    # ADuC OFF — было: aduc = AducHandler()
    # (FTDI-канал latency-зеркала, поднимал отдельный COM-порт)
    detector = BallDetector()
    
    # 1. СНАЧАЛА сканируем камеры (пока никто их не занял)
    available_cams = list_available_cameras()
    print(f"Found cameras: {available_cams}")

    # ----- HEADLESS PATH: no DPG, no GUI, just the control loop -----
    if args.headless:
        run_headless(
            args, store, arduino, detector,
            capture_w=capture_w, capture_h=capture_h,
        )
        return

    # 2. Инициализируем UI (передаем список камер)
    create_ui(store, available_cams, capture_w=capture_w, capture_h=capture_h, ui_w=ui_w, ui_h=ui_h)

    # 3. На macOS дать AVFoundation полностью отпустить ручки CoreMedia,
    #    освобождённые в list_available_cameras(), прежде чем переоткрывать
    #    тот же индекс в основном потоке.
    if IS_MACOS:
        time.sleep(0.5)

    # 4. ТОЛЬКО ТЕПЕРЬ открываем основной поток видео
    vs = VideoStream(
        src=store.camera_id,
        store=store,
        width=capture_w,
        height=capture_h,
        fps=args.cam_fps,
    ).start()
    vs_container = [vs]

    # Trajectory recorder — owned by main, sampled by the logic thread,
    # toggled by the UI checkbox via `store.recording_changed`.
    recorder = Recorder()

    # Запуск логики
    # ADuC OFF — раньше передавали `aduc` четвёртым аргументом.
    Thread(target=logic_thread_func, args=(store, detector, arduino, vs_container, recorder), daemon=True).start()

    # Trajectory plot bookkeeping. We sample `shared.nx` at PLOT_SAMPLE_HZ
    # (not at render rate) so the deque doesn't bloat when render FPS spikes
    # to 200+. The deque size cap is a safety net only — the time-based
    # popleft below is what actually defines the window.
    plot_t0 = time.perf_counter()
    plot_period = 1.0 / PLOT_SAMPLE_HZ
    plot_buf: deque = deque(maxlen=int(PLOT_WINDOW_SEC * PLOT_SAMPLE_HZ * 2))
    last_plot_sample = -1.0

    # Render-loop throttling. On Pi 4 the GLFW/Xwayland combo holds the GIL
    # during swap_buffers, so an unthrottled main loop pins ~one core AND
    # starves the logic thread (we measured logic FPS dropping from ~115 to
    # 60-90 with vsync=False free-running). 15 FPS is the sweet spot on Pi 4
    # — UI stays responsive, logic-thread keeps its GIL share. Configurable
    # via --ui-fps for desktop hosts that want smoother UI.
    render_target_fps = float(args.ui_fps)
    render_period = 1.0 / render_target_fps
    last_render_t = time.perf_counter()

    # Texture-update throttle, separate from render. dpg.set_value() with a
    # 1.2 MB float32 RGBA array holds the GIL during the memcpy into DPG's
    # internal texture buffer (~5-10 ms on Pi 4). Calling it at full render
    # rate ate ~25% of the GIL budget and held logic-thread back at 60-80
    # FPS. Configurable via --texture-fps.
    texture_period = 1.0 / float(args.texture_fps)
    last_texture_t = 0.0

    print(f"[main] UI render={args.ui_fps} fps  texture={args.texture_fps} fps")

    while dpg.is_dearpygui_running():
        if store.cam_id_changed:
            vs_container[0].stop()
            time.sleep(0.4)
            vs_container[0] = VideoStream(
                src=store.camera_id,
                store=store,
                width=capture_w,
                height=capture_h,
                fps=args.cam_fps,
            ).start()
            store.cam_id_changed = False
            store.save_to_json()

        if store.hw_changed:
            vs_container[0].apply_hw_settings()
            store.hw_changed = False

        # UI checkbox -> Recorder dispatch. We snapshot the current Kp /
        # max_omega / capture resolution into the metadata so the resulting
        # CSV (and HTML viewer) is self-explanatory weeks later.
        if store.recording_changed:
            store.recording_changed = False
            if store.is_recording:
                # Snapshot actual camera FPS at start. The driver can drift
                # later, but this gives a useful "the recording was made at
                # ~X FPS" headline. The recorder *also* computes the real
                # sample rate from len(samples)/duration on stop(), which is
                # what the logic thread actually delivered (usually higher
                # than the camera FPS because we re-process frames).
                meta = {
                    "kp": f"{store.kp:.3f}",
                    "max_omega": f"{store.max_omega:.1f}",
                    "resolution": f"{capture_w}x{capture_h}",
                    "source": "logic_thread",
                    "camera_fps": f"{vs_container[0].cam_fps:.1f}",
                }
                if recorder.start(meta) is None:
                    # Open failed — revert the checkbox so the UI stays
                    # honest about the actual recorder state.
                    store.is_recording = False
                    dpg.set_value("ui_record_toggle", False)
            else:
                recorder.stop()

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

            # Recording status — refreshed on every frame; cheap (one stat()).
            rec_status = recorder.status()
            if rec_status["recording"]:
                dpg.set_value(
                    "ui_record_status",
                    f"Rec: {rec_status['duration_sec']:.1f}s  "
                    f"{rec_status['samples']} pts  "
                    f"{rec_status['size_pretty']}",
                )
            else:
                dpg.set_value("ui_record_status", "Rec: idle")

            # Pi-friendly render path. Order matters for GIL hold time:
            #   1. resize BGR (small, ~0.5 ms via NEON INTER_AREA)
            #   2. cvtColor BGR->RGBA on the SMALL frame (~0.5 ms)
            #   3. float32 conversion in update_texture on the SMALL frame (~1-2 ms)
            # Doing cvtColor on full-res 640x480 first ate ~10 ms / render
            # frame, which is enough to starve logic-thread of GIL on Pi 4.
            # ALSO throttled to ~12 Hz: dpg.set_value() with the float32
            # buffer holds GIL ~5-10 ms; doing it at full render-rate kept
            # logic-thread starved.
            now_for_tex = time.perf_counter()
            if now_for_tex - last_texture_t >= texture_period:
                if (local_frame.shape[1], local_frame.shape[0]) != (ui_w, ui_h):
                    small = cv2.resize(local_frame, (ui_w, ui_h), interpolation=cv2.INTER_AREA)
                else:
                    small = local_frame
                rgba = cv2.cvtColor(small, cv2.COLOR_BGR2RGBA)
                update_texture("camera_texture", rgba)
                if dpg.is_item_shown("mask_window") and local_mask is not None:
                    if (local_mask.shape[1], local_mask.shape[0]) != (ui_w, ui_h):
                        small_mask = cv2.resize(local_mask, (ui_w, ui_h), interpolation=cv2.INTER_AREA)
                    else:
                        small_mask = local_mask
                    m_rgba = cv2.cvtColor(small_mask, cv2.COLOR_GRAY2RGBA)
                    update_texture("mask_texture", m_rgba)
                last_texture_t = now_for_tex

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

        # Sleep the rest of the frame budget. CRITICAL for Pi 4 — see comment
        # at the loop entry. Without this the main thread hogs the GIL.
        now = time.perf_counter()
        slept_until = last_render_t + render_period
        if slept_until > now:
            time.sleep(slept_until - now)
            last_render_t = slept_until
        else:
            last_render_t = now

    shared.running = False
    time.sleep(0.1)
    # Make sure an in-progress recording is closed cleanly so the CSV is
    # flushed and the HTML viewer is generated even if the user quits via
    # the window button.
    if recorder.is_recording:
        recorder.stop()
    vs_container[0].stop()
    arduino.close()
    # ADuC OFF — было: aduc.close()
    dpg.destroy_context()

if __name__ == "__main__":
    main()