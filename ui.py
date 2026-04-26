import dearpygui.dearpygui as dpg
import numpy as np


def _exposure_label(dshow_value: int) -> str:
    """Human-readable shutter time for the DirectShow log2-seconds scale."""
    seconds = 2.0 ** dshow_value
    if seconds >= 1.0:
        return f"{seconds:.2f} s"
    if seconds >= 1e-3:
        return f"{seconds * 1e3:.2f} ms"
    return f"{seconds * 1e6:.0f} us"


def _add_linked_value_control(
    label: str,
    tag_prefix: str,
    min_value,
    max_value,
    default_value,
    on_change,
    *,
    is_float: bool = False,
    fmt: str = "%.3f",
    step=1,
    step_fast=10,
    input_width: int = 85,
):
    """Render a slider paired with a numeric input field, kept in sync.

    Why both: the slider gives quick visual scrubbing, the input field lets you
    type a precise value (e.g. Kp = 1.235). Editing either widget updates the
    other AND calls `on_change(value)` exactly once.

    Notes:
      * `dpg.set_value()` does NOT fire the target widget's callback in
        DearPyGui, so the cross-updates below are recursion-safe.
      * The input has `on_enter=True` so we don't spam `on_change` on every
        keystroke; values commit when the user presses Enter or tabs away.
      * Both `min_clamped` / `max_clamped` are set so out-of-range typing is
        snapped to the slider's domain instead of breaking the slider.
    """
    slider_tag = f"slider_{tag_prefix}"
    input_tag = f"input_{tag_prefix}"

    add_slider = dpg.add_slider_float if is_float else dpg.add_slider_int
    add_input = dpg.add_input_float if is_float else dpg.add_input_int

    def _on_slider(_s, v):
        dpg.set_value(input_tag, v)
        on_change(v)

    def _on_input(_s, v):
        clamped = max(min_value, min(max_value, v))
        if clamped != v:
            dpg.set_value(input_tag, clamped)
        dpg.set_value(slider_tag, clamped)
        on_change(clamped)

    dpg.add_text(label)
    with dpg.group(horizontal=True):
        add_slider(
            tag=slider_tag,
            min_value=min_value,
            max_value=max_value,
            default_value=default_value,
            callback=_on_slider,
            width=-(input_width + 10),  # fill remaining row, leave room for input
        )
        input_kwargs = dict(
            tag=input_tag,
            default_value=default_value,
            callback=_on_input,
            min_value=min_value,
            max_value=max_value,
            min_clamped=True,
            max_clamped=True,
            on_enter=True,
            step=step,
            step_fast=step_fast,
            width=input_width,
        )
        if is_float:
            input_kwargs["format"] = fmt
        add_input(**input_kwargs)


def create_ui(store, available_cams):
    dpg.create_context()

    # Словарь пресетов (h_min, h_max, s_min, v_min)
    COLOR_PRESETS = {
        "Orange": (13, 35, 131, 100),
        "Yellow": (25, 40, 80, 100),
        "White":  (0, 179, 0, 180)
    }

    def apply_preset(name):
        vals = COLOR_PRESETS[name]
        store.h_min, store.h_max = vals[0], vals[1]
        store.s_min, store.v_min = vals[2], vals[3]
        
        # Обновляем ползунки в UI, чтобы они соответствовали пресету
        dpg.set_value("slider_h_min", store.h_min)
        dpg.set_value("slider_h_max", store.h_max)
        dpg.set_value("slider_s_min", store.s_min)
        dpg.set_value("slider_v_min", store.v_min)
        store.save_to_json()

    def toggle_mask_window(*_):
        """Show/hide the HSV mask preview window. Bound to button + key 'M'."""
        if dpg.is_item_shown("mask_window"):
            dpg.hide_item("mask_window")
        else:
            dpg.show_item("mask_window")

    # Реестр текстур для вывода видео
    with dpg.texture_registry(show=False):
        dpg.add_dynamic_texture(
            width=640, height=480, 
            default_value=np.zeros((480, 640, 4), dtype=np.float32), 
            tag="camera_texture"
        )
        dpg.add_dynamic_texture(
            width=640, height=480, 
            default_value=np.zeros((480, 640, 4), dtype=np.float32), 
            tag="mask_texture"
        )

    # Главное окно управления
    with dpg.window(label="Dashboard", width=300, height=850, pos=[0, 0], no_close=True, no_move=True):
        
        # СЕКЦИЯ 1: Камера и экспозиция
        with dpg.collapsing_header(label="HARDWARE SETUP", default_open=True):
            dpg.add_text("Camera Device:")
            dpg.add_combo(
                items=available_cams, 
                default_value=store.camera_id if store.camera_id in available_cams else available_cams[0],
                callback=lambda s, v: (setattr(store, 'camera_id', int(v)), setattr(store, 'cam_id_changed', True))
            )
            def _on_exposure(_s, v):
                store.update_hw("exposure", v)
                dpg.set_value("exposure_readout", f"Shutter: {_exposure_label(v)}")

            dpg.add_slider_int(
                label="Exposure",
                min_value=-13, max_value=-1,
                default_value=store.exposure,
                callback=_on_exposure,
            )
            dpg.add_text(
                f"Shutter: {_exposure_label(store.exposure)}",
                tag="exposure_readout",
                color=[180, 180, 180],
            )

        # СЕКЦИЯ 2: Выбор цели
        with dpg.collapsing_header(label="TARGET SELECTION", default_open=True):
            dpg.add_combo(
                items=list(COLOR_PRESETS.keys()), 
                default_value="Orange", 
                callback=lambda s, v: apply_preset(v)
            )

        # СЕКЦИЯ 3: Ручная подстройка цвета
        with dpg.collapsing_header(label="FINE TUNING (HSV)", default_open=True):
            dpg.add_slider_int(label="H Min", tag="slider_h_min", min_value=0, max_value=179, default_value=store.h_min, callback=lambda s, v: setattr(store, 'h_min', v))
            dpg.add_slider_int(label="H Max", tag="slider_h_max", min_value=0, max_value=179, default_value=store.h_max, callback=lambda s, v: setattr(store, 'h_max', v))
            dpg.add_slider_int(label="S Min", tag="slider_s_min", min_value=0, max_value=255, default_value=store.s_min, callback=lambda s, v: setattr(store, 's_min', v))
            dpg.add_slider_int(label="V Min", tag="slider_v_min", min_value=0, max_value=255, default_value=store.v_min, callback=lambda s, v: setattr(store, 'v_min', v))

        # СЕКЦИЯ 4: Телеметрия
        with dpg.collapsing_header(label="STATISTICS", default_open=True):
            # Camera FPS — honest, measured from successful cap.read() calls
            # in the capture thread. Differs from Logic FPS, which counts
            # detector iterations (and may re-process the same buffered
            # frame multiple times when the detector outruns the camera).
            dpg.add_text("Camera FPS: 0.0", tag="ui_cam_fps", color=[255, 200, 0])
            dpg.add_text("Render FPS: 0", tag="ui_render_fps", color=[0, 255, 0])
            dpg.add_text("Logic FPS: 0", tag="ui_logic_fps", color=[0, 255, 255])
        
        # СЕКЦИЯ 5: Моторы и приложение
        with dpg.collapsing_header(label="MOTOR & APP", default_open=True):
            # ENABLE TRACKING (motor on/off) and RECORD (CSV capture) sit
            # on the same row — they're the two main "live action" toggles
            # you reach for during a session. The "Open viewer" button next
            # to them launches the HTML graph viewer in the default browser
            # using the same `viewer_dir()` the recorder writes into, so
            # one click works the same in dev and prod.
            def _on_record_toggle(_s, v):
                store.is_recording = bool(v)
                store.recording_changed = True

            def _on_open_viewer(*_):
                # Imported lazily so ui.py stays loadable even if recorder.py
                # is broken (and so circular imports never bite us). No
                # success feedback in the UI on purpose: the browser
                # opening *is* the feedback, and `ui_record_status` is
                # already owned by the render loop (Rec: idle / Rec: 1.2s).
                from recorder import open_viewer_in_browser
                ok, msg = open_viewer_in_browser()
                if ok:
                    print(f"[ui] viewer opened: {msg}")
                else:
                    print(f"[ui] open viewer failed: {msg}")

            with dpg.group(horizontal=True):
                dpg.add_checkbox(
                    label="ENABLE TRACKING",
                    default_value=store.is_tracking,
                    callback=lambda s, v: setattr(store, 'is_tracking', v),
                )
                dpg.add_checkbox(
                    label="RECORD",
                    tag="ui_record_toggle",
                    default_value=False,
                    callback=_on_record_toggle,
                )
                dpg.add_button(
                    label="Open viewer",
                    tag="ui_open_viewer_btn",
                    callback=_on_open_viewer,
                )
                # Tooltip resolves the path lazily so it always reflects the
                # actual viewer location for the current run mode.
                with dpg.tooltip("ui_open_viewer_btn"):
                    from recorder import viewer_dir as _vd
                    dpg.add_text(
                        f"Opens {_vd() / 'index.html'} in your default browser.\n"
                        "Stages the bundled template into that folder first "
                        "if it isn't there yet (built app, first launch)."
                    )

            # Updated by the render loop in main.py via recorder.status().
            # Default text states "Idle" so the user sees something helpful
            # even before recording is ever started.
            dpg.add_text("Rec: idle", tag="ui_record_status",
                         color=[160, 160, 160])

            _add_linked_value_control(
                label="Kp Factor",
                tag_prefix="kp",
                min_value=0.0, max_value=5.0,
                default_value=float(store.kp),
                on_change=lambda v: setattr(store, 'kp', float(v)),
                is_float=True,
                fmt="%.3f",
                step=0.05,
                step_fast=0.5,
            )

            _add_linked_value_control(
                label="Max Speed",
                tag_prefix="max_omega",
                min_value=30, max_value=100,
                default_value=int(store.max_omega),
                on_change=lambda v: setattr(store, 'max_omega', float(v)),
                is_float=False,
                step=1,
                step_fast=10,
            )
            
            dpg.add_spacer(height=10)
            dpg.add_button(label="TOGGLE MASK VIEW (M)", width=-1, callback=toggle_mask_window)
            dpg.add_button(label="SAVE ALL SETTINGS", width=-1, callback=store.save_to_json)

        # СЕКЦИЯ 6: Тюнинг привода без камеры.
        #
        # Сценарий: камера физически снята с вала, нужно прогнать мотор на
        # разных скоростях/ускорениях и посмотреть где он срывается. Все
        # три контрола улетают на Ардуино как A/M/O команды (см. hardware
        # `_push_drive_tuning`), причём ТОЛЬКО при изменении значения, так
        # что серийная линия не забивается мусором между движениями
        # ползунка. Manual Override имеет приоритет над камерным
        # P-управлением, но физические кнопки на самой плате всё равно
        # перебивают всё.
        with dpg.collapsing_header(label="DRIVE TUNING", default_open=False):
            dpg.add_text(
                "Disconnect the camera from the motor shaft before using\n"
                "manual override. Acceleration is shared with camera mode\n"
                "(applies to every ramp, not just manual sweeps).",
                color=[160, 160, 160],
            )

            # Acceleration — sent to firmware as A<value>. Effective α is
            # capped by the current max_omega (≈ max_omega × 5 user/sec²)
            # because the firmware ramps via a 1 kHz timer with a one-idx
            # step of size max_omega/V_TABLE_N. Tooltip says so.
            _add_linked_value_control(
                label="Acceleration (units/sec^2)",
                tag_prefix="accel",
                min_value=10, max_value=500,
                default_value=int(round(store.accel)),
                on_change=lambda v: setattr(store, 'accel', float(v)),
                is_float=False,
                step=10, step_fast=50,
            )
            with dpg.tooltip("slider_accel"):
                dpg.add_text(
                    "Ramp rate the firmware is allowed to use when\n"
                    "transitioning from current omega to the target.\n"
                    "Effective max ~= 5 * Max Speed user-units/sec^2,\n"
                    "so increase Max Speed to unlock faster ramps."
                )

            def _on_manual_active_toggle(_s, v):
                store.manual_omega_active = bool(v)

            dpg.add_checkbox(
                label="MANUAL OMEGA OVERRIDE",
                tag="ui_manual_active",
                default_value=False,
                callback=_on_manual_active_toggle,
            )
            with dpg.tooltip("ui_manual_active"):
                dpg.add_text(
                    "ON: ignore camera, drive the motor straight from the\n"
                    "Manual omega slider below (good for characterising\n"
                    "the drive itself with the shaft disconnected).\n"
                    "OFF: normal P-control on pixel error from the camera."
                )

            _add_linked_value_control(
                label="Manual omega (units, signed)",
                tag_prefix="manual_omega",
                min_value=-100, max_value=100,
                default_value=int(round(store.manual_omega)),
                on_change=lambda v: setattr(store, 'manual_omega', float(v)),
                is_float=False,
                step=1, step_fast=10,
            )
            with dpg.tooltip("slider_manual_omega"):
                dpg.add_text(
                    "Direct omega target in user units. Sign = direction.\n"
                    "Only takes effect when 'MANUAL OMEGA OVERRIDE' is ON.\n"
                    "Final value is clamped on the Arduino side to\n"
                    "[-Max Speed, +Max Speed]."
                )

    # Окна для видеопотоков
    with dpg.window(label="Camera Feed", pos=[310, 0], no_close=True):
        dpg.add_image("camera_texture")
    
    with dpg.window(label="Mask View", tag="mask_window", pos=[310, 520], show=False):
        dpg.add_image("mask_texture")

    # Trajectory plot — live time-series of the X-axis ball delta (nx) that
    # is being sent to the Arduino. Units are PIXELS (-w/2..+w/2) because
    # that's what the firmware now consumes directly — see README and
    # `hardware.send_data`. For the default 640-wide capture we render a
    # ±340 window (≈6 % visual margin around ±320) so the curve never kisses
    # the axis edges; the X axis is a rolling 10-second window managed in
    # `main.py`. Pendulum motion in front of the camera should produce a
    # clean sinusoid here.
    with dpg.window(
        label="Trajectory: ball X delta -> Arduino",
        tag="trajectory_window",
        pos=[310, 520],
        width=640,
        height=300,
        no_close=True,
    ):
        with dpg.plot(label="", height=-1, width=-1, no_title=True):
            dpg.add_plot_legend()
            dpg.add_plot_axis(dpg.mvXAxis, label="t, s", tag="plot_x_axis")
            dpg.add_plot_axis(dpg.mvYAxis, label="X delta, px", tag="plot_y_axis")
            dpg.set_axis_limits("plot_y_axis", -340, 340)
            dpg.add_line_series(
                [], [],
                label="nx (px)",
                parent="plot_y_axis",
                tag="plot_nx_series",
            )

    # Глобальные горячие клавиши: 'M' переключает окно с маской.
    # Используем mvKey_M, чтобы код не зависел от ASCII-литералов.
    with dpg.handler_registry():
        dpg.add_key_press_handler(key=dpg.mvKey_M, callback=toggle_mask_window)

    dpg.create_viewport(title='BallTracker Pro v3.5', width=1000, height=900, vsync=False)
    dpg.setup_dearpygui()
    dpg.show_viewport()

# Утилита для обновления текстур
INV_255 = np.float32(1.0 / 255.0)

def update_texture(tag, frame):
    """Преобразование BGR/Gray кадра в текстуру float32 для Dear PyGui"""
    data = (frame.astype(np.float32) * INV_255).flatten()
    dpg.set_value(tag, data)