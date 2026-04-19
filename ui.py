import dearpygui.dearpygui as dpg
import numpy as np

def create_ui(store, available_cams):
    dpg.create_context()

    COLOR_PRESETS = {
        "Orange": (13, 35, 131, 255, 100, 255),
        "Yellow": (25, 40, 80, 255, 100, 255),
        "White":  (0, 179, 0, 50, 160, 255)
    }

    def apply_preset(name):
        vals = COLOR_PRESETS[name]
        store.h_min, store.h_max = vals[0], vals[1]
        store.s_min, store.s_max = vals[2], vals[3]
        store.v_min, store.v_max = vals[4], vals[5]
        
        for key in ["h_min", "h_max", "s_min", "s_max", "v_min", "v_max"]:
            dpg.set_value(f"slider_{key}", getattr(store, key))
        store.save_to_json()

    with dpg.texture_registry(show=False):
        dpg.add_dynamic_texture(width=640, height=480, default_value=np.zeros((480, 640, 4), dtype=np.float32), tag="camera_texture")
        dpg.add_dynamic_texture(width=640, height=480, default_value=np.zeros((480, 640, 4), dtype=np.float32), tag="mask_texture")

    with dpg.window(label="Dashboard", width=330, height=950, pos=[0, 0], no_close=True, no_move=True):
        
        with dpg.collapsing_header(label="HARDWARE SETUP", default_open=True):
            dpg.add_combo(items=available_cams, label="Camera", default_value=store.camera_id, 
                          callback=lambda s, v: (setattr(store, 'camera_id', int(v)), setattr(store, 'cam_id_changed', True)))
            dpg.add_slider_int(label="Exposure", min_value=-13, max_value=-1, default_value=store.exposure, 
                               callback=lambda s, v: store.update_hw('exposure', v))
            dpg.add_checkbox(label="SHOW MASK WINDOW", callback=lambda s, v: dpg.configure_item("mask_window", show=v))

        with dpg.collapsing_header(label="TARGET SELECTION", default_open=True):
            dpg.add_combo(items=list(COLOR_PRESETS.keys()), label="Preset", default_value="Orange", callback=lambda s, v: apply_preset(v))

        with dpg.collapsing_header(label="FINE TUNING (HSV)", default_open=True):
            dpg.add_slider_int(label="H Min", tag="slider_h_min", min_value=0, max_value=179, default_value=store.h_min, callback=lambda s, v: setattr(store, 'h_min', v))
            dpg.add_slider_int(label="H Max", tag="slider_h_max", min_value=0, max_value=179, default_value=store.h_max, callback=lambda s, v: setattr(store, 'h_max', v))
            dpg.add_slider_int(label="S Min", tag="slider_s_min", min_value=0, max_value=255, default_value=store.s_min, callback=lambda s, v: setattr(store, 's_min', v))
            dpg.add_slider_int(label="S Max", tag="slider_s_max", min_value=0, max_value=255, default_value=store.s_max, callback=lambda s, v: setattr(store, 's_max', v))
            dpg.add_slider_int(label="V Min", tag="slider_v_min", min_value=0, max_value=255, default_value=store.v_min, callback=lambda s, v: setattr(store, 'v_min', v))
            dpg.add_slider_int(label="V Max", tag="slider_v_max", min_value=0, max_value=255, default_value=store.v_max, callback=lambda s, v: setattr(store, 'v_max', v))

        with dpg.collapsing_header(label="STATISTICS", default_open=True):
            dpg.add_text("Render FPS: 0", tag="ui_render_fps", color=[0, 255, 0])
            dpg.add_text("Logic FPS: 0", tag="ui_logic_fps", color=[0, 255, 255])
            dpg.add_text("Status: IDLE", tag="ui_status", color=[255, 150, 0])
        
        with dpg.collapsing_header(label="MOTOR & APP", default_open=True):
            dpg.add_checkbox(label="ENABLE TRACKING", default_value=store.is_tracking, callback=lambda s, v: setattr(store, 'is_tracking', v))
            dpg.add_slider_float(label="Kp Factor", min_value=0.0, max_value=5.0, default_value=store.kp, callback=lambda s, v: setattr(store, 'kp', v))
            dpg.add_slider_int(label="Max Speed", min_value=30, max_value=100, default_value=int(store.max_omega), callback=lambda s, v: setattr(store, 'max_omega', float(v)))
            dpg.add_button(label="SAVE ALL", width=-1, callback=store.save_to_json)

    with dpg.window(label="Camera Feed", pos=[340, 0], no_close=True):
        dpg.add_image("camera_texture")
    
    with dpg.window(label="Mask View", tag="mask_window", pos=[340, 520], show=False):
        dpg.add_image("mask_texture")

    dpg.create_viewport(title='BallTracker Pro v3.8', width=1150, height=950, vsync=False)
    dpg.setup_dearpygui()
    dpg.show_viewport()

INV_255 = np.float32(1.0 / 255.0)
def update_texture(tag, frame):
    dpg.set_value(tag, (frame.astype(np.float32) * INV_255).flatten())