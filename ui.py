import dearpygui.dearpygui as dpg
import numpy as np

def create_ui(store):
    dpg.create_context()

    with dpg.texture_registry(show=False):
        dpg.add_dynamic_texture(width=640, height=480, default_value=np.zeros((480, 640, 4)), tag="camera_texture")
        dpg.add_dynamic_texture(width=640, height=480, default_value=np.zeros((480, 640, 4)), tag="mask_texture")

    with dpg.window(label="Dashboard", width=350, height=850, pos=[0, 0], no_close=True, no_move=True):
        
        with dpg.collapsing_header(label="STATISTICS", default_open=True):
            dpg.add_text("Render FPS: 0", tag="ui_render_fps", color=[0, 255, 0])
            dpg.add_text("Logic FPS: 0", tag="ui_logic_fps", color=[0, 255, 255])
            dpg.add_text("Status: IDLE", tag="ui_status")
        
        with dpg.collapsing_header(label="COLOR SETTINGS (HSV)", default_open=True):
            dpg.add_slider_int(label="H Min", min_value=0, max_value=179, default_value=store.h_min, callback=lambda s, v: setattr(store, 'h_min', v))
            dpg.add_slider_int(label="H Max", min_value=0, max_value=179, default_value=store.h_max, callback=lambda s, v: setattr(store, 'h_max', v))
            dpg.add_slider_int(label="S Min", min_value=0, max_value=255, default_value=store.s_min, callback=lambda s, v: setattr(store, 's_min', v))
            dpg.add_slider_int(label="V Min", min_value=0, max_value=255, default_value=store.v_min, callback=lambda s, v: setattr(store, 'v_min', v))

        with dpg.collapsing_header(label="MOTOR & APP", default_open=True):
            dpg.add_checkbox(label="ENABLE TRACKING", default_value=store.is_tracking, callback=lambda s, v: setattr(store, 'is_tracking', v))
            dpg.add_slider_float(label="Kp Factor", min_value=0.0, max_value=5.0, default_value=store.kp, callback=lambda s, v: setattr(store, 'kp', v))
            dpg.add_slider_int(label="Max Speed", min_value=30, max_value=100, default_value=int(store.max_omega), callback=lambda s, v: setattr(store, 'max_omega', float(v)))
            dpg.add_button(label="SAVE SETTINGS", width=-1, callback=store.save_to_json)

    with dpg.window(label="Camera Feed", pos=[360, 0], no_close=True):
        dpg.add_image("camera_texture")
    with dpg.window(label="Mask Stream", tag="mask_window", pos=[360, 525], show=False):
        dpg.add_image("mask_texture")

    dpg.create_viewport(title='Ball Tracker Pro v3.0', width=1100, height=900, vsync=True) # VSync для экрана
    dpg.setup_dearpygui()
    dpg.show_viewport()

INV_255 = 1.0 / 255.0

def update_texture(tag, frame):
    # Эта операция тяжелая, ее делаем только для экрана
    dpg.set_value(tag, (frame.astype(np.float32) * INV_255).flatten())