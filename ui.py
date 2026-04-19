import dearpygui.dearpygui as dpg
import numpy as np

def create_ui(store):
    dpg.create_context()

    # Словарь пресетов (h_min, h_max, s_min, v_min)
    # Оранжевый обновлен по твоим параметрам
    COLOR_PRESETS = {
        "Orange": (13, 35, 131, 184),
        "Yellow": (18, 46, 171, 98),
        "White":  (0, 179, 0, 180)
    }

    def apply_preset(name):
        vals = COLOR_PRESETS[name]
        
        # 1. Обновляем значения в оперативной памяти (Store)
        store.h_min, store.h_max = vals[0], vals[1]
        store.s_min, store.v_min = vals[2], vals[3]

        # 2. Синхронизируем визуальные слайдеры
        dpg.set_value("slider_h_min", store.h_min)
        dpg.set_value("slider_h_max", store.h_max)
        dpg.set_value("slider_s_min", store.s_min)
        dpg.set_value("slider_v_min", store.v_min)

        # 3. Сохраняем в settings.json автоматически
        store.save_to_json()
        print(f"Preset '{name}' applied and saved to JSON.")

    with dpg.texture_registry(show=False):
        dpg.add_dynamic_texture(width=640, height=480, default_value=np.zeros((480, 640, 4), dtype=np.float32), tag="camera_texture")
        dpg.add_dynamic_texture(width=640, height=480, default_value=np.zeros((480, 640, 4), dtype=np.float32), tag="mask_texture")

    with dpg.window(label="Dashboard", width=300, height=850, pos=[0, 0], no_close=True, no_move=True):
        
        with dpg.collapsing_header(label="TARGET SELECTION", default_open=True):
            dpg.add_text("Select Ball Color:")
            dpg.add_combo(items=list(COLOR_PRESETS.keys()), 
                          default_value="Orange", 
                          callback=lambda s, v: apply_preset(v))
            dpg.add_button(label="FORCE SAVE", width=-1, callback=store.save_to_json)

        with dpg.collapsing_header(label="FINE TUNING (HSV)", default_open=True):
            dpg.add_slider_int(label="H Min", tag="slider_h_min", min_value=0, max_value=179, default_value=store.h_min, callback=lambda s, v: setattr(store, 'h_min', v))
            dpg.add_slider_int(label="H Max", tag="slider_h_max", min_value=0, max_value=179, default_value=store.h_max, callback=lambda s, v: setattr(store, 'h_max', v))
            dpg.add_slider_int(label="S Min", tag="slider_s_min", min_value=0, max_value=255, default_value=store.s_min, callback=lambda s, v: setattr(store, 's_min', v))
            dpg.add_slider_int(label="V Min", tag="slider_v_min", min_value=0, max_value=255, default_value=store.v_min, callback=lambda s, v: setattr(store, 'v_min', v))

        with dpg.collapsing_header(label="STATISTICS", default_open=True):
            dpg.add_text("Render FPS: 0", tag="ui_render_fps", color=[0, 255, 0])
            dpg.add_text("Logic FPS: 0", tag="ui_logic_fps", color=[0, 255, 255])
        
        with dpg.collapsing_header(label="HARDWARE", default_open=True):
            dpg.add_checkbox(label="ENABLE TRACKING", default_value=store.is_tracking, callback=lambda s, v: setattr(store, 'is_tracking', v))
            dpg.add_slider_float(label="Kp Factor", min_value=0.0, max_value=5.0, default_value=store.kp, callback=lambda s, v: setattr(store, 'kp', v))
            dpg.add_slider_int(label="Max Speed", min_value=30, max_value=100, default_value=int(store.max_omega), callback=lambda s, v: setattr(store, 'max_omega', float(v)))

    with dpg.window(label="Camera Feed", pos=[310, 0], no_close=True):
        dpg.add_image("camera_texture")
    with dpg.window(label="Mask View", tag="mask_window", pos=[310, 520], show=False):
        dpg.add_image("mask_texture")

    dpg.create_viewport(title='BallTracker Pro v3.2', width=1000, height=900, vsync=False)
    dpg.setup_dearpygui()
    dpg.show_viewport()

INV_255 = np.float32(1.0 / 255.0)

def update_texture(tag, frame):
    data = (frame.astype(np.float32) * INV_255).flatten()
    dpg.set_value(tag, data)