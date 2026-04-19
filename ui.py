import cv2

def create_settings_ui(store):
    win_name = "Settings"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_name, 450, 850)
    
    # [GENERAL]
    cv2.createTrackbar("SAVE SETTINGS", win_name, 0, 1, lambda v: (store.save_to_json() if v==1 else None, cv2.setTrackbarPos("SAVE SETTINGS", win_name, 0)))
    cv2.createTrackbar("CAM ID", win_name, store.camera_id, 4, store.update_cam_id)
    cv2.createTrackbar("FOLLOW BALL", win_name, int(store.is_tracking), 1, lambda v: setattr(store, 'is_tracking', bool(v)))
    
    # [CAMERA]
    cv2.createTrackbar("H Min", win_name, store.h_min, 179, lambda v: setattr(store, 'h_min', v))
    cv2.createTrackbar("H Max", win_name, store.h_max, 179, lambda v: setattr(store, 'h_max', v))
    cv2.createTrackbar("S Min", win_name, store.s_min, 255, lambda v: setattr(store, 's_min', v))
    cv2.createTrackbar("V Min", win_name, store.v_min, 255, lambda v: setattr(store, 'v_min', v))
    cv2.createTrackbar("Exposure", win_name, abs(store.exposure), 13, lambda v: store.update_hw('exposure', -v))
    
    # [MOTOR]
    cv2.createTrackbar("Kp x100", win_name, int(store.kp * 100), 500, lambda v: setattr(store, 'kp', v / 100.0))
    cv2.createTrackbar("Max Speed", win_name, int(store.max_omega), 100, lambda v: setattr(store, 'max_omega', float(max(30, v))))

def is_window_closed(win_name):
    return cv2.getWindowProperty(win_name, cv2.WND_PROP_VISIBLE) < 1

def draw_info_overlay(frame, fps, store):
    """Рисует данные (FPS, Kp и т.д.) прямо на кадре"""
    h, w = frame.shape[:2]
    info_text = f"FPS: {int(fps)} | Kp: {store.kp:.2f} | MaxV: {int(store.max_omega)}"
    # Рисуем черную подложку для текста (чтобы было видно на любом фоне)
    cv2.putText(frame, info_text, (11, h - 19), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
    # Рисуем сам текст
    cv2.putText(frame, info_text, (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)