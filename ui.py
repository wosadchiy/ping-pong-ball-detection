import cv2

def create_settings_ui(store):
    cv2.namedWindow("Settings", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Settings", 450, 850)
    
    cv2.createTrackbar("[GENERAL]", "Settings", 0, 0, lambda x: None)
    cv2.createTrackbar("SAVE SETTINGS", "Settings", 0, 1, lambda v: (store.save_to_json() if v==1 else None, cv2.setTrackbarPos("SAVE SETTINGS", "Settings", 0)))
    cv2.createTrackbar("CAM ID", "Settings", store.camera_id, 4, store.update_cam_id)
    cv2.createTrackbar("FOLLOW BALL", "Settings", int(store.is_tracking), 1, lambda v: setattr(store, 'is_tracking', bool(v)))
    
    cv2.createTrackbar("[CAMERA]", "Settings", 0, 0, lambda x: None)
    cv2.createTrackbar("H Min", "Settings", store.h_min, 179, lambda v: setattr(store, 'h_min', v))
    cv2.createTrackbar("H Max", "Settings", store.h_max, 179, lambda v: setattr(store, 'h_max', v))
    cv2.createTrackbar("S Min", "Settings", store.s_min, 255, lambda v: setattr(store, 's_min', v))
    cv2.createTrackbar("V Min", "Settings", store.v_min, 255, lambda v: setattr(store, 'v_min', v))
    cv2.createTrackbar("Exposure", "Settings", abs(store.exposure), 13, lambda v: store.update_hw('exposure', -v))
    cv2.createTrackbar("Gain", "Settings", store.gain, 255, lambda v: store.update_hw('gain', v))
    
    cv2.createTrackbar("[MOTOR]", "Settings", 0, 0, lambda x: None)
    cv2.createTrackbar("Kp x100", "Settings", int(store.kp * 100), 500, lambda v: setattr(store, 'kp', v / 100.0))
    cv2.createTrackbar("Max Speed", "Settings", int(store.max_omega), 100, lambda v: setattr(store, 'max_omega', float(max(30, v))))