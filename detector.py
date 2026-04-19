import cv2
import numpy as np
import math
from utils import ema

class BallDetector:
    def __init__(self):
        self.FOV_X, self.FOV_Y = 68.0, 46.0
        self.alpha = 0.25  # Скорость реакции (чем меньше, тем плавнее остановка)
        self.f_ax, self.f_ay, self.f_nx, self.f_ny = 0.0, 0.0, 0.0, 0.0

    def process(self, frame, store):
        h, w = frame.shape[:2]
        cx_f, cy_f = w // 2, h // 2
        
        # Оптимизация: чуть меньшее размытие для поднятия Logic FPS
        blurred = cv2.GaussianBlur(frame, (7, 7), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        lower = np.array([store.h_min, store.s_min, store.v_min], dtype=np.uint8)
        upper = np.array([store.h_max, store.s_max, store.v_max], dtype=np.uint8)
        
        mask = cv2.inRange(hsv, lower, upper)
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_cnt = None
        max_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500 and area > max_area:
                perimeter = cv2.arcLength(cnt, True)
                circularity = 4*math.pi*area/(perimeter**2) if perimeter > 0 else 0
                if circularity > 0.5:
                    max_area = area
                    best_cnt = cnt

        if best_cnt is not None:
            # ШАР НАЙДЕН: обновляем фильтры реальными координатами
            (cx, cy), radius = cv2.minEnclosingCircle(best_cnt)
            dx, dy = cx - cx_f, cy_f - cy
            
            self.f_ax = ema(self.f_ax, dx * (self.FOV_X / w), self.alpha)
            self.f_ay = ema(self.f_ay, dy * (self.FOV_Y / h), self.alpha)
            self.f_nx = ema(self.f_nx, max(-100, min(100, dx / cx_f * 100)), self.alpha)
            self.f_ny = ema(self.f_ny, max(-100, min(100, dy / cy_f * 100)), self.alpha)
            
            cv2.circle(frame, (int(cx), int(cy)), int(radius), (0, 255, 255), 2)
            cv2.drawMarker(frame, (int(cx), int(cy)), (0, 255, 255), cv2.MARKER_CROSS, 15, 2)
            
            found = True
        else:
            # ШАР ПОТЕРЯН: плавно тянем координаты к нулю
            self.f_ax = ema(self.f_ax, 0, self.alpha)
            self.f_ay = ema(self.f_ay, 0, self.alpha)
            self.f_nx = ema(self.f_nx, 0, self.alpha)
            self.f_ny = ema(self.f_ny, 0, self.alpha)
            found = False

        data = (self.f_ax, self.f_ay, self.f_nx, self.f_ny)
        return frame, mask, data, found