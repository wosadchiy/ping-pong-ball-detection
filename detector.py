import cv2
import numpy as np
import math
from utils import ema

class BallDetector:
    def __init__(self):
        # Camera FOV is kept here for `f_ax`/`f_ay` (degree-based diagnostics
        # that are still emitted in slots 0,1 of the serial frame for any
        # future use). The motor on Arduino now consumes the *pixel* values
        # in slots 2,3 (`f_nx`, `f_ny`) — see `detector.process` and
        # `hardware.send_data`. Raw pixels give ~30x finer effective
        # resolution than the previous percent (-100..100) representation.
        self.FOV_X, self.FOV_Y = 68.0, 46.0
        self.alpha = 0.25
        self.f_ax, self.f_ay, self.f_nx, self.f_ny = 0.0, 0.0, 0.0, 0.0
        self.last_data = (0.0, 0.0, 0.0, 0.0)

    def process(self, frame, store):
        h, w = frame.shape[:2]
        cx_f, cy_f = w // 2, h // 2
        
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        # Используем uint8 для максимальной производительности OpenCV
        lower = np.array([store.h_min, store.s_min, store.v_min], dtype=np.uint8)
        upper = np.array([store.h_max, 255, 255], dtype=np.uint8)
        
        mask = cv2.inRange(hsv, lower, upper)
        kernel = np.ones((7,7), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_cnt = None
        max_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500 and area > max_area:
                perimeter = cv2.arcLength(cnt, True)
                circularity = 4*math.pi*area/(perimeter*perimeter) if perimeter > 0 else 0
                if circularity > 0.5:
                    max_area = area
                    best_cnt = cnt

        if best_cnt is not None:
            (cx, cy), radius = cv2.minEnclosingCircle(best_cnt)
            dx, dy = cx - cx_f, cy_f - cy

            # Clamp pixel deltas to the half-frame range so a noisy detection
            # that briefly wanders outside the image never injects a giant
            # spike into the EMA — the smoother would take ages to recover.
            dx = max(-cx_f, min(cx_f, float(dx)))
            dy = max(-cy_f, min(cy_f, float(dy)))

            # Slots 0,1 — degree-based angles (kept for diagnostics and
            # backward compatibility; not consumed by the firmware anymore).
            self.f_ax = ema(self.f_ax, dx * (self.FOV_X / w), self.alpha)
            self.f_ay = ema(self.f_ay, dy * (self.FOV_Y / h), self.alpha)
            # Slots 2,3 — raw pixel deltas in (-w/2..+w/2) and (-h/2..+h/2).
            # This is what the Arduino reads as `normX`/`normY` and uses to
            # compute `omega = normX * Kp`. Float precision is preserved all
            # the way down the wire (`hardware.send_data` formats with %.2f).
            self.f_nx = ema(self.f_nx, dx, self.alpha)
            self.f_ny = ema(self.f_ny, dy, self.alpha)
            self.last_data = (self.f_ax, self.f_ay, self.f_nx, self.f_ny)
            
            cv2.circle(frame, (int(cx), int(cy)), int(radius), (0, 255, 255), 2)
            # Detector centre cross — blue (BGR) so it doesn't bleed
            # into a yellow ball; the contour ring stays yellow.
            cv2.drawMarker(frame, (int(cx), int(cy)), (255, 0, 0), cv2.MARKER_CROSS, 15, 2)

        # Red cross at the EMA-smoothed (nx, ny) — i.e. exactly the pair
        # that is shipped to the Arduino on every packet (matches the
        # trajectory plot). Drawn unconditionally so it stays visible
        # while the ball is briefly lost (firmware also sees the last
        # value). Y axis is flipped by the detector (positive ny =
        # above centre), so we subtract to convert back to image coords.
        sent_x = int(round(cx_f + self.f_nx))
        sent_y = int(round(cy_f - self.f_ny))
        cv2.drawMarker(frame, (sent_x, sent_y), (0, 0, 255),
                       cv2.MARKER_CROSS, 20, 2)

        return frame, mask, self.last_data