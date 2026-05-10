import time

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
        self.alpha = 0.5
        self.f_ax, self.f_ay, self.f_nx, self.f_ny = 0.0, 0.0, 0.0, 0.0

        # ---- Derivative components for the PD regulator -----------------
        # Computed in pixels/sec from the RAW per-frame ball position
        # (pre-EMA), and only on actual NEW frames. This is critical:
        # the logic thread runs ~3–5x faster than the camera (1 kHz tick
        # vs 120 fps), so naively differentiating `f_nx` at every detector
        # call would inject huge spikes the moment a new frame arrives
        # (Δf_nx ≈ EMA half-step over Δt ≈ 1 ms ⇒ ~10x inflation of the
        # true derivative). Instead we:
        #   1. Sample `dx`/`dy` (raw) once per actual frame change.
        #   2. Compute (dx - prev_dx_raw)/Δt where Δt is the real camera
        #      period (~8 ms at 120 fps).
        #   3. Filter the resulting raw derivative through its own
        #      AGGRESSIVE EMA (`alpha_d`) — see below.
        #
        # alpha_d = 0.10 → first-order cutoff ≈ 2 Hz at 120 fps. We pick
        # this very low on purpose: D acts as a high-pass amplifier on
        # measurement noise (HSV mask edge "breathes" by ±2-3 px between
        # frames → ~±300 px/s noise in raw_dnx, comparable to true peak
        # signal). With cutoff > motor bandwidth (~5-10 Hz), the noise
        # passes straight through to the step generator and AM-modulates
        # step rate at multi-tens-of-Hz frequencies — driving sidebands
        # right into the NEMA17 resonance band (200-450 step/sec). The
        # motor can't physically follow such fast omega_target updates,
        # but the pulse-train DOES emit them as audible chatter.
        #
        # 1 Hz signal loses ~10% amplitude at this cutoff — compensate
        # with a slightly higher Td if needed. 0.5 Hz is essentially
        # untouched. For low-frequency tracking targets (≤ 1.5 Hz, our
        # use case) this is the right trade-off.
        self.alpha_d = 0.10
        # Hard floor on dt for the derivative update. AVFoundation can
        # occasionally deliver two camera frames within 1-2 ms of each
        # other (USB framing jitter). Without this guard, the next raw
        # derivative becomes (3 px / 0.001 s) = 3000 px/s — a ~10x spike
        # that the EMA smears across the next ~10 frames. 4 ms = half a
        # camera period at 120 fps; anything faster than that is treated
        # as the same logical sample and skipped.
        self._dt_min = 0.004
        self.dnx = 0.0
        self.dny = 0.0
        self._prev_dx_raw: float | None = None
        self._prev_dy_raw: float | None = None
        self._prev_t: float | None = None

        # last_data layout: (ax, ay, nx, ny, dnx, dny). Width-6 tuple is
        # consumed by `arduino.send_data(*data, store)` in main.py.
        self.last_data = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

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
            # This is what the Arduino reads as `normX`/`normY` and uses
            # for the P-part of the PD regulator.
            self.f_nx = ema(self.f_nx, dx, self.alpha)
            self.f_ny = ema(self.f_ny, dy, self.alpha)

            # Slots 4,5 — derivative of (nx, ny) in pixels/sec.
            #
            # We update D ONLY when the raw (dx, dy) actually changed —
            # i.e. when a NEW camera frame has arrived. Why bit-exact
            # equality is safe: cv2.minEnclosingCircle is deterministic,
            # and the same buffered frame (logic thread > camera rate)
            # produces byte-identical outputs across re-processings. So
            # `(dx, dy) != prev_raw` ⇔ "camera delivered a fresh frame".
            #
            # Then we filter raw_dnx through its own EMA (`alpha_d`) to
            # absorb single-pixel detector quantisation noise without
            # re-introducing the inter-frame `f_nx` aliasing problem.
            new_frame = (
                self._prev_dx_raw is None
                or dx != self._prev_dx_raw
                or dy != self._prev_dy_raw
            )
            if new_frame:
                now = time.perf_counter()
                if self._prev_t is not None and self._prev_dx_raw is not None:
                    dt = now - self._prev_t
                    # `_dt_min` rejects sub-frame USB jitter pairs; see
                    # __init__ for the rationale. Pairs that arrive too
                    # close together would inflate raw_dnx ~10x and the
                    # EMA would smear that spike across the next
                    # ~1/alpha_d frames of D-output.
                    if dt > self._dt_min:
                        raw_dnx = (dx - self._prev_dx_raw) / dt
                        raw_dny = (dy - self._prev_dy_raw) / dt
                        self.dnx = ema(self.dnx, raw_dnx, self.alpha_d)
                        self.dny = ema(self.dny, raw_dny, self.alpha_d)
                        # Только при УСПЕШНОМ обновлении сдвигаем «якорь»
                        # назад в _prev. Пропустили из-за dt-jitter — на
                        # следующем кадре посчитаем по более длинному
                        # интервалу, который заведомо превысит _dt_min.
                        self._prev_dx_raw = dx
                        self._prev_dy_raw = dy
                        self._prev_t = now
                else:
                    # Первый детект после потери трекинга / cold start.
                    self._prev_dx_raw = dx
                    self._prev_dy_raw = dy
                    self._prev_t = now

            self.last_data = (
                self.f_ax, self.f_ay,
                self.f_nx, self.f_ny,
                self.dnx, self.dny,
            )

            cv2.circle(frame, (int(cx), int(cy)), int(radius), (0, 255, 255), 2)
            # Detector centre cross — blue (BGR) so it doesn't bleed
            # into a yellow ball; the contour ring stays yellow.
            cv2.drawMarker(frame, (int(cx), int(cy)), (255, 0, 0), cv2.MARKER_CROSS, 15, 2)
        else:
            # Tracking gap — кадр без шарика. Сбрасываем D в ноль и
            # «забываем» предыдущую позицию, чтобы при возврате шарика
            # производная не выскочила огромной (за секунды отсутствия
            # шарик мог появиться где угодно — фактическая «производная»
            # бессмысленна). EMA-сглаженные nx/ny оставляем как были,
            # чтобы Arduino продолжал держать последнюю команду до
            # тайм-аута 250 мс (см. `cameraControl.ino`).
            self.dnx = 0.0
            self.dny = 0.0
            self._prev_dx_raw = None
            self._prev_dy_raw = None
            self._prev_t = None
            self.last_data = (
                self.f_ax, self.f_ay,
                self.f_nx, self.f_ny,
                self.dnx, self.dny,
            )

        # Зелёный крестик-репер в геометрическом центре кадра (cx_f, cy_f).
        # Это «точка нуля» ошибки: красный EMA-маркер должен совпадать с ним,
        # когда шарик идеально по центру. Рисуем ПЕРЕД красным, чтобы
        # динамический красный был поверх и не терялся, когда они
        # совпадают.
        cv2.drawMarker(frame, (cx_f, cy_f), (0, 255, 0),
                       cv2.MARKER_CROSS, 14, 1)

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