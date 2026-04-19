import cv2
from threading import Thread
import time
import os

# Отключаем лишние логи Microsoft Media Foundation
os.environ["OPENCV_VIDEOIO_PRIORITY_MSMF"] = "0"

def list_available_cameras(max_to_test=3):
    """Сканирует порты до запуска основного потока"""
    available = []
    for i in range(max_to_test):
        # Используем CAP_DSHOW для Windows
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        if cap.isOpened():
            # Проверяем, отдаёт ли камера кадр
            ret, _ = cap.read()
            if ret:
                available.append(i)
            cap.release()
    return available if available else [0]

class VideoStream:
    def __init__(self, src=0, store=None):
        self.store = store
        self.cap = cv2.VideoCapture(src, cv2.CAP_DSHOW)
        
        # Настройки разрешения
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Ручное управление экспозицией
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25) 
        self.apply_hw_settings()

        self.grabbed, self.frame = self.cap.read()
        self.started = False
        self.thread = None

    def apply_hw_settings(self):
        if self.store and self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_EXPOSURE, self.store.exposure)

    def start(self):
        if self.started: return self
        self.started = True
        self.thread = Thread(target=self.update, daemon=True)
        self.thread.start()
        return self

    def update(self):
        while self.started:
            if not self.cap.isOpened():
                time.sleep(0.1)
                continue
                
            grabbed, frame = self.cap.read()
            if grabbed:
                self.frame = frame
            else:
                # Если кадр не считан, ждем чуть-чуть
                time.sleep(0.01)

    def read(self):
        return self.frame

    def stop(self):
        self.started = False
        if self.thread:
            self.thread.join(timeout=0.5)
        if self.cap.isOpened():
            self.cap.release()