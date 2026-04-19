import cv2
import time
from threading import Thread

class VideoStream:
    def __init__(self, src=0, store=None):
        """
        Инициализация потока видео.
        :param src: ID камеры (0, 1, 2...)
        :param store: Объект ConfigStore для доступа к настройкам экспозиции и т.д.
        """
        self.src = src
        self.store = store
        self.cap = cv2.VideoCapture(self.src)
        self.setup_camera()
        
        # Читаем первый кадр, чтобы инициализировать переменную frame
        self.ret, self.frame = self.cap.read()
        self.stopped = False

    def setup_camera(self):
        """Базовые настройки формата и FPS"""
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FPS, 120)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        if self.store:
            self.apply_hw_settings()

    def apply_hw_settings(self):
        """Применение настроек яркости, усиления и экспозиции из store"""
        if not self.store:
            return
            
        # 0.25 переводит камеру в ручной режим на многих драйверах
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25) 
        self.cap.set(cv2.CAP_PROP_EXPOSURE, self.store.exposure)
        self.cap.set(cv2.CAP_PROP_GAIN, self.store.gain)
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, self.store.brightness)

    def change_source(self, new_src):
        """Переключение на другую камеру на лету"""
        self.stopped = True
        time.sleep(0.1) # Даем время потоку остановиться
        self.cap.release()
        
        self.src = new_src
        self.cap = cv2.VideoCapture(self.src)
        self.setup_camera()
        
        self.stopped = False
        Thread(target=self.update, args=(), daemon=True).start()

    def start(self):
        """Запуск отдельного потока для чтения кадров"""
        Thread(target=self.update, args=(), daemon=True).start()
        return self

    def update(self):
        """Цикл, который постоянно читает камеру в фоне"""
        while not self.stopped:
            self.ret, self.frame = self.cap.read()

    def read(self):
        """Возвращает последний захваченный кадр"""
        return self.frame if self.ret else None

    def stop(self):
        """Остановка потока и освобождение ресурсов"""
        self.stopped = True
        self.cap.release()