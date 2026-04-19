import json
import os
import sys

class ConfigStore:
    def __init__(self, filename="settings.json"):
        # Определяем путь к папке, где лежит .exe или .py файл
        if getattr(sys, 'frozen', False):
        # Если запущено как скомпилированный .exe
            base_path = os.path.dirname(sys.executable)
        else:
        # Если запущен обычный .py скрипт
            base_path = os.path.dirname(os.path.abspath(__file__))
            
        self.filepath = os.path.join(base_path, filename)
        self.camera_id = 0
        self.h_min, self.h_max = 20, 50
        self.s_min, self.s_max = 100, 255
        self.v_min, self.v_max = 50, 255
        self.exposure = -5
        self.gain = 100
        self.brightness = 30
        self.kp = 1.0
        self.is_tracking = False
        self.max_omega = 40.0
        
        self.hw_changed = False
        self.cam_id_changed = False
        self.load_from_json()

    def save_to_json(self):
        data = {k: v for k, v in self.__dict__.items() if not k.endswith('_changed')}
        try:
            with open(self.filepath, 'w') as f:
                json.dump(data, f, indent=4)
        except Exception as e:
            print(f"Save error: {e}")

    def load_from_json(self):
        if os.path.exists(self.filepath):
            try:
                with open(self.filepath, 'r') as f:
                    data = json.load(f)
                    for key, value in data.items():
                        if hasattr(self, key): setattr(self, key, value)
            except: print("Error parsing JSON")

    def update_hw(self, key, value):
        setattr(self, key, value)
        self.hw_changed = True

    def update_cam_id(self, val):
        if self.camera_id != val:
            self.camera_id = val
            self.cam_id_changed = True