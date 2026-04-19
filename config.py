import json
import os
import sys

class ConfigStore:
    def __init__(self, filename="settings.json"):
        # Определяем путь к файлу рядом со скриптом
        if getattr(sys, 'frozen', False):
            base_path = os.path.dirname(sys.executable)
        else:
            base_path = os.path.dirname(os.path.abspath(__file__))
        
        self._filepath = os.path.join(base_path, filename)

        # Состояние по умолчанию
        self.camera_id = 0
        self.exposure = -5
        self.kp = 1.0
        self.is_tracking = False
        self.max_omega = 40.0
        
        # Настройки HSV (Orange по умолчанию)
        self.h_min, self.h_max = 13, 35
        self.s_min, self.s_max = 131, 255
        self.v_min, self.v_max = 100, 255

        # Служебные флаги (не сохраняются в JSON)
        self.hw_changed = False
        self.cam_id_changed = False

        self.load_from_json()

    def update_hw(self, key, value):
        """Метод для обновления параметров железа (экспозиция и т.д.)"""
        setattr(self, key, value)
        self.hw_changed = True

    def save_to_json(self):
        # Сохраняем всё, кроме служебных полей (начинаются с _) и флагов
        exclude = ['hw_changed', 'cam_id_changed']
        data = {k: v for k, v in self.__dict__.items() 
                if not k.startswith('_') and k not in exclude}
        try:
            with open(self._filepath, 'w') as f:
                json.dump(data, f, indent=4)
        except Exception as e:
            print(f"Error saving config: {e}")

    def load_from_json(self):
        if os.path.exists(self._filepath):
            try:
                with open(self._filepath, 'r') as f:
                    data = json.load(f)
                    for k, v in data.items():
                        if hasattr(self, k):
                            setattr(self, k, v)
            except Exception as e:
                print(f"Error loading config: {e}")