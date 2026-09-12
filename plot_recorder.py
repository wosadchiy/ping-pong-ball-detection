"""Time-series recorder for the trajectory plot.

Захватывает те же 4 канала, что видит пользователь в UI (nx, pot_px,
err→STM32, omega_out) + служебные (pot_raw, pot_vel, frame_age_us) с
таймстемпами. Кнопка REC в UI переключает старт/стоп; по стопу дампим
два файла в `plot_recordings/`:

* `plot_YYYYMMDD_HHMMSS[_LABEL].csv`  — плоские данные, легко разобрать
  numpy/pandas или Excel-подобным софтом.
* `plot_YYYYMMDD_HHMMSS[_LABEL].json` — метаданные: коэффициенты
  регулятора на момент записи, длительность, число сэмплов. Без них
  сравнение серий бессмысленно.

Логика умышленно синхронная и однонитевая: `append()` вызывается из
render-треда в том же месте, где обновляется plot_buf (60 Hz), а
`stop_and_save()` — из UI-callback. Никаких блокировок и очередей —
buffer это list, add в конец GIL-атомарен.
"""

from __future__ import annotations

import csv
import json
import os
import time
from dataclasses import dataclass, field
from datetime import datetime


RECORDINGS_DIR = "plot_recordings"

# Модуль-уровневый singleton. UI (ui.py) вешает callbacks кнопки REC на
# него; main.py (render-loop) пишет в него сэмплы. Один процесс — одна
# сессия записи, конкурентный доступ невозможен по конструкции.
INSTANCE = None  # type: ignore[assignment]  # инициализируется ниже

# Список параметров store, снапшот которых пишем в json-метадату. Всё,
# что реально влияет на форму графика на момент записи, — сюда. Порядок
# = порядок в json (dict сохраняет insertion order с Python 3.7).
META_PARAMS = (
    "kp", "td", "max_omega", "accel",
    "kd_pot", "pot_center", "pot_px_per_count",
    "predict_gain", "latency_offset_ms",
    "manual_omega", "is_tracking",
)


@dataclass
class PlotRecorder:
    """Single recording session. Не reentrant — по одной записи за раз."""

    active: bool = False
    label: str = ""
    start_wall: str = ""     # человекочитаемая метка (YYYYMMDD_HHMMSS)
    start_perf: float = 0.0  # perf_counter в момент старта — для «t от 0»
    samples: list = field(default_factory=list)

    def start(self, label: str = "") -> None:
        """Начинаем новую запись. Сбрасываем всё, что было."""
        self.active = True
        # Санитайзим метку: только a-zA-Z0-9_-, чтобы не убить файловую
        # систему. Пустая строка — норм, тогда просто без суффикса.
        self.label = "".join(
            c if (c.isalnum() or c in "_-") else "_"
            for c in (label or "").strip()
        )[:32]
        self.start_wall = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.start_perf = time.perf_counter()
        self.samples.clear()

    def append(self, *,
               t: float,
               nx: float,
               pot_px: float,
               err_pred_px: float,
               omega_out: int,
               pot_raw: int,
               pot_vel: int,
               frame_age_us: int) -> None:
        """Один сэмпл. Дергается из render-треда каждые ~16.7 мс.

        Никаких проверок active — вызывающая сторона сама решает, звать
        нас или нет (спасаем 60 if-ов в секунду).
        """
        self.samples.append((t, nx, pot_px, err_pred_px, omega_out,
                             pot_raw, pot_vel, frame_age_us))

    def status_text(self) -> str:
        """То, что UI показывает справа от кнопки REC."""
        if not self.active:
            return "Rec: idle"
        elapsed = time.perf_counter() - self.start_perf
        return f"Rec: {elapsed:5.1f}s / {len(self.samples)} pts"

    def stop_and_save(self, store) -> tuple[str, str] | None:
        """Останавливаем запись и пишем два файла на диск.

        Возвращаем (csv_path, json_path) или None если записи не было /
        буфер пустой (например, пользователь дважды нажал стоп).
        """
        if not self.active:
            return None
        self.active = False
        if not self.samples:
            return None

        os.makedirs(RECORDINGS_DIR, exist_ok=True)
        suffix = f"_{self.label}" if self.label else ""
        base = f"plot_{self.start_wall}{suffix}"
        csv_path = os.path.join(RECORDINGS_DIR, base + ".csv")
        json_path = os.path.join(RECORDINGS_DIR, base + ".json")

        # CSV: t относительно старта, чтобы файлы разных запусков были
        # сравнимы без выравнивания эпох. Колонки в порядке визуальной
        # важности для анализа.
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow([
                "t_s", "nx_px", "pot_px", "err_pred_px", "omega_out",
                "pot_raw", "pot_vel", "frame_age_us",
            ])
            for row in self.samples:
                w.writerow([f"{row[0]:.4f}", f"{row[1]:.2f}",
                            f"{row[2]:.2f}", f"{row[3]:.2f}",
                            row[4], row[5], row[6], row[7]])

        meta = {
            "recorded_at": self.start_wall,
            "label": self.label,
            "duration_s": self.samples[-1][0] - self.samples[0][0],
            "sample_count": len(self.samples),
            "sample_rate_hz_effective":
                len(self.samples) /
                max(1e-6, self.samples[-1][0] - self.samples[0][0]),
            "params": {p: getattr(store, p, None) for p in META_PARAMS},
        }
        with open(json_path, "w") as f:
            json.dump(meta, f, indent=2, ensure_ascii=False)

        return (csv_path, json_path)

INSTANCE = PlotRecorder()
