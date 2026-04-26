"""Application settings persisted to disk as JSON.

Where settings.json lives depends on whether we are running from source or
from a frozen PyInstaller bundle, AND on the host OS. The rules below match
what users on each platform expect:

    Dev (running `python main.py`)
        Always next to the source files (project root). Easy to inspect and
        edit while developing, easy to nuke with `git clean`.

    Frozen build on Windows
        Next to the executable, exactly like a portable app. Drop the folder
        anywhere, settings travel with it.

    Frozen build on macOS
        ~/Library/Application Support/<AppName>/settings.json
        macOS forbids writing inside the .app bundle (Gatekeeper / SIP /
        read-only DMG mounts), so we MUST keep state outside it. The
        bundle still ships a seed file in Contents/Resources/settings.json
        that gets copied on first launch if no user file exists yet.

    Frozen build on Linux
        ~/.config/<AppName>/settings.json (XDG Base Directory spec).
"""

from __future__ import annotations

import json
import os
import shutil
import sys
from pathlib import Path

APP_NAME = "BallTrackerPro"


def _is_frozen() -> bool:
    return getattr(sys, "frozen", False)


def _meipass_dir() -> Path | None:
    """PyInstaller's runtime extraction dir (read-only at app run-time)."""
    base = getattr(sys, "_MEIPASS", None)
    return Path(base) if base else None


def _writable_settings_path(filename: str) -> Path:
    """Pick the right user-writable location for `settings.json`.

    See the module docstring for the platform matrix.
    """
    if not _is_frozen():
        # Dev mode: keep it next to the source so contributors can poke it.
        return Path(__file__).resolve().parent / filename

    if sys.platform == "darwin":
        base = Path.home() / "Library" / "Application Support" / APP_NAME
    elif sys.platform.startswith("win"):
        # Stay portable on Windows: settings live next to the .exe so the
        # whole folder can be moved around like a self-contained app.
        base = Path(sys.executable).resolve().parent
    else:
        # Linux / *BSD — follow XDG.
        xdg = os.environ.get("XDG_CONFIG_HOME")
        base = Path(xdg) if xdg else Path.home() / ".config"
        base = base / APP_NAME

    base.mkdir(parents=True, exist_ok=True)
    return base / filename


def _bundled_seed_path(filename: str) -> Path | None:
    """Look for a default settings.json shipped INSIDE the frozen bundle."""
    if not _is_frozen():
        return None

    candidates: list[Path] = []
    meipass = _meipass_dir()
    if meipass:
        candidates.append(meipass / filename)

    if sys.platform == "darwin":
        # In a .app bundle the executable sits in Contents/MacOS, so
        # ../Resources is Contents/Resources — that's where we copied the
        # seed during the build (see tasks.py).
        exe_parent = Path(sys.executable).resolve().parent
        candidates.append(exe_parent.parent / "Resources" / filename)

    # Onedir / Windows portable layout: settings.json next to the exe.
    candidates.append(Path(sys.executable).resolve().parent / filename)

    for c in candidates:
        if c.is_file():
            return c
    return None


class ConfigStore:
    def __init__(self, filename: str = "settings.json"):
        self._filepath = _writable_settings_path(filename)

        # If the user has no settings yet AND the bundle ships a seed,
        # copy it so the first run feels populated instead of stock-default.
        if not self._filepath.exists():
            seed = _bundled_seed_path(filename)
            if seed and seed.resolve() != self._filepath.resolve():
                try:
                    shutil.copy2(seed, self._filepath)
                except OSError as e:
                    print(f"[config] could not seed settings from {seed}: {e}")

        self.camera_id = 0
        self.exposure = -5
        self.kp = 1.0
        self.is_tracking = False
        self.max_omega = 40.0

        # Drive-tuning controls (sent to Arduino as out-of-band A/M/O
        # commands — see hardware.py). `accel` is the desired ramp rate in
        # user-units/sec²; the firmware clamps it against the current
        # max_omega (effective α_max ≈ max_omega × 5 user-units/sec²).
        # `manual_omega_active` is intentionally NOT persisted (see
        # `save_to_json` exclude set below): we want every fresh launch to
        # start in safe camera-controlled mode, so a left-on override
        # checkbox can't surprise the user with a spinning shaft.
        self.accel = 100.0
        self.manual_omega_active = False
        self.manual_omega = 0.0

        # Привязка USB-UVC камеры на macOS (используется uvc-util для управления
        # экспозицией). Если камер UVC несколько — задайте либо часть имени
        # (например "Global Shutter"), либо точные vendor/product ID. При
        # ровно одной UVC-камере на шине эти поля можно не трогать.
        self.uvc_device_name = ""
        self.uvc_vendor_id = 0
        self.uvc_product_id = 0

        # HSV defaults (Orange).
        self.h_min, self.h_max = 13, 35
        self.s_min, self.s_max = 131, 255
        self.v_min, self.v_max = 100, 255

        self.hw_changed = False
        self.cam_id_changed = False

        # Trajectory recording — session-only state, never persisted.
        # `recording_changed` is a one-shot flag the UI sets so the main
        # render loop can dispatch a start()/stop() on the Recorder.
        self.is_recording = False
        self.recording_changed = False

        self.load_from_json()

    @property
    def filepath(self) -> str:
        """Absolute path of the active settings file (handy for the UI / logs)."""
        return str(self._filepath)

    def update_hw(self, key, value):
        """Метод для обновления параметров железа (экспозиция и т.д.)"""
        setattr(self, key, value)
        self.hw_changed = True

    def save_to_json(self):
        exclude = {
            "hw_changed", "cam_id_changed",
            "is_recording", "recording_changed",
            # Drive-tuning override is session-only: each launch starts
            # in camera mode regardless of how the previous session ended.
            "manual_omega_active",
        }
        data = {
            k: v for k, v in self.__dict__.items()
            if not k.startswith("_") and k not in exclude
        }
        try:
            with open(self._filepath, "w") as f:
                json.dump(data, f, indent=4)
        except Exception as e:
            print(f"Error saving config: {e}")

    def load_from_json(self):
        if self._filepath.exists():
            try:
                with open(self._filepath, "r") as f:
                    data = json.load(f)
                for k, v in data.items():
                    if hasattr(self, k):
                        setattr(self, k, v)
            except Exception as e:
                print(f"Error loading config: {e}")
