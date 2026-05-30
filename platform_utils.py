"""Cross-platform helpers: OpenCV camera backend, serial-port matching, etc.

Centralises every place where the codebase has to know about the host OS, so
the rest of the code can stay platform-agnostic.
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import cv2

IS_WINDOWS = sys.platform.startswith("win")
IS_MACOS = sys.platform == "darwin"
IS_LINUX = sys.platform.startswith("linux")


def _detect_raspberry_pi() -> bool:
    """True iff we are running on Raspberry Pi hardware (any model).

    Reads /proc/device-tree/model — present on every Pi running mainline
    Debian/Raspbian since at least Buster. Cheap (single file read), no
    subprocess.
    """
    if not IS_LINUX:
        return False
    try:
        model = Path("/proc/device-tree/model").read_text(errors="ignore")
    except OSError:
        return False
    return "raspberry pi" in model.lower()


IS_RPI = _detect_raspberry_pi()


def apply_pi_tuning() -> None:
    """Best-effort runtime tuning for Raspberry Pi.

    1. Switch every CPU's `cpufreq` governor to `performance` so the cores
       hold a steady 1.5–1.8 GHz instead of bouncing on `ondemand`. Realtime
       CV pipelines benefit a lot — `ondemand` adds ~5–15 ms of latency
       jitter at low load.
    2. Pin OpenCV's thread pool to the number of physical cores so the
       internal scheduler doesn't fight `cv2.dnn` / our own threads.

    Silent no-op on non-Pi hosts. No-op (with a log line) if we don't have
    write permission to `scaling_governor` — the user can `sudo` once
    manually if they want this to stick across reboots.
    """
    if not IS_RPI:
        return

    cpus = sorted(Path("/sys/devices/system/cpu").glob("cpu[0-9]*"))
    changed = 0
    skipped_perm = 0
    for cpu in cpus:
        gov_path = cpu / "cpufreq" / "scaling_governor"
        if not gov_path.exists():
            continue
        try:
            current = gov_path.read_text().strip()
        except OSError:
            continue
        if current == "performance":
            continue
        try:
            gov_path.write_text("performance")
            changed += 1
        except PermissionError:
            skipped_perm += 1
        except OSError:
            pass

    if changed:
        print(f"[pi-tune] cpufreq governor → performance on {changed} CPU(s)")
    if skipped_perm:
        # cpupower is the path-of-least-resistance fallback when we don't
        # have direct sysfs write access (typical for non-root user with
        # default permissions on `scaling_governor`).
        try:
            subprocess.run(
                ["sudo", "-n", "cpupower", "frequency-set", "-g", "performance"],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=2,
            )
        except (FileNotFoundError, subprocess.TimeoutExpired):
            pass
        print(
            "[pi-tune] no permission to set cpufreq governor "
            f"on {skipped_perm} CPU(s). For stable FPS run once: "
            "`sudo cpupower frequency-set -g performance` "
            "(install: `sudo apt install linux-cpupower`)."
        )

    # OpenCV thread pool. Counter-intuitive on Pi 4: with 4 threads, every
    # cv2 call (GaussianBlur, morphologyEx, etc.) spawns up to 4 worker
    # pthreads, sync barriers and all — for a 640x480 frame that's pure
    # overhead. We already run 3 our own threads (capture, logic, render),
    # so OpenCV's thread pool fights ours for the same 4 cores. Setting
    # threads=1 means each cv2 call is sequential on the calling thread,
    # which empirically lifts logic FPS from ~70 to ~110+ on Pi 4. On
    # Pi 5 / x86 with bigger images, a higher number is better — keep
    # threads=2 there as a safe-but-not-too-many compromise.
    try:
        if "raspberry pi 4" in (Path("/proc/device-tree/model").read_text(errors="ignore").lower()
                                if Path("/proc/device-tree/model").exists() else ""):
            cv2.setNumThreads(1)
        else:
            cv2.setNumThreads(min(cv2.getNumberOfCPUs(), 2))
    except cv2.error:
        pass


def get_camera_backend() -> int:
    """Return the most suitable cv2.CAP_* backend for the current OS.

    Windows  -> DirectShow (stable UVC support, exposes most properties).
    macOS    -> AVFoundation (the only backend Apple exposes for UVC).
    Linux    -> V4L2.
    Other    -> CAP_ANY (let OpenCV pick).
    """
    if IS_WINDOWS:
        return cv2.CAP_DSHOW
    if IS_MACOS:
        return cv2.CAP_AVFOUNDATION
    if IS_LINUX:
        return cv2.CAP_V4L2
    return cv2.CAP_ANY


def configure_opencv_env() -> None:
    """Tweak OpenCV behaviour through env vars (must run before cv2 use).

    Currently only suppresses the noisy MSMF backend on Windows; on every
    other OS this is a no-op.
    """
    if IS_WINDOWS:
        os.environ.setdefault("OPENCV_VIDEOIO_PRIORITY_MSMF", "0")


# Keywords used to recognise USB-serial adapters of known board families.
# Each set is matched case-insensitively against (description + manufacturer
# + device). Matching is intentionally narrow per family so we don't accidentally
# pick up the wrong board when several USB-serial devices are plugged in.
#
# The ADuC841 latency-test board uses an FTDI bridge (e.g. FT232R), which on
# macOS shows up as `/dev/cu.usbserial-A100VKSF` etc. Older Arduino boards
# (Nano clones with CH340) show up as `/dev/cu.wchusbserial-XXXX` or as
# native USB modems (`usbmodem`). On the same Mac you can have both plugged
# in at once — splitting the keyword sets ensures `find_arduino()` /
# `find_aduc()` each return THEIR board, not the other.

ARDUINO_MATCH_KEYWORDS: tuple[str, ...] = (
    # Vendor / board names
    "arduino",
    "wch",            # CH340/CH341 manufacturer string on macOS
    "ch340",
    "ch341",
    "silicon labs",   # CP2102/CP2104 (often on cheap Nano clones)
    "silabs",
    # Device-path fragments
    "usbmodem",       # native-USB Arduinos (UNO R3, Leonardo, Nano Every)
    "wchusbserial",   # CH340 on macOS
    "slab_usbtouart", # CP210x on macOS
    "ttyacm",         # Linux: native-USB Arduinos
    # NOTE: deliberately NO "usbserial" / "ftdi" / "ttyusb" here — those would
    # match the ADuC841's FTDI bridge as well. If you have an FTDI-based
    # Arduino (rare these days), use the ARDUINO_PORT env var to force it.
)

ADUC_MATCH_KEYWORDS: tuple[str, ...] = (
    # Vendor names
    "ftdi",
    "future technology",  # full FTDI manufacturer string
    # Device-path fragments specific to FTDI on macOS — the suffix after
    # `usbserial-` is the chip's burned-in EEPROM serial. FT232R serials
    # typically start with 'A' (e.g. A100VKSF), FT232H with 'FT'.
    "usbserial-a",
    "usbserial-f",
    "usbserial-b",
)

# Backwards-compat alias — older code still imports the old name.
SERIAL_MATCH_KEYWORDS = ARDUINO_MATCH_KEYWORDS + ADUC_MATCH_KEYWORDS
