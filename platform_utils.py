"""Cross-platform helpers: OpenCV camera backend, serial-port matching, etc.

Centralises every place where the codebase has to know about the host OS, so
the rest of the code can stay platform-agnostic.
"""

from __future__ import annotations

import os
import sys

import cv2

IS_WINDOWS = sys.platform.startswith("win")
IS_MACOS = sys.platform == "darwin"
IS_LINUX = sys.platform.startswith("linux")


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
