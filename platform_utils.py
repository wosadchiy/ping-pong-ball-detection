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


# Keywords used to recognise an Arduino-compatible USB-serial adapter.
# Matched case-insensitively against (description + manufacturer + device).
SERIAL_MATCH_KEYWORDS: tuple[str, ...] = (
    # Boards / vendors
    "arduino",
    "wch",            # CH340/CH341 manufacturer string on macOS
    "ch340",
    "ch341",
    "ftdi",
    "silicon labs",   # CP2102/CP2104
    "silabs",
    # Device-path fragments (mostly macOS / Linux)
    "usbmodem",       # Native USB MCUs (UNO R3 ATmega16u2, Leonardo, Nano Every, ...)
    "usbserial",      # Generic USB-UART bridges on macOS
    "wchusbserial",   # CH340 on macOS
    "slab_usbtouart", # CP210x on macOS
    "ttyusb",         # Linux: CH340/FTDI
    "ttyacm",         # Linux: native-USB Arduinos
)
