import time
from threading import Thread

import cv2

import uvc_macos
from platform_utils import IS_MACOS, configure_opencv_env, get_camera_backend

configure_opencv_env()

CAMERA_BACKEND = get_camera_backend()

# AVFoundation releases CoreMedia handles asynchronously, so reopening the
# same index right after a release() can crash the process or hang. Give it
# a moment to settle on macOS; on other backends the wait is unnecessary.
_CAM_RELEASE_DELAY = 0.3 if IS_MACOS else 0.0

# Print the "uvc-util missing" warning only once per process.
_UVC_WARNED = False


def _resolve_uvc_index(store) -> int | None:
    """Pick the uvc-util device index that corresponds to the active UVC camera.

    Selection order:
        1. `store.uvc_device_name` (substring match, if user pinned it)
        2. `(store.uvc_vendor_id, store.uvc_product_id)` exact match
        3. The single UVC device on the bus, if there is exactly one
        4. None — no UVC controls available
    """
    name = getattr(store, "uvc_device_name", "") if store else ""
    vid = getattr(store, "uvc_vendor_id", 0) if store else 0
    pid = getattr(store, "uvc_product_id", 0) if store else 0

    if name:
        idx = uvc_macos.find_index(name=name)
        if idx is not None:
            return idx
    if vid and pid:
        idx = uvc_macos.find_index(vendor_id=vid, product_id=pid)
        if idx is not None:
            return idx

    devices = uvc_macos.list_devices()
    if len(devices) == 1:
        return devices[0].index
    return None


def list_available_cameras(max_to_test=3):
    """Probe a few indices to find usable cameras BEFORE opening the main stream."""
    available = []
    for i in range(max_to_test):
        cap = cv2.VideoCapture(i, CAMERA_BACKEND)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                available.append(i)
            cap.release()
            if _CAM_RELEASE_DELAY:
                time.sleep(_CAM_RELEASE_DELAY)
    return available if available else [0]


class VideoStream:
    def __init__(self, src=0, store=None):
        global _UVC_WARNED
        self.store = store
        self.cap = cv2.VideoCapture(src, CAMERA_BACKEND)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # AVFoundation uses a different convention for AUTO_EXPOSURE and most
        # UVC controls are simply not exposed by Apple's pipeline, so calling
        # `set` here usually has no effect and just spams warnings. Skip it.
        if not IS_MACOS:
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)

        # On macOS we route exposure through `uvc-util` (USB Video Class API)
        # because AVFoundation does not propagate UVC controls. Resolve the
        # device index once and cache it.
        self._uvc_index: int | None = None
        if IS_MACOS:
            if uvc_macos.is_available():
                self._uvc_index = _resolve_uvc_index(store)
                if self._uvc_index is None and not _UVC_WARNED:
                    print(
                        "[uvc] no matching UVC device found; exposure controls "
                        "will be inactive on macOS"
                    )
                    _UVC_WARNED = True
            elif not _UVC_WARNED:
                print(
                    "[uvc] uvc-util binary not found — exposure controls are "
                    "inactive on macOS. See README -> 'macOS exposure setup'."
                )
                _UVC_WARNED = True

        self.apply_hw_settings()

        self.grabbed, self.frame = self.cap.read()
        self.started = False
        self.thread = None

        # Honest camera FPS: number of *successful* cap.read() calls per
        # second, sampled in the capture thread. This is the only counter that
        # reflects what AVFoundation/V4L2/DShow actually delivers — in
        # contrast to "Logic FPS" which can re-process the same buffered
        # frame many times per second when the detector is faster than the
        # camera. Updated once per second by `update()`. Atomic float write
        # under the GIL, no lock required.
        self.cam_fps: float = 0.0

    def apply_hw_settings(self):
        if not (self.store and self.cap.isOpened()):
            return

        # Cross-platform path: V4L2 / DirectShow honour CAP_PROP_EXPOSURE.
        # On macOS this is a no-op (left in for transparency / future Apple fix).
        self.cap.set(cv2.CAP_PROP_EXPOSURE, self.store.exposure)

        # macOS extra: drive the same value through uvc-util so that USB-UVC
        # cameras actually react to the slider.
        if IS_MACOS and self._uvc_index is not None:
            units = uvc_macos.dshow_to_uvc_units(self.store.exposure)
            uvc_macos.set_manual_exposure(self._uvc_index, units)

    def start(self):
        if self.started:
            return self
        self.started = True
        self.thread = Thread(target=self.update, daemon=True)
        self.thread.start()
        return self

    def update(self):
        # Sliding 1-second window for the FPS counter.
        frames_in_window = 0
        window_start = time.perf_counter()
        while self.started:
            if not self.cap.isOpened():
                time.sleep(0.1)
                continue

            grabbed, frame = self.cap.read()
            if grabbed:
                self.frame = frame
                frames_in_window += 1
                now = time.perf_counter()
                elapsed = now - window_start
                if elapsed >= 1.0:
                    self.cam_fps = frames_in_window / elapsed
                    frames_in_window = 0
                    window_start = now
            else:
                time.sleep(0.01)

    def read(self):
        return self.frame

    def stop(self):
        self.started = False
        if self.thread:
            self.thread.join(timeout=0.5)
        if self.cap.isOpened():
            self.cap.release()
            if _CAM_RELEASE_DELAY:
                time.sleep(_CAM_RELEASE_DELAY)
