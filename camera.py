import shutil
import subprocess
import time
from threading import Thread

import cv2

import uvc_macos
from platform_utils import IS_LINUX, IS_MACOS, IS_RPI, configure_opencv_env, get_camera_backend

configure_opencv_env()

CAMERA_BACKEND = get_camera_backend()

# AVFoundation releases CoreMedia handles asynchronously, so reopening the
# same index right after a release() can crash the process or hang. Give it
# a moment to settle on macOS; on other backends the wait is unnecessary.
_CAM_RELEASE_DELAY = 0.3 if IS_MACOS else 0.0

# Print the "uvc-util missing" warning only once per process.
_UVC_WARNED = False

# /dev/videoN heuristic for V4L2-only helpers. OpenCV does not expose the
# underlying device path, so we reconstruct it from the integer index. This
# is correct for `cv2.VideoCapture(N, CAP_V4L2)` on every Pi we'll see.
def _v4l2_device_path(src) -> str | None:
    try:
        return f"/dev/video{int(src)}"
    except (TypeError, ValueError):
        return None


# Default capture geometry / framerate. Keep here, not deep in VideoStream,
# so external tools (bench, headless probe) can import the same numbers.
DEFAULT_CAPTURE_W = 640
DEFAULT_CAPTURE_H = 480
DEFAULT_CAPTURE_FPS = 120  # Pi targets 120 with MJPEG; macOS/Win simply ignore if camera caps out


def _try_set_mjpeg_pipeline(cap, width: int, height: int, fps: int) -> None:
    """On Linux/V4L2, ask the camera for MJPEG + size + framerate.

    Order matters with V4L2: fourcc must be set BEFORE width/height, otherwise
    some drivers silently fall back to YUYV at the previously-negotiated size.

    BUFFERSIZE=4 is critical for Pi 4 (and any host where MJPEG decode +
    numpy copy is comparable to the camera frame interval): with BUFFERSIZE=1
    the driver had to overwrite frames OpenCV hadn't dequeued yet, capping the
    effective FPS at ~half the camera's output. With BUFFERSIZE=4 the driver
    queues incoming frames and OpenCV catches up easily — empirically this
    flipped 60→117 fps at 640x480 MJPEG on the Pi 4 + Global Shutter Camera.

    Trade-off: up to 4 frames of latency if the consumer (logic thread) is
    really slow. Our capture thread loops with NO sleep, so it drains the
    queue continuously — actual sustained latency ~= 1 frame interval.

    Silent no-op on non-Linux / non-V4L2 paths. macOS AVFoundation and Windows
    DirectShow both pick a sensible default; forcing MJPEG there is either
    unsupported (AVFoundation) or sometimes hurts (DSHOW + cheap webcams).
    """
    if not IS_LINUX:
        return
    try:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FPS, float(fps))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(width))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(height))
        # Was 1 (latest-wins). On Pi 4 that capped MJPEG at 60 fps because
        # BGR conversion + numpy alloc in cap.read() (~10 ms) didn't fit
        # into the 8.3 ms frame interval — driver kept overwriting.
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 4)
    except cv2.error:
        pass


def _v4l2ctl(device: str, *args, timeout: float = 1.5) -> tuple[int, str, str]:
    """Tiny wrapper around `v4l2-ctl -d <device> ...`. Returns (rc, stdout, stderr).

    Silently degrades to (-1, "", "no v4l-utils") if the binary is missing.
    """
    if not shutil.which("v4l2-ctl"):
        return (-1, "", "no v4l-utils")
    try:
        out = subprocess.run(
            ["v4l2-ctl", "-d", device, *args],
            capture_output=True, text=True, timeout=timeout,
        )
        return (out.returncode, out.stdout.strip(), out.stderr.strip())
    except (subprocess.TimeoutExpired, OSError) as e:
        return (-1, "", str(e))


def _apply_v4l2_high_fps_controls(device: str, fps: int) -> None:
    """Push V4L2 controls that OpenCV's CAP_PROP_FPS can't reliably set.

    1. exposure_dynamic_framerate=0 — without this the driver is allowed to
       slow the stream down to fit the current exposure. With it set to 0
       and a manual exposure shorter than 1/fps, the driver will lock the
       advertised FPS.
    2. --set-parm=<fps> — actual VIDIOC_S_PARM ioctl, which OpenCV doesn't
       always issue. `cap.get(CAP_PROP_FPS)` may report 120 while the kernel
       streams at 60 because S_PARM was never sent.

    Either control may be missing on a given camera (different drivers), so
    we treat failures as advisory log lines, not errors.
    """
    if not (IS_LINUX and device):
        return

    rc, _out, _err = _v4l2ctl(device, "--set-ctrl=exposure_dynamic_framerate=0")
    if rc == 0:
        print("[v4l2] exposure_dynamic_framerate -> 0")
    elif rc != -1:
        # control just doesn't exist on this camera — that's fine.
        pass

    rc, out, err = _v4l2ctl(device, f"--set-parm={fps}")
    if rc == 0:
        # v4l2-ctl prints the actually negotiated frame interval here.
        actual = out.replace("\n", " ").strip()
        print(f"[v4l2] --set-parm={fps} -> {actual or 'OK'}")
    elif rc != -1:
        print(f"[v4l2] --set-parm={fps} failed: {err or out}")


# DirectShow exposure values are log2(seconds): -1 = 1/2 s, -6 = 1/64 s ≈ 15.6 ms.
# Settings written by the macOS/Windows builds use this convention. On V4L2
# CAP_PROP_EXPOSURE expects "100 µs units" (V4L2_CID_EXPOSURE_ABSOLUTE), so we
# need to translate. Also enforce a sane floor for high-FPS sessions.
def _dshow_to_microseconds(value: float) -> int:
    """Convert DirectShow log2(sec) into microseconds, clamped to UVC limits.

    Typical UVC range: 100 µs .. 5 000 000 µs. Clamping prevents accidental
    "store.exposure = -1" (=500 ms) from killing the stream after a settings
    file from another OS.
    """
    seconds = 2.0 ** float(value)
    us = int(round(seconds * 1_000_000))
    return max(100, min(us, 5_000_000))


def _apply_v4l2_exposure(cap, exposure_us: int, *, manual: bool = True) -> None:
    """Apply manual exposure on V4L2 in proper units.

    OpenCV CAP_PROP_AUTO_EXPOSURE on V4L2: 1 = Manual Mode, 3 = Aperture Priority.
    OpenCV CAP_PROP_EXPOSURE on V4L2:      V4L2_CID_EXPOSURE_ABSOLUTE in 100-µs units.
    """
    if not IS_LINUX:
        return
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0 if manual else 3.0)
    if manual:
        units_100us = max(1, exposure_us // 100)
        cap.set(cv2.CAP_PROP_EXPOSURE, float(units_100us))


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
    def __init__(
        self,
        src=0,
        store=None,
        width: int = DEFAULT_CAPTURE_W,
        height: int = DEFAULT_CAPTURE_H,
        fps: int = DEFAULT_CAPTURE_FPS,
    ):
        global _UVC_WARNED
        self.store = store
        self.requested_w = width
        self.requested_h = height
        self.requested_fps = fps
        self._device_path = _v4l2_device_path(src)
        self.cap = cv2.VideoCapture(src, CAMERA_BACKEND)

        # Linux path: MJPEG-first, latest-wins buffer, target 120 FPS. On the
        # Pi this is what unlocks high FPS — YUYV at 640x480 saturates USB 2.0
        # well before 60 FPS, MJPEG decodes via libjpeg-turbo + NEON in <2 ms.
        _try_set_mjpeg_pipeline(self.cap, width, height, fps)

        # Fallback / non-Linux size set. Harmless if MJPEG path already set it.
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(width))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(height))

        # Linux: turn off the V4L2 auto-priority that lets the driver slow
        # the stream down to suit the current exposure, and force the FPS via
        # VIDIOC_S_PARM (which OpenCV's CAP_PROP_FPS doesn't always do).
        if IS_LINUX and self._device_path:
            _apply_v4l2_high_fps_controls(self._device_path, fps)

        # AVFoundation uses a different convention for AUTO_EXPOSURE and most
        # UVC controls are simply not exposed by Apple's pipeline, so calling
        # `set` here usually has no effect and just spams warnings. Skip it.
        if not IS_MACOS and not IS_LINUX:
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)

        # Report what the driver actually negotiated. On Pi this reveals e.g.
        # "asked MJPG 120, got MJPG 60" for cameras that don't support the
        # higher rate at this resolution.
        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        try:
            actual_fcc = int(self.cap.get(cv2.CAP_PROP_FOURCC)).to_bytes(4, "little").decode(errors="ignore")
        except Exception:
            actual_fcc = "?"
        print(
            f"[camera] negotiated {actual_w}x{actual_h} @ {actual_fps:.1f} fps "
            f"fourcc={actual_fcc} (requested {width}x{height} @ {fps})"
        )

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

        if IS_LINUX:
            # V4L2 path: convert DSHOW-format `store.exposure` (log2 sec) to
            # microseconds and apply through CAP_PROP_AUTO_EXPOSURE=1 (manual)
            # + CAP_PROP_EXPOSURE in 100-µs units. This is the conversion
            # the legacy DSHOW value would otherwise be silently dropped at.
            exposure_us = _dshow_to_microseconds(self.store.exposure)
            _apply_v4l2_exposure(self.cap, exposure_us, manual=True)
            print(f"[camera] V4L2 manual exposure -> {exposure_us} µs")
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
        # Tight capture loop on purpose — no time.sleep on the success path.
        # The driver blocks cap.read() until the next frame is ready (~8 ms
        # at 120 fps), so this thread spends ~all its time in V4L2 syscalls,
        # not in Python. Combined with BUFFERSIZE=4, this lets the driver
        # queue frames whenever decode jitter pushes us above the frame
        # period — keeping sustained 100+ FPS on Pi 4.
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
                # read() failed — usually transient (USB unplug, format
                # mismatch). Brief sleep prevents a hot CPU spin if the
                # driver is in an error state.
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
