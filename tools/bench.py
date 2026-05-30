"""Headless performance bench for the tracker pipeline.

Запускать БЕЗ открытого основного приложения (камера занята). Скрипт
последовательно прогоняет несколько сценариев и сравнивает их.

    1. v4l2-ctl: какие форматы и FPS реально поддерживает камера.
    2. Capture в default-режиме (auto-exposure).
    3. Capture с явной ручной экспозицией (по умолчанию 10 мс).
    4. Capture + detector в том же режиме что и stage 3.

Запуск:
    task bench
    # или напрямую:
    python -m tools.bench
    python -m tools.bench --low-res
    python -m tools.bench --no-mjpeg --duration 5
    python -m tools.bench --exposure-us 5000     # 5 ms ручная
    python -m tools.bench --auto-exposure        # форсить auto

Цель: получить честные числа ДО/ПОСЛЕ оптимизаций, чтобы видеть прогресс.
"""

from __future__ import annotations

import argparse
import os
import shutil
import statistics
import subprocess
import sys
import time
from pathlib import Path

# Allow `python tools/bench.py` from project root without -m.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cv2  # noqa: E402

from camera import _apply_v4l2_high_fps_controls  # noqa: E402
from config import ConfigStore  # noqa: E402
from detector import BallDetector  # noqa: E402
from platform_utils import (  # noqa: E402
    IS_LINUX,
    IS_RPI,
    apply_pi_tuning,
    get_camera_backend,
)


def _hw_summary() -> None:
    print("=" * 60)
    print("HARDWARE")
    print("=" * 60)

    model = "(unknown)"
    try:
        model = Path("/proc/device-tree/model").read_text(errors="ignore").strip("\x00 \n")
    except OSError:
        pass
    print(f"  Model        : {model}")

    try:
        with open("/proc/cpuinfo") as f:
            cpus = sum(1 for line in f if line.startswith("processor"))
    except OSError:
        cpus = os.cpu_count() or 0
    print(f"  CPUs         : {cpus}")

    if IS_LINUX:
        for i in range(cpus):
            try:
                cur = int(Path(f"/sys/devices/system/cpu/cpu{i}/cpufreq/scaling_cur_freq").read_text())
                gov = Path(f"/sys/devices/system/cpu/cpu{i}/cpufreq/scaling_governor").read_text().strip()
                print(f"  CPU{i}         : {cur/1000:.0f} MHz  governor={gov}")
            except (OSError, ValueError):
                break

        try:
            t = int(Path("/sys/class/thermal/thermal_zone0/temp").read_text()) / 1000.0
            print(f"  Temp         : {t:.1f} °C")
        except (OSError, ValueError):
            pass

    try:
        info = cv2.getBuildInformation()
        ocl = cv2.ocl.haveOpenCL()
        print(f"  OpenCV       : {cv2.__version__}  threads={cv2.getNumThreads()}  OpenCL_avail={ocl}")
        for kw in ("Parallel framework", "JPEG ", "FFMPEG", "GStreamer"):
            for line in info.split("\n"):
                if kw in line:
                    print(f"    {line.strip()}")
                    break
    except cv2.error:
        pass
    print()


def _v4l2_enum(camera_idx: int) -> None:
    """Show what the camera actually advertises via v4l2-ctl."""
    if not IS_LINUX:
        return
    print("=" * 60)
    print(f"STAGE 0: v4l2-ctl /dev/video{camera_idx}")
    print("=" * 60)
    if not shutil.which("v4l2-ctl"):
        print("  v4l2-ctl not installed (apt install v4l-utils). Skipping.")
        print()
        return
    try:
        out = subprocess.run(
            ["v4l2-ctl", "-d", f"/dev/video{camera_idx}", "--list-formats-ext"],
            capture_output=True, text=True, timeout=4,
        )
        if out.returncode != 0:
            print(f"  v4l2-ctl error: {out.stderr.strip() or out.stdout.strip()}")
        else:
            # Print only Size / Interval lines for brevity.
            for line in out.stdout.splitlines():
                s = line.strip()
                if s.startswith(("Size:", "Interval:", "[", "Pixel Format")):
                    print("  " + s)
    except (subprocess.TimeoutExpired, OSError) as e:
        print(f"  v4l2-ctl failed: {e}")

    # Also dump current exposure controls — handy to see what the driver
    # was last left at.
    for ctl in ("exposure_auto", "exposure_absolute", "exposure_dynamic_framerate"):
        try:
            out = subprocess.run(
                ["v4l2-ctl", "-d", f"/dev/video{camera_idx}", f"--get-ctrl={ctl}"],
                capture_output=True, text=True, timeout=2,
            )
            if out.returncode == 0:
                print(f"  current {out.stdout.strip()}")
        except OSError:
            pass
    print()


def _set_exposure_v4l2(cap: cv2.VideoCapture, exposure_us: int | None, auto: bool) -> None:
    """Apply V4L2 exposure controls.

    V4L2 conventions (different from DirectShow!):
      CAP_PROP_AUTO_EXPOSURE: 1 = Manual Mode, 3 = Aperture Priority (auto).
      CAP_PROP_EXPOSURE: V4L2_CID_EXPOSURE_ABSOLUTE — units of 100 µs.
                        e.g. 100 -> 10 ms shutter.

    OpenCV's cv2.cap.set returns True/False; we report it for diagnostics.
    """
    if not IS_LINUX:
        return
    if auto:
        ok = cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3.0)  # Aperture Priority = auto
        print(f"[exposure] AUTO (V4L2 mode 3)  set_returned={ok}")
        return
    if exposure_us is None:
        return
    ok_mode = cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0)  # Manual
    abs_units = max(1, exposure_us // 100)              # in 100-µs steps
    ok_val = cap.set(cv2.CAP_PROP_EXPOSURE, float(abs_units))
    actual = cap.get(cv2.CAP_PROP_EXPOSURE)
    print(
        f"[exposure] MANUAL {exposure_us} µs (={abs_units}*100µs)  "
        f"set_mode={ok_mode}  set_val={ok_val}  read_back={actual}"
    )


def _open_capture(args, *, manual_exposure_us: int | None, auto_exposure: bool, force_fps: bool = False) -> cv2.VideoCapture | None:
    cap = cv2.VideoCapture(args.camera, get_camera_backend())
    if not cap.isOpened():
        print(f"[bench] could not open camera {args.camera}")
        return None

    if IS_LINUX and not args.no_mjpeg:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FPS, float(args.fps))
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(args.width))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(args.height))

    if force_fps and IS_LINUX:
        # Force VIDIOC_S_PARM and turn off the auto-priority that lets the
        # driver slow the stream down to suit the current exposure.
        _apply_v4l2_high_fps_controls(f"/dev/video{args.camera}", args.fps)

    _set_exposure_v4l2(cap, manual_exposure_us, auto_exposure)

    aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    afps = cap.get(cv2.CAP_PROP_FPS)
    try:
        afcc = int(cap.get(cv2.CAP_PROP_FOURCC)).to_bytes(4, "little").decode(errors="ignore")
    except Exception:
        afcc = "?"
    print(f"[bench] negotiated {aw}x{ah} @ {afps:.1f} fps fourcc={afcc} (requested {args.width}x{args.height} @ {args.fps})")
    # Drop a few frames to let auto-anything settle. Some UVC drivers report
    # 1-second latency on the first 2-3 reads while internal buffers warm up.
    for _ in range(5):
        cap.read()
    return cap


def bench_capture(cap: cv2.VideoCapture, duration: float, label: str) -> dict:
    """Pure capture FPS — no processing."""
    n = 0
    times: list[float] = []
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < duration:
        ts = time.perf_counter()
        ok, _frame = cap.read()
        if ok:
            n += 1
            times.append(time.perf_counter() - ts)
    elapsed = time.perf_counter() - t0
    p50 = statistics.median(times) * 1000 if times else 0
    p95 = (sorted(times)[int(len(times) * 0.95)] * 1000) if len(times) > 20 else 0
    print(f"[{label}] FPS={n/elapsed:.1f}  ({n} frames in {elapsed:.2f}s)  read p50={p50:.2f} ms  p95={p95:.2f} ms")
    return {"fps": n / elapsed, "frames": n, "p50_ms": p50, "p95_ms": p95}


def bench_detector(cap: cv2.VideoCapture, duration: float) -> dict:
    """Capture + detector FPS (mirrors logic_thread_func work)."""
    store = ConfigStore()
    detector = BallDetector()
    n = 0
    proc_times: list[float] = []
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < duration:
        ok, frame = cap.read()
        if not ok:
            continue
        ts = time.perf_counter()
        detector.process(frame, store)
        proc_times.append(time.perf_counter() - ts)
        n += 1
    elapsed = time.perf_counter() - t0
    p50 = statistics.median(proc_times) * 1000 if proc_times else 0
    p95 = (sorted(proc_times)[int(len(proc_times) * 0.95)] * 1000) if len(proc_times) > 20 else 0
    print(f"[detector] FPS={n/elapsed:.1f}  ({n} frames in {elapsed:.2f}s)  blur={detector.BLUR_KSIZE} morph={detector.MORPH_KSIZE}  process p50={p50:.2f} ms  p95={p95:.2f} ms")
    return {"fps": n / elapsed, "frames": n, "p50_ms": p50, "p95_ms": p95}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=120)
    parser.add_argument("--duration", type=float, default=3.0, help="Per-stage duration, seconds")
    parser.add_argument("--no-mjpeg", action="store_true", help="Disable MJPEG fourcc on Linux")
    parser.add_argument("--low-res", action="store_true", help="Shortcut: 320x240")
    parser.add_argument("--no-pi-tune", action="store_true", help="Skip apply_pi_tuning()")
    parser.add_argument("--exposure-us", type=int, default=10000,
                        help="Manual exposure in microseconds for the manual stage (default 10000 = 10 ms)")
    parser.add_argument("--auto-exposure", action="store_true",
                        help="Skip manual stage, run only auto-exposure capture")
    args = parser.parse_args()

    if args.low_res:
        args.width, args.height = 320, 240

    if not args.no_pi_tune:
        apply_pi_tuning()

    _hw_summary()
    _v4l2_enum(args.camera)

    # ----------- Stage A: AUTO exposure (driver decides) -----------
    print("=" * 60)
    print(f"STAGE A: capture + AUTO exposure ({args.duration}s)")
    print("=" * 60)
    cap_a = _open_capture(args, manual_exposure_us=None, auto_exposure=True)
    if cap_a:
        bench_capture(cap_a, args.duration, "auto-exp")
        cap_a.release()
    print()

    if args.auto_exposure:
        return

    # ----------- Stage B: MANUAL exposure (fixed shutter) -----------
    print("=" * 60)
    print(f"STAGE B: capture + MANUAL exposure {args.exposure_us} µs ({args.duration}s)")
    print("=" * 60)
    cap_b = _open_capture(args, manual_exposure_us=args.exposure_us, auto_exposure=False)
    if cap_b:
        bench_capture(cap_b, args.duration, "manual-exp")
        cap_b.release()
    print()

    # ----------- Stage C: capture + detector at the manual settings -----------
    print("=" * 60)
    print(f"STAGE C: capture + detector + MANUAL exposure ({args.duration}s)")
    print("=" * 60)
    cap_c = _open_capture(args, manual_exposure_us=args.exposure_us, auto_exposure=False)
    if cap_c:
        bench_detector(cap_c, args.duration)
        cap_c.release()
    print()

    # ----------- Stage D: SHORT exposure + force-FPS via v4l2-ctl ------------
    # This is the combo that should unlock 120 FPS at 640x480 on UVC: short
    # enough exposure (< 1/fps) AND auto-priority disabled AND --set-parm
    # actually issued.
    short_exp_us = min(args.exposure_us, max(1000, int(900_000 / args.fps)))
    print("=" * 60)
    print(f"STAGE D: capture + SHORT exposure {short_exp_us} µs + force {args.fps} fps ({args.duration}s)")
    print("=" * 60)
    cap_d = _open_capture(args, manual_exposure_us=short_exp_us, auto_exposure=False, force_fps=True)
    if cap_d:
        bench_capture(cap_d, args.duration, "force-fps")
        cap_d.release()
    print()

    # ----------- Stage E: OpenCV WITHOUT MJPEG decoding ----------------------
    # If pure v4l2-ctl reports >>60 fps (as it does here, ~100 fps) but
    # cap.read() returns 60, the prime suspects are (a) MJPEG decoding cost
    # and (b) a frame-drop on the driver side because OpenCV doesn't dequeue
    # fast enough. CAP_PROP_CONVERT_RGB=0 disables the libjpeg-turbo decode
    # path inside cap.read() — it returns the raw MJPEG buffer as a 1D byte
    # array. If THIS stage matches pure-v4l2 FPS, decode is the bottleneck.
    print("=" * 60)
    print(f"STAGE E: OpenCV WITHOUT MJPEG decode ({args.duration}s)")
    print("=" * 60)
    cap_e = _open_capture(args, manual_exposure_us=short_exp_us, auto_exposure=False, force_fps=True)
    if cap_e:
        cap_e.set(cv2.CAP_PROP_CONVERT_RGB, 0.0)
        print("[mode] CAP_PROP_CONVERT_RGB=0 -> raw MJPEG out, no decode")
        bench_capture(cap_e, args.duration, "raw-mjpg")
        cap_e.release()
    print()

    # ----------- Stage F: bigger driver buffer + drain via grab() ------------
    # If the driver is dropping frames because OpenCV doesn't dequeue them
    # fast enough, a larger BUFFERSIZE plus calling cap.grab() in a tight
    # loop (no decode) can pull more frames per second through. The actual
    # FPS reported here counts only successful grab()s.
    print("=" * 60)
    print(f"STAGE F: BUFFERSIZE=4 + grab-only loop ({args.duration}s)")
    print("=" * 60)
    cap_f = _open_capture(args, manual_exposure_us=short_exp_us, auto_exposure=False, force_fps=True)
    if cap_f:
        cap_f.set(cv2.CAP_PROP_BUFFERSIZE, 4)
        n = 0
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < args.duration:
            if cap_f.grab():
                n += 1
        elapsed = time.perf_counter() - t0
        print(f"[grab-only] FPS={n/elapsed:.1f}  ({n} frames in {elapsed:.2f}s)")
        cap_f.release()
    print()

    # ----------- Stage G: BUFFERSIZE=4 + cap.read() (production path) --------
    # This is what VideoStream.update() actually does after the fix:
    # BUFFERSIZE=4 + tight cap.read() (full path: DQBUF + MJPEG decode + numpy
    # BGR ndarray). If FPS here matches Stage F (~100+), the production
    # pipeline gets the same uplift.
    print("=" * 60)
    print(f"STAGE G: BUFFERSIZE=4 + cap.read() decode (production path) ({args.duration}s)")
    print("=" * 60)
    cap_g = _open_capture(args, manual_exposure_us=short_exp_us, auto_exposure=False, force_fps=True)
    if cap_g:
        cap_g.set(cv2.CAP_PROP_BUFFERSIZE, 4)
        bench_capture(cap_g, args.duration, "prod-read")
        cap_g.release()
    print()

    print("Done.")
    print()
    print("Reading the numbers:")
    print("  - STAGE A FPS << target (e.g. 1-3) → camera is in auto-exposure low-light mode.")
    print("  - STAGE B FPS climbs to 60-120 → manual exposure works; basis for next steps.")
    print("  - STAGE C ~= STAGE B → detector is NOT the bottleneck (it isn't, on Pi 4).")
    print("  - STAGE D → if FPS lifts to 120, the dynamic-framerate flag was the culprit.")
    print("  - STAGE E ~ pure-v4l2 fps → MJPEG decode in cap.read() is the bottleneck.")
    print("  - STAGE F ~ pure-v4l2 fps → buffer-latency dropping was the bottleneck.")
    print("  - STAGE E ≈ STAGE F ≈ STAGE D (≈60) → camera/USB hardware-locked at this rate.")


if __name__ == "__main__":
    main()
