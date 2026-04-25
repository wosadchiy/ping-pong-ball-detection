"""macOS-only UVC control bridge built on top of `uvc-util`.

Why this module exists:
    macOS AVFoundation does not expose UVC controls (exposure, gain, ...) for
    USB cameras. The only reliable workaround is to talk to the device through
    the UVC class API directly. We do this by shelling out to the
    `uvc-util` binary (https://github.com/jtfrey/uvc-util), which is a small
    Objective-C utility that ships pre-built in `vendor/uvc-util/` of this
    repo (see README "macOS exposure setup" section).

Public API:
    is_available()                           -> bool
    list_devices()                           -> list[UvcDevice]
    find_index(name=..., vendor=..., ...)    -> int | None
    set_manual_exposure(index, exposure_units, gain=None) -> bool
    set_auto_exposure(index)                 -> bool
    dshow_to_uvc_units(dshow_value)          -> int

The binary is searched in this order:
    1. `<bundle>/Contents/Resources/uvc-util` (frozen .app, copied at build time)
    2. `sys._MEIPASS/uvc-util`                (frozen onedir/onefile, --add-binary)
    3. `vendor/uvc-util/src/uvc-util`         (project-local build, dev mode)
    4. `vendor/uvc-util`                      (project-local copy)
    5. `uvc-util` on PATH                     (system install / brew)
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parent


def _bundled_candidates() -> list[Path]:
    """Locations the uvc-util binary may live in when running from a build."""
    paths: list[Path] = []
    meipass = getattr(sys, "_MEIPASS", None)
    if meipass:
        paths.append(Path(meipass) / "uvc-util")
    if getattr(sys, "frozen", False) and sys.platform == "darwin":
        # Inside a .app bundle: Contents/MacOS/<exe> -> Contents/Resources/uvc-util
        exe_parent = Path(sys.executable).resolve().parent
        paths.append(exe_parent.parent / "Resources" / "uvc-util")
    return paths


_LOCAL_CANDIDATES = (
    _PROJECT_ROOT / "vendor" / "uvc-util" / "src" / "uvc-util",
    _PROJECT_ROOT / "vendor" / "uvc-util",
)

# UVC `auto-exposure-mode` is a bitmap field, see USB Video Class spec §A.9.7.
AE_MODE_MANUAL = 1
AE_MODE_AUTO = 2
AE_MODE_SHUTTER_PRIORITY = 4
AE_MODE_APERTURE_PRIORITY = 8


@dataclass(frozen=True)
class UvcDevice:
    index: int
    vendor_id: int
    product_id: int
    location_id: str
    name: str


def _binary_path() -> str | None:
    for candidate in (*_bundled_candidates(), *_LOCAL_CANDIDATES):
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return str(candidate)
    return shutil.which("uvc-util")


def is_available() -> bool:
    """True if the uvc-util binary can be found AND we are on macOS."""
    return sys.platform == "darwin" and _binary_path() is not None


def _run(args: list[str], timeout: float = 3.0) -> tuple[int, str, str]:
    """Run uvc-util with given arguments. Returns (returncode, stdout, stderr)."""
    binary = _binary_path()
    if binary is None:
        return 127, "", "uvc-util binary not found"
    try:
        proc = subprocess.run(
            [binary, *args],
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
        return proc.returncode, proc.stdout, proc.stderr
    except subprocess.TimeoutExpired:
        return 124, "", "uvc-util timed out"


_LIST_RE = re.compile(
    r"^\s*(\d+)\s+0x([0-9a-fA-F]+):0x([0-9a-fA-F]+)\s+(\S+)\s+\S+\s+(.+?)\s*$"
)


def list_devices() -> list[UvcDevice]:
    """Parse `uvc-util -d` output into structured records."""
    rc, out, _ = _run(["-d"])
    if rc != 0:
        return []
    devices: list[UvcDevice] = []
    for line in out.splitlines():
        # Skip table header / separator rows.
        if not line.strip() or line.lstrip().startswith(("-", "Index")):
            continue
        m = _LIST_RE.match(line)
        if not m:
            continue
        idx, vid, pid, loc, name = m.groups()
        devices.append(
            UvcDevice(
                index=int(idx),
                vendor_id=int(vid, 16),
                product_id=int(pid, 16),
                location_id=loc,
                name=name.strip(),
            )
        )
    return devices


def find_index(
    name: str | None = None,
    vendor_id: int | None = None,
    product_id: int | None = None,
) -> int | None:
    """Find a uvc-util device index by partial name and/or VID/PID match.

    All provided filters must match. Returns the first matching device's
    `index` (which is what -I expects), or None if nothing matched.
    """
    needle = name.lower() if name else None
    for d in list_devices():
        if needle is not None and needle not in d.name.lower():
            continue
        if vendor_id is not None and d.vendor_id != vendor_id:
            continue
        if product_id is not None and d.product_id != product_id:
            continue
        return d.index
    return None


def _set(index: int, control: str, value: int | str) -> bool:
    rc, _, err = _run(["-I", str(index), "-s", f"{control}={value}"])
    if rc != 0 and err.strip():
        print(f"[uvc-util] failed to set {control}={value}: {err.strip()}")
    return rc == 0


def set_manual_exposure(
    index: int,
    exposure_units: int,
    gain: int | None = None,
) -> bool:
    """Switch the camera into manual-exposure mode and apply a value.

    `exposure_units` is in UVC native units of 100 microseconds
    (i.e. value 312 == 31.2 ms == 1/32 s).
    """
    ok = _set(index, "auto-exposure-mode", AE_MODE_MANUAL)
    ok &= _set(index, "exposure-time-abs", int(exposure_units))
    if gain is not None:
        ok &= _set(index, "gain", int(gain))
    return ok


def set_auto_exposure(index: int) -> bool:
    """Hand exposure back to the camera (aperture-priority is the UVC default)."""
    return _set(index, "auto-exposure-mode", AE_MODE_APERTURE_PRIORITY)


def dshow_to_uvc_units(dshow_value: int) -> int:
    """Convert DirectShow log2-seconds exposure (e.g. -5 == 1/32 s) to UVC units.

    DirectShow:  exposure_seconds = 2 ** dshow_value
    UVC:         1 unit = 100 microseconds
    -> units = (2 ** dshow_value) * 1_000_000 / 100 = (2 ** dshow_value) * 10_000

    Clamped to the typical UVC range [1, 10000] (== 100μs .. 1s).
    """
    units = (2.0 ** dshow_value) * 10_000.0
    return max(1, min(10_000, int(round(units))))
