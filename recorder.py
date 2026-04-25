"""Trajectory recorder.

Writes the X/Y pixel deltas (the same `nx`/`ny` floats the firmware sees as
`normX`/`normY`) into a per-session CSV file. After the recording is stopped
two more artifacts are produced:

    1. A sibling `*.data.js` next to the CSV, containing only the data the
       viewer actually shows (`t` + `nx`) wrapped as
       `window.RECORDING_DATA = {...};`. We use a `.js` wrapper instead of a
       plain JSON file because browsers block `fetch()` to local files when
       a page is opened with the `file://` protocol — but `<script src=...>`
       tags work fine. The CSV stays the canonical source for analysis tools.

    2. An entry appended to the manifest in `viewer/`. Two files are kept in
       sync there:

           viewer/manifest.json   <- proper JSON, source of truth
           viewer/manifest.js     <- mirror, `window.MANIFEST = <json>;`

       Both are git-tracked so the viewer folder is reusable; the JSON is for
       any tool that wants to read it raw (jq, pandas, your own scripts), the
       JS mirror is what the browser actually loads.

Threading model
---------------
Three call sites, three different threads:

    start() / stop()  – main/render thread (DPG callback)
    add_sample()      – logic thread, ~capture FPS (60–120 Hz)
    status()          – main/render thread (UI refresh)

A single mutex guards file open/close transitions and the writer object so
add_sample() can safely race with stop(): worst case it observes the
"not recording" flag and returns early without touching anything.

File layout
-----------
    <recordings_dir>/trajectory_YYYY-MM-DD_HH-MM-SS.csv      <- raw data
    <recordings_dir>/trajectory_YYYY-MM-DD_HH-MM-SS.data.js  <- viewer payload
    <project_root>/viewer/manifest.json                      <- source of truth
    <project_root>/viewer/manifest.js                        <- browser-facing

CSV header lines start with `#` so spreadsheets ignore them when imported,
and `pandas.read_csv(path, comment="#")` parses them cleanly.
"""

from __future__ import annotations

import csv
import json
import os
import shutil
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

APP_NAME = "BallTrackerPro"


def _is_frozen() -> bool:
    return getattr(sys, "frozen", False)


def _project_root() -> Path:
    """Folder where the dev-mode `viewer/` sits.

    In dev this is just the parent of recorder.py. In a frozen build there is
    no source tree so we fall back to recordings_dir().parent — this still
    gives the user a usable layout next to where their recordings end up.
    """
    if not _is_frozen():
        return Path(__file__).resolve().parent
    return recordings_dir().parent


def viewer_dir() -> Path:
    """Where `index.html` + `manifest.{json,js}` live.

    Always under the project root in dev. In a frozen build the recorder
    creates a `viewer/` folder next to the recordings the first time the
    user records something so the manifest has a place to live.
    """
    return _project_root() / "viewer"


def _bundled_viewer_html() -> Optional[Path]:
    """Locate the viewer/index.html shipped with the build (or with src).

    PyInstaller's `--add-data` extracts bundled files into a temporary
    folder exposed as `sys._MEIPASS` at runtime; we look there first. We
    also fall back to a few platform-specific spots in case a future build
    config puts the resource elsewhere (Windows portable next to the exe,
    macOS .app/Contents/Resources/...).
    """
    if not _is_frozen():
        return Path(__file__).resolve().parent / "viewer" / "index.html"

    meipass = getattr(sys, "_MEIPASS", None)
    if meipass:
        candidate = Path(meipass) / "viewer" / "index.html"
        if candidate.is_file():
            return candidate

    exe_dir = Path(sys.executable).resolve().parent
    candidate = exe_dir / "viewer" / "index.html"
    if candidate.is_file():
        return candidate

    if sys.platform == "darwin":
        # exe sits at <App>.app/Contents/MacOS/<exe>
        contents = exe_dir.parent
        candidate = contents / "Resources" / "viewer" / "index.html"
        if candidate.is_file():
            return candidate

    return None


_viewer_staged_once = False


def open_viewer_in_browser() -> tuple[bool, str]:
    """Open the viewer's index.html in the default browser.

    Works the same in dev and frozen builds because both go through
    `viewer_dir()`. In a frozen build we first call
    `_ensure_viewer_html_present()` so the user can hit the button before
    they've ever recorded anything (otherwise the file wouldn't exist yet).

    Returns (success, message). On success `message` is the resolved
    file:// URL we opened; on failure it's a human-readable reason the UI
    can surface to the user / log.
    """
    _ensure_viewer_html_present()
    target = viewer_dir() / "index.html"
    if not target.is_file():
        return False, f"viewer/index.html not found at {target}"

    import webbrowser
    url = target.resolve().as_uri()
    try:
        # new=2 -> new tab in existing browser window when possible.
        opened = webbrowser.open(url, new=2)
    except Exception as e:
        return False, f"webbrowser raised: {e}"
    if not opened:
        return False, f"no browser registered to open {url}"
    return True, url


def _ensure_viewer_html_present() -> None:
    """Copy the bundled index.html into the user-data viewer dir if needed.

    No-op in dev (the source file IS the user-facing file). In a frozen
    build this is what makes `~/Documents/BallTrackerPro/viewer/index.html`
    appear next to the manifest the first time the user records something.

    Re-copies if the bundled template is newer than what's on disk so app
    updates ship viewer fixes too. User-edited copies will get overwritten
    in that case — accept that trade-off; deleting `index.html` from the
    user dir is enough to opt back in to the bundled one anyway.
    """
    global _viewer_staged_once
    if _viewer_staged_once:
        return

    src = _bundled_viewer_html()
    if src is None:
        _viewer_staged_once = True
        return

    target = viewer_dir() / "index.html"
    try:
        if src.resolve() == target.resolve():
            _viewer_staged_once = True
            return  # dev mode — same file
    except OSError:
        pass

    try:
        target.parent.mkdir(parents=True, exist_ok=True)
        if (
            not target.exists()
            or src.stat().st_mtime > target.stat().st_mtime
        ):
            shutil.copy2(src, target)
            print(f"[recorder] viewer template staged -> {target}")
    except OSError as e:
        print(f"[recorder] could not stage viewer template: {e}")
    finally:
        _viewer_staged_once = True


def recordings_dir() -> Path:
    """Where recordings live, per OS / launch mode.

    Recordings are USER DATA the user wants to find and analyse later, so
    they live in `~/Documents/<AppName>/recordings/` on macOS and Linux —
    not in Application Support / .config which are reserved for app
    settings. On Windows we keep the portable feel: `<exe-dir>/recordings/`.
    In dev mode they go straight into the project folder so they are easy
    to delete with `git clean`.
    """
    if not _is_frozen():
        return Path(__file__).resolve().parent / "recordings"

    if sys.platform == "darwin":
        return Path.home() / "Documents" / APP_NAME / "recordings"
    if sys.platform.startswith("win"):
        return Path(sys.executable).resolve().parent / "recordings"
    return Path.home() / "Documents" / APP_NAME / "recordings"


def _format_size(n: int) -> str:
    if n < 1024:
        return f"{n} B"
    if n < 1024 * 1024:
        return f"{n / 1024:.1f} KB"
    return f"{n / (1024 * 1024):.2f} MB"


def _relpath_for_viewer(target: Path) -> str:
    """Path of `target` expressed relative to viewer/ with forward slashes.

    The viewer loads sibling files via `<script src=...>` tags whose `src`
    is resolved against the document's URL, so paths stored in the manifest
    must be relative to `viewer/index.html`. We force forward slashes so
    the same manifest works on Windows too — browsers prefer them and so
    do file:// URLs.
    """
    try:
        rel = os.path.relpath(target, viewer_dir())
    except ValueError:
        # Different drive on Windows — fall back to absolute file:// URI.
        return target.resolve().as_uri()
    return rel.replace(os.sep, "/")


class Recorder:
    def __init__(self):
        self._lock = threading.Lock()
        self._is_recording = False
        self._file = None
        self._writer = None
        self._path: Optional[Path] = None
        self._t0: float = 0.0
        self._sample_count = 0
        self._last_flush = 0.0
        self._metadata: dict = {}
        # Keep an in-memory copy of the (t, nx) pairs while recording so the
        # `.data.js` payload can be emitted at stop() without re-reading and
        # re-parsing the CSV. ny is *not* buffered: the viewer doesn't show
        # it (only nx is currently meaningful for control), and we'd rather
        # spend the bytes on a slightly longer recording window. ny still
        # lands in the CSV for offline analysis.
        self._buf_t: list[float] = []
        self._buf_nx: list[float] = []

    @property
    def is_recording(self) -> bool:
        return self._is_recording

    @property
    def path(self) -> Optional[Path]:
        return self._path

    def start(self, metadata: dict) -> Optional[Path]:
        """Open a new CSV. Returns the path on success, None on failure."""
        with self._lock:
            if self._is_recording:
                return self._path

            try:
                base = recordings_dir()
                base.mkdir(parents=True, exist_ok=True)
                # Make sure the viewer template lives next to the manifest
                # we're about to write — otherwise the user would end up
                # with manifest.{json,js} but nothing to open them with.
                _ensure_viewer_html_present()
                stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                self._path = base / f"trajectory_{stamp}.csv"
                self._file = open(
                    self._path, "w", newline="", encoding="utf-8"
                )
                self._writer = csv.writer(self._file)

                self._metadata = dict(metadata)
                self._metadata["started_at"] = datetime.now().isoformat(
                    timespec="seconds"
                )
                # Comment-prefixed metadata: spreadsheets ignore '#' rows on
                # import, pandas reads them with comment='#'. Keep human-
                # friendly so a `head -n 6 file.csv` is enough to recover
                # context weeks later.
                for key in (
                    "started_at", "kp", "max_omega",
                    "resolution", "camera_fps", "source",
                ):
                    if key in self._metadata:
                        self._file.write(
                            f"# {key}: {self._metadata[key]}\n"
                        )
                self._writer.writerow(["t_sec", "nx_px", "ny_px"])
                self._file.flush()

                self._t0 = time.perf_counter()
                self._sample_count = 0
                self._last_flush = self._t0
                self._buf_t.clear()
                self._buf_nx.clear()
                self._is_recording = True
                print(f"[recorder] -> {self._path}")
                return self._path
            except OSError as e:
                print(f"[recorder] failed to start: {e}")
                self._cleanup()
                return None

    def add_sample(self, nx: float, ny: float):
        """Append one row. Hot path — must stay cheap.

        Boolean flag check is intentionally lock-free for the common
        "not recording" case so the logic thread doesn't pay any mutex
        cost when recording is off.
        """
        if not self._is_recording:
            return
        with self._lock:
            if not self._is_recording or self._writer is None:
                return
            try:
                t = time.perf_counter() - self._t0
                self._writer.writerow(
                    [f"{t:.4f}", f"{nx:.3f}", f"{ny:.3f}"]
                )
                self._buf_t.append(t)
                self._buf_nx.append(nx)
                self._sample_count += 1
                # Flush about once per second so a hard crash loses at most
                # ~1s of samples instead of the whole file. Disk hits are
                # rare enough not to matter at 120 Hz.
                now = time.perf_counter()
                if now - self._last_flush > 1.0:
                    self._file.flush()
                    self._last_flush = now
            except OSError:
                pass

    def stop(self) -> Optional[Path]:
        """Close the CSV, emit `.data.js`, append manifest entry."""
        with self._lock:
            if not self._is_recording:
                return None
            self._is_recording = False
            csv_path = self._path
            try:
                self._file.flush()
                self._file.close()
            except OSError as e:
                print(f"[recorder] error closing file: {e}")

            # Snapshot the buffers + metadata; release the lock for the
            # (potentially slow-ish) viewer write so a follow-up start()
            # isn't blocked.
            buf_t = list(self._buf_t)
            buf_nx = list(self._buf_nx)
            metadata = dict(self._metadata)
            sample_count = self._sample_count

            self._cleanup()

        if csv_path is None:
            return None

        try:
            data_js_path = csv_path.with_suffix(".data.js")
            _write_data_js(data_js_path, csv_path.stem, buf_t, buf_nx)
            print(f"[recorder] viewer payload -> {data_js_path}")

            duration = round(buf_t[-1], 3) if buf_t else 0.0
            # Actual sample rate the logic thread produced — derived, not
            # supplied by the caller. This is the truth: even if main.py
            # *thinks* it's running at 120 Hz, what landed on disk is the
            # only number that matters for any downstream analysis.
            sample_rate = (
                round(sample_count / duration, 1) if duration > 0 else 0.0
            )

            entry = {
                "id": csv_path.stem,
                "csv": _relpath_for_viewer(csv_path),
                "data_js": _relpath_for_viewer(data_js_path),
                "started_at": metadata.get("started_at"),
                "duration_sec": duration,
                "samples": sample_count,
                "sample_rate_hz": sample_rate,
                "camera_fps": metadata.get("camera_fps"),
                "kp": metadata.get("kp"),
                "max_omega": metadata.get("max_omega"),
                "resolution": metadata.get("resolution"),
                "source": metadata.get("source"),
            }
            _append_to_manifest(entry)
            print(f"[recorder] manifest updated -> {viewer_dir()}")
        except Exception as e:
            print(f"[recorder] failed to publish to viewer: {e}")

        return csv_path

    def status(self) -> dict:
        """Cheap snapshot for the UI — safe to call every render frame."""
        if not self._is_recording or self._path is None:
            return {
                "recording": False,
                "duration_sec": 0.0,
                "size_bytes": 0,
                "size_pretty": "—",
                "samples": 0,
                "path": None,
            }
        try:
            size = self._path.stat().st_size if self._path.exists() else 0
        except OSError:
            size = 0
        return {
            "recording": True,
            "duration_sec": time.perf_counter() - self._t0,
            "size_bytes": size,
            "size_pretty": _format_size(size),
            "samples": self._sample_count,
            "path": str(self._path),
        }

    def _cleanup(self):
        self._file = None
        self._writer = None


# --------------------------------------------------------------------------- #
#  Viewer publishing                                                          #
# --------------------------------------------------------------------------- #


def _write_data_js(
    path: Path, recording_id: str, ts: list[float], nxs: list[float]
) -> None:
    """Emit a `<id>.data.js` file the viewer can <script>-load.

    Format:

        window.RECORDING_DATA = {"id": "...", "t": [...], "nx": [...]};

    A single global is overwritten on each load; the viewer copies the
    payload into its own cache and clears the global so the next load
    doesn't see stale data.
    """
    payload = {
        "id": recording_id,
        "t":  [round(v, 4) for v in ts],
        "nx": [round(v, 3) for v in nxs],
    }
    body = json.dumps(payload, separators=(",", ":"))
    path.write_text(
        "// Auto-generated by recorder.py — DO NOT EDIT BY HAND.\n"
        f"window.RECORDING_DATA = {body};\n",
        encoding="utf-8",
    )


_MANIFEST_JS_HEADER = (
    "// Auto-generated by recorder.py — DO NOT EDIT BY HAND.\n"
    "// Mirror of manifest.json wrapped in a `window.MANIFEST = ...;` "
    "assignment so the viewer\n"
    "// can load it via a <script> tag (browsers block fetch() to local "
    "files when the page\n"
    "// is served from file://).\n"
)


def _read_manifest_json(path: Path) -> list[dict]:
    if not path.exists():
        return []
    try:
        raw = path.read_text(encoding="utf-8").strip()
    except OSError:
        return []
    if not raw:
        return []
    try:
        data = json.loads(raw)
    except json.JSONDecodeError:
        # Corrupted file — back it up and start fresh so we don't lose new
        # recordings just because someone hand-edited the manifest badly.
        backup = path.with_suffix(".json.bak")
        try:
            path.replace(backup)
            print(f"[recorder] manifest.json was invalid, backed up -> {backup}")
        except OSError:
            pass
        return []
    return data if isinstance(data, list) else []


def _write_manifest_pair(viewer: Path, entries: list[dict]) -> None:
    """Atomically write manifest.json + mirror it into manifest.js."""
    json_path = viewer / "manifest.json"
    js_path = viewer / "manifest.js"

    json_blob = json.dumps(entries, indent=2)
    # Write json first via temp file so a half-written manifest never gets
    # observed by a recorder running in parallel (which we don't expect, but
    # cheap insurance).
    tmp = json_path.with_suffix(".json.tmp")
    tmp.write_text(json_blob + "\n", encoding="utf-8")
    os.replace(tmp, json_path)

    js_path.write_text(
        _MANIFEST_JS_HEADER + f"window.MANIFEST = {json_blob};\n",
        encoding="utf-8",
    )


def _append_to_manifest(entry: dict) -> None:
    viewer = viewer_dir()
    viewer.mkdir(parents=True, exist_ok=True)
    entries = _read_manifest_json(viewer / "manifest.json")
    # If a recording with the same id already exists (e.g. the user reran
    # within the same second), replace it — the file on disk is the truth.
    entries = [e for e in entries if e.get("id") != entry["id"]]
    entries.append(entry)
    _write_manifest_pair(viewer, entries)
