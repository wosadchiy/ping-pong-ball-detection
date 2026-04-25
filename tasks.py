"""Cross-platform build / clean orchestrator invoked by taskipy.

Why a Python launcher instead of inline shell commands?
    Windows uses cmd.exe (`del`, `rd /s /q`, `copy`, `&`) and macOS/Linux use
    bash (`rm -rf`, `cp`, `&&`). Hard-coding either dialect into pyproject.toml
    breaks the other host. This script uses the stdlib (`shutil`, `subprocess`)
    so the same task name works everywhere.

Usage (called via `task <name>`, see [tool.taskipy.tasks] in pyproject.toml):
    python tasks.py clean
    python tasks.py build [--debug]
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent

IS_WINDOWS = sys.platform.startswith("win")
IS_MACOS = sys.platform == "darwin"


def _info(msg: str) -> None:
    print(f"[TASK] {msg}", flush=True)


def cmd_clean() -> int:
    """Remove PyInstaller artefacts (`build/`, `dist/`, `*.spec`)."""
    _info("Cleaning build artefacts...")
    for d in ("build", "dist"):
        target = ROOT / d
        if target.exists():
            shutil.rmtree(target, ignore_errors=True)
            _info(f"  removed {d}/")
    for spec in ROOT.glob("*.spec"):
        spec.unlink(missing_ok=True)
        _info(f"  removed {spec.name}")
    _info("Done.")
    return 0


def cmd_build(debug: bool) -> int:
    """Build a stand-alone bundle via PyInstaller (debug => with console)."""
    cmd_clean()

    name = "BallTracker_Debug" if debug else "BallTrackerPro"
    mode_flag = "--console" if debug else "--windowed"

    args = [
        sys.executable, "-m", "PyInstaller",
        "--noconfirm",
        "--onedir",
        mode_flag,
        "--name", name,
        "--clean",
        "main.py",
    ]
    _info(f"Running: {' '.join(args)}")
    result = subprocess.run(args, cwd=ROOT)
    if result.returncode != 0:
        _info(f"PyInstaller failed with code {result.returncode}")
        return result.returncode

    # PyInstaller layout differs by platform:
    #   Windows / Linux  -> dist/<name>/<name>(.exe)        + side files
    #   macOS --windowed -> dist/<name>.app/Contents/MacOS  (a real .app bundle)
    #                       AND dist/<name>/                (raw onedir copy)
    settings_src = ROOT / "settings.json"
    if not settings_src.exists():
        _info("settings.json not found in project root, skipping copy.")
        return 0

    targets: list[Path] = []
    onedir = ROOT / "dist" / name
    if onedir.exists():
        targets.append(onedir / "settings.json")
    if IS_MACOS and not debug:
        app_resources = ROOT / "dist" / f"{name}.app" / "Contents" / "Resources"
        app_resources.mkdir(parents=True, exist_ok=True)
        targets.append(app_resources / "settings.json")

    for dst in targets:
        shutil.copy2(settings_src, dst)
        _info(f"  copied settings.json -> {dst.relative_to(ROOT)}")

    _info(f"Build OK: dist/{name}{'.app' if IS_MACOS and not debug else ''}")
    return 0


def cmd_install() -> int:
    """Install Python dependencies from requirements.txt into the active env.

    Roughly the equivalent of `npm install` / `pnpm install`. Run this AFTER
    activating the virtual env. On macOS it also kicks off `install_uvc` so
    the camera exposure controls work out of the box.
    """
    req = ROOT / "requirements.txt"
    if not req.exists():
        _info("requirements.txt not found")
        return 1

    _info("Installing Python deps from requirements.txt ...")
    rc = subprocess.run(
        [sys.executable, "-m", "pip", "install", "-r", str(req)],
        cwd=ROOT,
    ).returncode
    if rc != 0:
        return rc

    if IS_MACOS:
        _info("macOS detected -> ensuring uvc-util helper is built ...")
        rc = cmd_install_uvc()
        if rc != 0:
            _info("uvc-util build failed; you can retry later with: task install_uvc")
            # Don't fail the whole install — Python deps are usable even
            # without UVC controls.

    _info("All set. Run `task dev` to start the app.")
    return 0


def cmd_install_uvc() -> int:
    """macOS-only: clone & compile the `uvc-util` helper into vendor/.

    On Windows / Linux this is a no-op (UVC controls there work via
    OpenCV / V4L2 directly, no helper needed).
    """
    if not IS_MACOS:
        _info("install_uvc is only needed on macOS, skipping.")
        return 0

    vendor = ROOT / "vendor"
    repo = vendor / "uvc-util"
    binary = repo / "src" / "uvc-util"

    if binary.exists():
        _info(f"uvc-util already built at {binary.relative_to(ROOT)}")
        return 0

    vendor.mkdir(exist_ok=True)
    if not repo.exists():
        _info("cloning jtfrey/uvc-util...")
        rc = subprocess.run(
            ["git", "clone", "--depth", "1",
             "https://github.com/jtfrey/uvc-util.git", str(repo)],
        ).returncode
        if rc != 0:
            _info("git clone failed.")
            return rc

    src_dir = repo / "src"
    sources = sorted(str(p) for p in src_dir.glob("*.m"))
    if not sources:
        _info(f"no .m sources found in {src_dir}")
        return 1

    _info("compiling universal binary (arm64 + x86_64)...")
    rc = subprocess.run(
        [
            "clang",
            "-arch", "arm64", "-arch", "x86_64",
            "-O2", "-fno-objc-arc",
            "-framework", "IOKit",
            "-framework", "Foundation",
            "-framework", "CoreMedia",
            "-framework", "AVFoundation",
            "-o", str(binary),
            *sources,
        ],
        cwd=src_dir,
    ).returncode
    if rc != 0:
        _info("clang failed.")
        return rc

    _info(f"OK: {binary.relative_to(ROOT)}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(prog="tasks", description="Build helper")
    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("clean", help="remove dist/, build/ and *.spec")
    sub.add_parser("install", help="pip install -r requirements.txt (+uvc on macOS)")
    sub.add_parser("install_uvc", help="build vendor/uvc-util (macOS only)")

    p_build = sub.add_parser("build", help="bundle the app via PyInstaller")
    p_build.add_argument(
        "--debug",
        action="store_true",
        help="build the debug variant (with console window)",
    )

    args = parser.parse_args()
    if args.cmd == "clean":
        return cmd_clean()
    if args.cmd == "build":
        return cmd_build(debug=args.debug)
    if args.cmd == "install":
        return cmd_install()
    if args.cmd == "install_uvc":
        return cmd_install_uvc()
    parser.error(f"unknown command: {args.cmd}")
    return 2


if __name__ == "__main__":
    sys.exit(main())
