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
import plistlib
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent

IS_WINDOWS = sys.platform.startswith("win")
IS_MACOS = sys.platform == "darwin"

# Reverse-DNS bundle ID for the macOS .app. Used as CFBundleIdentifier and
# also fed to `--osx-bundle-identifier` so PyInstaller stamps it consistently.
MACOS_BUNDLE_ID = "com.partyplay.balltrackerpro"

# Strings shown by macOS in the system permission dialogs ("BallTrackerPro
# would like to access the camera." + this sentence). Required by Apple — if
# absent the OS terminates the process the moment it touches the camera.
MACOS_USAGE_DESCRIPTIONS = {
    "NSCameraUsageDescription": (
        "BallTrackerPro uses the camera to detect ping-pong balls in real time."
    ),
    # Some OpenCV builds initialise AVCaptureSession in a way that probes the
    # mic too. Adding this avoids a second TCC crash if that ever happens.
    "NSMicrophoneUsageDescription": (
        "BallTrackerPro does not record audio; this entry is only here to "
        "satisfy the AVFoundation pipeline initialised by OpenCV."
    ),
}


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


def _uvc_binary_for_bundle() -> Path | None:
    """Return the path of the locally-built uvc-util binary, if any.

    We bundle this helper into the .app so end-users don't have to clone /
    compile anything just to control camera exposure on macOS.
    """
    candidate = ROOT / "vendor" / "uvc-util" / "src" / "uvc-util"
    return candidate if candidate.is_file() else None


def _patch_macos_info_plist(app_path: Path) -> None:
    """Write the privacy keys macOS demands; without them the app SIGABRTs.

    macOS aborts any process that reaches a TCC-protected API (camera, mic,
    location...) without a matching `NS*UsageDescription` in its Info.plist.
    PyInstaller doesn't add these by default, so we inject them here.

    We also normalise CFBundleIdentifier (PyInstaller defaults to just the
    app name, which macOS treats as a non-namespaced ID and refuses to
    persist TCC grants for) and bump LSMinimumSystemVersion.
    """
    plist_path = app_path / "Contents" / "Info.plist"
    if not plist_path.is_file():
        _info(f"WARN: {plist_path} missing — cannot patch privacy keys")
        return

    with plist_path.open("rb") as f:
        info = plistlib.load(f)

    info.update(MACOS_USAGE_DESCRIPTIONS)
    info["CFBundleIdentifier"] = MACOS_BUNDLE_ID
    info["LSMinimumSystemVersion"] = "11.0"
    info["NSHighResolutionCapable"] = True
    # PyInstaller defaults these to "0.0.0" / empty which looks broken in the
    # Dock and the crash reporter. Override unconditionally.
    info["CFBundleShortVersionString"] = "1.0.0"
    info["CFBundleVersion"] = "1"

    with plist_path.open("wb") as f:
        plistlib.dump(info, f)
    _info(f"  patched {plist_path.relative_to(ROOT)}")


def _adhoc_resign(app_path: Path) -> None:
    """Re-sign the bundle with an ad-hoc identity.

    Touching Info.plist after PyInstaller's signing step invalidates the
    embedded signature; macOS will refuse to launch (or, worse, kill the
    process mid-run) until we sign again. Ad-hoc (`-`) is enough for local
    use and CI; for App Store / notarisation you'd swap in a real Developer
    ID here.
    """
    rc = subprocess.run(
        ["codesign", "--force", "--deep", "--sign", "-", str(app_path)],
        cwd=ROOT,
    ).returncode
    if rc == 0:
        _info(f"  ad-hoc re-signed {app_path.name}")
    else:
        _info(f"WARN: codesign returned {rc}; the app may refuse to launch")


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

    # macOS: ship the uvc-util helper inside the bundle so exposure control
    # works on user machines that never ran `task install_uvc`.
    if IS_MACOS:
        args += ["--osx-bundle-identifier", MACOS_BUNDLE_ID]
        uvc = _uvc_binary_for_bundle()
        if uvc is not None:
            # Format: "src{os.pathsep}dest_inside_bundle". "." == _MEIPASS root.
            args += ["--add-binary", f"{uvc}{':.' if not IS_WINDOWS else ';.'}"]
            _info(f"  including uvc-util helper from {uvc.relative_to(ROOT)}")
        else:
            _info(
                "  NOTE: vendor/uvc-util/src/uvc-util not found — exposure "
                "controls will be inactive in the built app. Run "
                "`task install_uvc` then rebuild to fix."
            )

    _info(f"Running: {' '.join(args)}")
    result = subprocess.run(args, cwd=ROOT)
    if result.returncode != 0:
        _info(f"PyInstaller failed with code {result.returncode}")
        return result.returncode

    # PyInstaller layout differs by platform:
    #   Windows / Linux  -> dist/<name>/<name>(.exe)        + side files
    #   macOS --windowed -> dist/<name>.app/Contents/MacOS  (a real .app bundle)
    #                       AND dist/<name>/                (raw onedir copy)
    app_path: Path | None = None
    if IS_MACOS and not debug:
        app_path = ROOT / "dist" / f"{name}.app"
        if app_path.exists():
            _patch_macos_info_plist(app_path)

    settings_src = ROOT / "settings.json"
    if settings_src.exists():
        targets: list[Path] = []
        onedir = ROOT / "dist" / name
        if onedir.exists():
            targets.append(onedir / "settings.json")
        if app_path is not None and app_path.exists():
            res = app_path / "Contents" / "Resources"
            res.mkdir(parents=True, exist_ok=True)
            targets.append(res / "settings.json")

        for dst in targets:
            shutil.copy2(settings_src, dst)
            _info(f"  seeded settings.json -> {dst.relative_to(ROOT)}")
    else:
        _info("settings.json not found in project root, skipping seed copy.")

    # Re-sign LAST: any modification under .app (plist edit, file copy)
    # invalidates the previous signature.
    if app_path is not None and app_path.exists():
        _adhoc_resign(app_path)

    suffix = ".app" if (IS_MACOS and not debug) else ""
    _info(f"Build OK: dist/{name}{suffix}")
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
