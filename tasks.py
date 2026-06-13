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


def cmd_clean_recordings() -> int:
    """Wipe `recordings/` and reset `viewer/manifest.{json,js}` to empty.

    Use this after experimenting to get back to a pristine state without
    having to remember to also reset the manifest by hand (otherwise the
    viewer would still try to load deleted .data.js files and show errors).
    """
    _info("Cleaning trajectory recordings + viewer manifest...")

    rec_dir = ROOT / "recordings"
    removed_files = 0
    if rec_dir.exists():
        for p in rec_dir.iterdir():
            if p.is_file():
                try:
                    p.unlink()
                    removed_files += 1
                except OSError as e:
                    _info(f"  WARN could not delete {p.name}: {e}")
    _info(f"  removed {removed_files} file(s) from recordings/")

    viewer = ROOT / "viewer"
    if viewer.exists():
        # Reuse recorder's writer so the wrapper format stays in sync with
        # whatever the running app produces. Otherwise it's too easy for
        # this script and the recorder to drift apart.
        sys.path.insert(0, str(ROOT))
        try:
            from recorder import _write_manifest_pair
            _write_manifest_pair(viewer, [])
            _info("  reset viewer/manifest.json + viewer/manifest.js")
        except Exception as e:
            _info(f"  WARN could not reset manifest via recorder: {e}")

        for p in list(viewer.glob("*.bak")) + list(viewer.glob("*.tmp")):
            try:
                p.unlink()
                _info(f"  removed {p.relative_to(ROOT)}")
            except OSError:
                pass

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

    # Ship the viewer template inside the bundle. At runtime recorder.py
    # extracts it to ~/Documents/BallTrackerPro/viewer/ (macOS) or
    # <exe-dir>/viewer/ (Windows) so the user has a UI for the manifest
    # the recorder is about to start producing. Without this the built app
    # would create `viewer/manifest.{json,js}` next to the recordings but
    # leave the user with no `index.html` to open.
    viewer_html = ROOT / "viewer" / "index.html"
    if viewer_html.exists():
        sep = ";" if IS_WINDOWS else ":"
        # "src{sep}dest_inside_bundle" — dest is relative to _MEIPASS.
        args += ["--add-data", f"{viewer_html}{sep}viewer"]
        _info(f"  bundling viewer template from {viewer_html.relative_to(ROOT)}")
    else:
        _info(
            "  WARN viewer/index.html not found — built app will record "
            "data but the user will have no UI to view it."
        )

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


def _find_pio_openocd() -> tuple[Path, Path] | None:
    """Locate the openocd binary + its scripts dir that PlatformIO downloaded.

    PlatformIO drops the toolchain in `~/.platformio/packages/tool-openocd/`
    on Linux/macOS and `%USERPROFILE%\\.platformio\\packages\\tool-openocd`
    on Windows. We don't shell out to `pio` because it adds ~3 s of startup
    just to print a path.
    """
    pio_home = Path.home() / ".platformio" / "packages" / "tool-openocd"
    if not pio_home.is_dir():
        return None
    bin_name = "openocd.exe" if IS_WINDOWS else "openocd"
    candidates = [
        pio_home / "bin" / bin_name,
        pio_home / bin_name,
    ]
    binary = next((c for c in candidates if c.is_file()), None)
    if binary is None:
        return None
    scripts = pio_home / "openocd" / "scripts"
    if not scripts.is_dir():
        scripts = pio_home / "scripts"
    if not scripts.is_dir():
        return None
    return binary, scripts


def _openocd_cmds(extra_cmds: list[str]) -> list[str] | None:
    """Build an openocd argv targeting the on-board ST-Link/V2-1.

    Mirrors the upload settings in `firmware/stm32_nucleo_f103rb/platformio.ini`
    (hla_swd transport, srst-only reset under assert) so we never confuse the
    MCU about how the probe drives reset. Returns None if openocd isn't on
    disk yet — caller should print a hint to run `task fw_build` first.
    """
    found = _find_pio_openocd()
    if found is None:
        return None
    binary, scripts = found
    argv: list[str] = [
        str(binary),
        "-s", str(scripts),
        "-f", "interface/stlink.cfg",
        "-c", "transport select hla_swd",
        "-c", "reset_config srst_only srst_nogate connect_assert_srst",
        "-f", "target/stm32f1x.cfg",
    ]
    for c in extra_cmds:
        argv += ["-c", c]
    return argv


def cmd_fw_diag() -> int:
    """Probe the MCU via SWD: dump PC/SP, resume, and check if PC moved.

    Nothing about this writes to flash. Useful when the LED is stuck
    on/off after a flash — tells you whether the core is actually running
    the freshly written firmware or stuck in halt / HardFault.
    """
    argv = _openocd_cmds([
        "init",
        "halt",
        "echo {== first halt ==}",
        # General-purpose registers + the relevant clock-control + reset-cause
        # registers. RCC_CSR (0x40021024) low-byte tells us *why* the MCU
        # last reset (POR / NRST / WDG / SOFT). Bit 25 (LPWRRSTF), bit 24
        # (WWDGRSTF), bit 23 (IWDGRSTF), bit 22 (SFTRSTF), bit 21 (PORRSTF),
        # bit 20 (PINRSTF). Bit 25:24 set => crashed via watchdog.
        "reg pc",
        "reg sp",
        "reg lr",
        "echo {-- RCC_CSR (0x40021024), bit26=LPWRRSTF .. bit20=PINRSTF --}",
        "mdw 0x40021024 1",
        "echo {-- RCC_CR    (0x40021000), bit25=PLLRDY bit17=HSERDY bit1=HSIRDY --}",
        "mdw 0x40021000 1",
        "echo {-- RCC_CFGR  (0x40021004), bit3:2=SWS (00=HSI,01=HSE,10=PLL) --}",
        "mdw 0x40021004 1",
        "resume",
        "sleep 500",
        "halt",
        "echo {== second halt (500 ms after resume) ==}",
        "echo {== if PC differs => firmware is running. If identical => stuck. ==}",
        "reg pc",
        "reg sp",
        "reg lr",
        "exit",
    ])
    if argv is None:
        _info("openocd not found. Run `task fw_build` first to fetch the toolchain.")
        return 1
    _info("Running openocd diagnostic ...")
    return subprocess.run(argv, cwd=ROOT).returncode


def cmd_fw_reset() -> int:
    """Issue a clean external NRST + run via openocd, no flashing.

    Use when `task fw_flash` printed `Error 1` after `** Verified OK **`
    and left the MCU halted, or when you want to restart the existing
    firmware without unplugging the USB cable.
    """
    argv = _openocd_cmds([
        "init",
        "reset run",
        "exit",
    ])
    if argv is None:
        _info("openocd not found. Run `task fw_build` first to fetch the toolchain.")
        return 1
    _info("Issuing reset run via openocd ...")
    return subprocess.run(argv, cwd=ROOT).returncode


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
    sub.add_parser(
        "clean_recordings",
        help="wipe recordings/ and reset viewer/manifest.{json,js}",
    )
    sub.add_parser("install", help="pip install -r requirements.txt (+uvc on macOS)")
    sub.add_parser("install_uvc", help="build vendor/uvc-util (macOS only)")
    sub.add_parser("fw_diag", help="probe target STM32 via SWD (read-only)")
    sub.add_parser("fw_reset", help="reset target STM32 via SWD (no flashing)")

    p_build = sub.add_parser("build", help="bundle the app via PyInstaller")
    p_build.add_argument(
        "--debug",
        action="store_true",
        help="build the debug variant (with console window)",
    )

    args = parser.parse_args()
    if args.cmd == "clean":
        return cmd_clean()
    if args.cmd == "clean_recordings":
        return cmd_clean_recordings()
    if args.cmd == "build":
        return cmd_build(debug=args.debug)
    if args.cmd == "install":
        return cmd_install()
    if args.cmd == "install_uvc":
        return cmd_install_uvc()
    if args.cmd == "fw_diag":
        return cmd_fw_diag()
    if args.cmd == "fw_reset":
        return cmd_fw_reset()
    parser.error(f"unknown command: {args.cmd}")
    return 2


if __name__ == "__main__":
    sys.exit(main())
