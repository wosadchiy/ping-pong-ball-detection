# 🏓 Ping-Pong Ball Detection & Tracking System

A professional-grade computer vision system designed for real-time table tennis ball tracking (up to 120 FPS on Windows) and hardware-in-the-loop (HiL) control using an ELP Global Shutter camera and Arduino-based actuators.

The codebase is **cross-platform** (Windows / macOS / Linux). The active OpenCV backend and Arduino USB-serial discovery are selected automatically by `platform_utils.py`, so the same `python main.py` entry point works on every host.

---

## 🛠 Hardware Specifications

* **Camera:** ELP Global Shutter USB Camera (Model: USBGS1200P02-LC1100).
* **Controller:** Arduino (UNO / Nano / CH340) with a stepper motor driver.
* **Optics:** High-speed UVC-compliant lens (68° FOV recommended).
* **Connectivity:** High-speed USB 2.0/3.0 port (direct connection recommended for stable 120 FPS on Windows).

---

## 💻 Environment Setup

### 1. Prerequisites
* **Python** 3.10 or higher (recommended: 3.11 / 3.12).
* **Git** latest version.

### 2. Platform-Specific Installation

#### 🪟 Windows
1. **Python:** download from [python.org](https://www.python.org/) and tick **"Add Python to PATH"**.
2. **Git:** install [Git for Windows](https://git-scm.com/).
3. **Drivers:** install the [CH340 driver](http://www.wch-ic.com/downloads/CH341SER_EXE.html) if you use a generic Arduino clone.

#### 🍎 macOS (Intel / Apple Silicon)
1. **Homebrew:**
   ```bash
   /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
   ```
2. **Core tools:**
   ```bash
   brew install python@3.12 git
   ```
3. **CH340 / CH341 driver** is **built into macOS 11+**, no manual install required. Only on older macOS (≤ Catalina) install the [WCH driver](https://www.wch-ic.com/downloads/CH34XSER_MAC_ZIP.html).
4. **Camera permission:** the first launch triggers a Privacy & Security prompt. If you deny it, re-enable manually in
   `System Settings → Privacy & Security → Camera → <your terminal / IDE>`.
5. **USB / serial permissions:** none required — Arduino appears as `/dev/cu.usbmodem*` (native USB) or `/dev/cu.wchusbserial*` (CH340).

#### 🐧 Linux
1. Install Python and pip via your package manager.
2. Add your user to the `dialout` group so you can talk to `/dev/ttyACM*` / `/dev/ttyUSB*` without sudo:
   ```bash
   sudo usermod -aG dialout $USER && newgrp dialout
   ```

---

## 🚀 Project Installation

### TL;DR copy-paste

**Windows (PowerShell or cmd):**
```powershell
git clone https://github.com/wosadchiy/ping-pong-ball-detection.git
cd ping-pong-ball-detection
python -m venv venv
.\venv\Scripts\activate
pip install taskipy
task install
task dev
```

**macOS / Linux:**
```bash
git clone https://github.com/wosadchiy/ping-pong-ball-detection.git
cd ping-pong-ball-detection
python3 -m venv venv
source venv/bin/activate
pip install taskipy
task install
task dev
```

### Step-by-step

1. **Clone the repository**
   ```bash
   git clone https://github.com/wosadchiy/ping-pong-ball-detection.git
   cd ping-pong-ball-detection
   ```

2. **Initialise a virtual environment**
   ```bash
   # Windows
   python -m venv venv
   .\venv\Scripts\activate

   # macOS / Linux
   python3 -m venv venv
   source venv/bin/activate
   ```

   ⚠️ A `venv/` created on Windows **cannot** be reused on macOS/Linux (and vice versa). Always recreate it on the target OS.

3. **Install dependencies**

   The fast way (after the venv is activated) — installs everything in `requirements.txt` and, on macOS, also builds the `uvc-util` helper:
   ```bash
   pip install taskipy
   task install
   ```

   Or the manual equivalent:
   ```bash
   pip install -r requirements.txt
   # macOS only:
   task install_uvc
   ```

   > `task install` is roughly the Python equivalent of `npm install` / `pnpm install` — it reads `requirements.txt` (our `package.json`) and installs everything into the active virtual env. Re-run it any time `requirements.txt` changes.

---

## 🌅 macOS exposure setup

macOS AVFoundation **does not expose UVC controls** (exposure, gain, brightness, ...) for USB-UVC cameras like the ELP Global Shutter. Without a workaround the **Exposure** slider in the Dashboard is a no-op on macOS.

The fix is the [`uvc-util`](https://github.com/jtfrey/uvc-util) helper by jtfrey — a tiny Objective-C utility that talks to the camera through the USB Video Class API directly. The project includes a one-shot installer that clones it, compiles a universal arm64+x86_64 binary, and stores it under `vendor/uvc-util/`:

```bash
task install_uvc
```

Requirements: Xcode Command Line Tools (`xcode-select --install`).

After the build, `python main.py` will detect the binary automatically. On startup you should see `[uvc] no warning` (silence == OK). The Exposure slider then physically changes shutter time on the camera and the UI label below it shows the value in milliseconds.

If you have **multiple UVC cameras**, pin one in `settings.json` (otherwise the first device on the bus is used):

```json
{
  "uvc_device_name": "Global Shutter",
  "uvc_vendor_id":  13028,
  "uvc_product_id": 8756
}
```

(VID/PID can be read from `vendor/uvc-util/src/uvc-util -d`.)

Manual control directly from the shell, in case you need to tweak something the UI doesn't expose (gain, brightness, white balance, ...):

```bash
./vendor/uvc-util/src/uvc-util -d                                   # list devices
./vendor/uvc-util/src/uvc-util -I 0 -c                              # list controls of device 0
./vendor/uvc-util/src/uvc-util -I 0 -S exposure-time-abs            # show current value + range
./vendor/uvc-util/src/uvc-util -I 0 -s auto-exposure-mode=1         # 1 = manual
./vendor/uvc-util/src/uvc-util -I 0 -s exposure-time-abs=300        # 300 * 100µs = 30 ms
./vendor/uvc-util/src/uvc-util -I 0 -s gain=200                     # sensor gain (0..1023 on ELP)
```

Note: the **built-in FaceTime HD** camera is *not* a UVC device and will not appear in `uvc-util -d`. macOS auto-controls it; the exposure slider has no effect on it (which is fine — for ball tracking you'll be using the ELP).

---

## 🛠 Development Workflow (taskipy)

The project uses [`taskipy`](https://github.com/taskipy/taskipy) for routine commands. The default tasks are **cross-platform** — they delegate to `tasks.py` which uses Python's stdlib (`shutil`, `subprocess`) instead of shell-specific syntax.

| Command              | Description                                                          |
|----------------------|----------------------------------------------------------------------|
| `task install`       | Install Python deps from `requirements.txt` (+ uvc-util on macOS).   |
| `task install_uvc`   | macOS only: clone & compile `uvc-util` into `vendor/`.               |
| `task dev`           | Start the application.                                               |
| `task clean`         | Remove `dist/`, `build/` and `*.spec` artefacts.                     |
| `task buildDev`      | Clean and build a debug bundle (with console window) via PyInstaller.|
| `task buildProd`     | Clean and build a release bundle (no console / `.app` on macOS).     |

### Platform-specific variants

If you prefer raw shell invocations, the `pyproject.toml` also exposes explicit per-OS tasks:

| Task               | Shell    | Notes                                            |
|--------------------|----------|--------------------------------------------------|
| `task clean_win`   | cmd.exe  | `if exist … rd /s /q …`                          |
| `task buildDev_win`/`buildProd_win` | cmd.exe | Uses `copy`, `\\` paths.    |
| `task clean_mac`   | bash/zsh | `rm -rf dist build *.spec`                       |
| `task buildDev_mac`/`buildProd_mac` | bash/zsh | Builds a proper `.app` on macOS Prod. |

**Usage example:**
```bash
task dev          # run the app
task buildProd    # create a release bundle for the current OS
```

> **Cross-compiling is NOT possible.** PyInstaller (and every other Python freezer)
> bundles host system libraries — it cannot produce a Windows `.exe` from a Mac
> or vice versa. Build the Windows version on Windows, the macOS version on macOS.
> For automation, set up GitHub Actions with a `matrix: [windows-latest, macos-latest]`.

---

## 📦 Distribution / Packaged builds

`task buildProd` produces:

* **Windows:** `dist/BallTrackerPro/BallTrackerPro.exe` + side files. Drop the
  whole folder anywhere; settings.json sits next to the exe and travels with it.
* **macOS:** `dist/BallTrackerPro.app` (proper signed Cocoa bundle) + the raw
  onedir variant under `dist/BallTrackerPro/`.

### Where settings.json lives at run-time

| Mode                           | Path                                                          |
|--------------------------------|---------------------------------------------------------------|
| Dev (`python main.py`)         | `./settings.json` (project root)                              |
| Built Windows app              | next to `BallTrackerPro.exe` (portable)                       |
| Built macOS app                | `~/Library/Application Support/BallTrackerPro/settings.json`  |
| Built Linux app                | `$XDG_CONFIG_HOME/BallTrackerPro/settings.json` (defaults to `~/.config/...`) |

On macOS / Linux the app cannot write inside its own bundle (read-only DMG mount,
Gatekeeper restrictions), so we keep settings in the standard per-user location.
The bundle still ships a seed `settings.json` that gets copied on first launch
if no user copy exists yet.

### Where trajectory recordings + viewer live at run-time

The **RECORD** checkbox writes per-session `.csv` files plus a small `.data.js`
sibling, then appends an entry to a manifest the viewer reads. Both folders are
created on the **first** record:

| Mode                | recordings/                                          | viewer/ (open `index.html`)                          |
|---------------------|------------------------------------------------------|------------------------------------------------------|
| Dev                 | `./recordings/`                                      | `./viewer/`                                          |
| Built Windows app   | `<exe-dir>/recordings/`                              | `<exe-dir>/viewer/`                                  |
| Built macOS app     | `~/Documents/BallTrackerPro/recordings/`             | `~/Documents/BallTrackerPro/viewer/`                 |
| Built Linux app     | `~/Documents/BallTrackerPro/recordings/`             | `~/Documents/BallTrackerPro/viewer/`                 |

> **You don't open the viewer from `dist/`.** The build embeds
> `viewer/index.html` inside the bundle (via PyInstaller `--add-data`); on the
> first record the recorder copies it out to the user-data `viewer/` folder
> next to the freshly created `manifest.{json,js}`. That's the path you want
> to open in your browser. Subsequent app updates re-stage a newer
> `index.html` automatically (the user copy is replaced when the bundled one
> has a newer mtime).

Reset to a clean slate at any time with:

```bash
task clean_recordings
```

This wipes `recordings/` and resets `manifest.{json,js}` to empty in dev mode.
For the built app you can clear the same paths by hand (drag the
`~/Documents/BallTrackerPro/recordings/` folder to the trash and the manifest
will be regenerated on next record).

### macOS-specific build details

The `task buildProd` pipeline does three macOS-only steps after PyInstaller
finishes:

1. **Patch `Contents/Info.plist`** — adds `NSCameraUsageDescription` (without it
   macOS kills the process the moment OpenCV touches the camera with a
   `Termination Reason: Namespace TCC` SIGABRT), sets a proper reverse-DNS
   `CFBundleIdentifier` (`com.partyplay.balltrackerpro`), bumps the version to
   `1.0.0` and `LSMinimumSystemVersion` to 11.0.
2. **Bundle `vendor/uvc-util/src/uvc-util`** into the `.app` so the exposure
   controls work on machines that never ran `task install_uvc`. If you forgot to
   build the helper before packaging, the build still succeeds but you'll see
   a warning and the slider will be a no-op in the resulting bundle.
3. **Ad-hoc re-sign** the bundle (`codesign --force --deep --sign - …`) — every
   modification under `.app/` invalidates PyInstaller's signature, so we sign
   again. For Mac App Store / outside-distribution you'd swap the `-` for a
   real Developer ID certificate.

### Distributing to other Macs (Gatekeeper notes)

The ad-hoc signature is enough for **your own Mac** (and typically for CI
runners), but Gatekeeper will block strangers downloading the `.app` from the
internet unless one of these is true:

* The user opens it via right-click → **Open** the first time and confirms the
  dialog.
* You sign with a paid Apple Developer ID and notarise via `notarytool`.
* You ship the source and let users build it themselves.

---

## 📂 Project Layout

```
.
├── main.py             # entry point (UI loop + worker thread)
├── camera.py           # VideoStream + camera enumeration (cross-platform backend)
├── hardware.py         # ArduinoHandler (cross-platform port discovery)
├── detector.py         # HSV-based ball detection
├── ui.py               # Dear PyGui dashboard
├── config.py           # ConfigStore (settings.json persistence)
├── utils.py            # small helpers (EMA, …)
├── platform_utils.py   # OS detection + serial-port keywords
├── tasks.py            # cross-platform clean/build orchestrator (taskipy)
├── camera_arduino.py   # legacy single-file prototype (kept for reference)
├── pyproject.toml
├── requirements.txt
└── settings.json
```

---

## 🔧 Drive tuning mode (camera disconnected)

The Dashboard's **DRIVE TUNING** section (collapsible, off by default) lets
you characterise the stepper motor by itself — useful when the camera is
physically detached from the shaft and you want to find the limits of
acceleration / max speed without the closed-loop noise from the vision
stack.

Three controls live there:

| Control | What it sends to Arduino | Effect |
|---|---|---|
| `Acceleration (units/sec²)` | `A<value>\n` once per change | Caps the rate at which `omega` is allowed to change. The firmware ramps via a 1 kHz Timer2 ISR — see comments in `cameraControl.ino`. |
| `MANUAL OMEGA OVERRIDE` | `M0\n` / `M1\n` once per change | When ON, the firmware ignores camera P-control input and drives the motor straight from `Manual omega`. Physical jog buttons still take priority. |
| `Manual omega (units, signed)` | `O<value>\n` once per change | Direct omega target in user units. Sign = direction. Clamped on the firmware side to `[-Max Speed, +Max Speed]`. |

These messages are only emitted when the relevant value changes, so the
serial line stays quiet between slider movements. The legacy 7-field CSV
packet (`ax,ay,nx,ny,kp,tracking,max_omega`) is unchanged — old firmware
remains compatible, it just won't react to the new commands.

### Effective acceleration ceiling

Inside the firmware the velocity is quantised into `V_TABLE_N = 200`
levels in `[0, max_omega]`. The 1 kHz ISR can advance one level every N
ticks (where `N = accel_skip ∈ [1, 250]`), so:

```
α_eff = (max_omega / 200) * 1000 / accel_skip   user-units / sec²
```

The lowest `accel_skip` is 1, giving the maximum α the current
`max_omega` allows:

| Max Speed | α_max possible |
|---:|---:|
| 40  | ≈ 200 |
| 60  | ≈ 300 |
| 80  | ≈ 400 |
| 100 | ≈ 500 |

If you ask for more in the UI than `α_max` (e.g. α=500 with Max Speed 40),
the firmware silently clamps to `accel_skip=1`. To unlock faster ramps,
raise Max Speed first.

### Suggested tuning workflow

1. Detach the camera from the motor shaft (no mechanical load is added,
   but if your specific rig has friction quirks you'll want them in the
   loop while characterising).
2. Run the app, open **DRIVE TUNING**, tick `MANUAL OMEGA OVERRIDE`.
3. Sweep `Manual omega`: 0 → +Max Speed → 0 → −Max Speed → 0. Watch /
   listen for stalls.
4. If stalls happen during the ramp, lower `Acceleration` until clean.
5. If the motor hits speed without stalls, raise `Max Speed` and repeat.
6. Untick `MANUAL OMEGA OVERRIDE` when done — camera P-control resumes
   immediately.

Acceleration is shared with camera mode: whatever you settle on here
also caps the ramp rate when the camera drives the motor, so it's a
one-shot tuning step.

### 🧮 Browser-side simulator

Want to feel the parameters before flashing? Open
[`docs/drive-simulator.html`](docs/drive-simulator.html) in any browser —
pure-JS replica of the firmware ramp algorithm with three live plots:

- **ω(t)** — the stepwise advance of `cur_v_idx` toward `target_v_idx`
- **Step pulses** — vertical tick per rising edge on `stepPin`, with
  zoom buttons (20 ms / 100 ms / full / last 50 ms) and `T₁ T₂ T₃` annotations
- **T(n)** — period of the n-th pulse in µs (the curve from the original
  napkin sketch)

Inputs: `Max Speed`, `Acceleration`, `Initial omega`, `Manual omega
(target)`, `Duration`. Plus quick presets for "rest → max", "reversal",
"α clamped", "slow ramp" and "small step". Everything is computed with
the same `vel_table[]`, `accel_skip` formula and Timer2 walk as
`cameraControl.ino` — so what you see is what the MCU would actually do.
No build step, no server: just open the file.

### 📐 Structural block diagram (control-theory view)

For MATLAB/Simulink modelling there's a separate
[`docs/structural-diagram.html`](docs/structural-diagram.html) — a
GOST-style block diagram that splits the loop into Python host blocks
(optics+detector+EMA+TX), Arduino blocks (parser+P-regulator+ramp+pulse
generator) and the physical kinematics, with **explicit transfer functions
(s- and z-domain) for every stage** ready to paste into a Simulink model.

The diagram makes it visually obvious that the inner motor loop is
currently **open** (no encoder, no tachometer, no current loop) — only the
outer visual loop is closed via the camera. A side-by-side comparison
table shows what would need to be added to reach a classical
three-regulator servo-drive structure (current → speed → position).

---

## 🧪 Troubleshooting

* **macOS: `cv2.VideoCapture` returns black frames** — the terminal/IDE has no camera permission. Open `System Settings → Privacy & Security → Camera`.
* **macOS: built `.app` crashes immediately with `Termination Reason: Namespace TCC`** — your `Info.plist` is missing `NSCameraUsageDescription`. This happens if you build with raw `pyinstaller` instead of `task buildProd` (the latter patches the plist + re-signs automatically). Re-build via the task or apply the same patch manually.
* **macOS: Exposure slider doesn't change anything** — you forgot `task install_uvc`. The Dashboard slider only takes effect on USB-UVC cameras after the helper is built. See "macOS exposure setup" above.
* **macOS: settings.json doesn't persist between launches of the built app** — confirm it actually lives at `~/Library/Application Support/BallTrackerPro/settings.json` (not next to the `.app`, that path is read-only inside a signed bundle).
* **Arduino not detected** — verify the device shows up:
  * Windows: Device Manager → Ports (COM & LPT)
  * macOS: `ls /dev/cu.*`
  * Linux: `ls /dev/ttyACM* /dev/ttyUSB*`
  Then check that `platform_utils.SERIAL_MATCH_KEYWORDS` matches your adapter's description / device path.
* **High FPS (>60) on macOS** — generally not achievable through AVFoundation for UVC cameras; cap at 60 FPS or use Windows.
