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

## 🧪 Troubleshooting

* **macOS: `cv2.VideoCapture` returns black frames** — the terminal/IDE has no camera permission. Open `System Settings → Privacy & Security → Camera`.
* **macOS: Exposure slider doesn't change anything** — you forgot `task install_uvc`. The Dashboard slider only takes effect on USB-UVC cameras after the helper is built. See "macOS exposure setup" above.
* **Arduino not detected** — verify the device shows up:
  * Windows: Device Manager → Ports (COM & LPT)
  * macOS: `ls /dev/cu.*`
  * Linux: `ls /dev/ttyACM* /dev/ttyUSB*`
  Then check that `platform_utils.SERIAL_MATCH_KEYWORDS` matches your adapter's description / device path.
* **High FPS (>60) on macOS** — generally not achievable through AVFoundation for UVC cameras; cap at 60 FPS or use Windows.
