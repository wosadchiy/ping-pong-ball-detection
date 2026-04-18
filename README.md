# 🏓 Ping-Pong Ball Detection & Tracking System

A professional-grade computer vision system designed for real-time table tennis ball tracking (120 FPS) and hardware-in-the-loop (HiL) control using an ELP Global Shutter camera and Arduino-based actuators.

---

## 🛠 Hardware Specifications

* **Camera:** ELP Global Shutter USB Camera (Model: USBGS1200P02-LC1100).
* **Controller:** Arduino (UNO/Nano/CH340) with a Stepper Motor driver.
* **Optics:** High-speed UVC-compliant lens (68° FOV recommended).
* **Connectivity:** High-speed USB 2.0/3.0 port (Direct connection recommended for stable 120 FPS).

---

## 💻 Environment Setup

### 1. Prerequisites
* **Python:** 3.10 or higher (Recommended: 3.11/3.12).
* **Git:** Latest version for version control.

### 2. Platform-Specific Installation

#### 🪟 Windows
1.  **Python:** Download from [python.org](https://www.python.org/). Ensure **"Add Python to PATH"** is checked during installation.
2.  **Git:** Download and install [Git for Windows](https://git-scm.com/).
3.  **Drivers:** Install the [CH340 Driver](http://www.wch-ic.com/downloads/CH341SER_EXE.html) if using a generic Arduino clone.

#### 🍎 macOS
1.  **Homebrew:** Install the package manager via terminal:
    ```bash
    /bin/bash -c "$(curl -fsSL [https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh](https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh))"
    ```
2.  **Core Tools:**
    ```bash
    brew install python git
    ```

---

## 🚀 Project Installation

1.  **Clone the Repository:**
    ```bash
    git clone [https://github.com/wosadchiy/ping-pong-ball-detection.git](https://github.com/wosadchiy/ping-pong-ball-detection.git)
    cd ping-pong-ball-detection
    ```

2.  **Initialize Virtual Environment:**
    ```bash
    # Windows
    python -m venv venv
    .\venv\Scripts\activate

    # macOS/Linux
    python3 -m venv venv
    source venv/bin/activate
    ```

3.  **Install Dependencies:**
    ```bash
    pip install -r requirements.txt
    ```
    *If `requirements.txt` is missing, run:*
    ```bash
    pip install opencv-python numpy pyserial taskipy pyinstaller
    ```

---

## 🛠 Development Workflow (Taskipy)

The project uses `taskipy` to automate routine tasks, similar to `npm scripts` in web development.

| Command | Description |
| :--- | :--- |
| `task dev` | Starts the application in development mode. |
| `task clean` | Performs a deep clean: removes `dist`, `build`, and `.spec` files with console feedback. |
| `task build` | Executes `clean`, compiles the standalone `.exe` (onedir), and copies `settings.json` to the output folder. |

**Usage Example:**
```bash
task build

