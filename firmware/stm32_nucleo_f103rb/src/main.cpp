/**
 * STM32 Nucleo-F103RB — SPI drive controller (Step 3.1 of Arduino migration).
 *
 * What this sketch does
 * ---------------------
 *  • Acts as an SPI **slave** on SPI2 (Pi 4 is the master via /dev/spidev0.0).
 *  • Receives a fixed 16-byte framed packet carrying everything the motor
 *    control loop needs (the same inputs the old Arduino sketch consumed
 *    over UART, now binary over SPI):
 *        [0]  0xAA      preamble
 *        [1]  0x55      preamble
 *        [2]  flags     bit0 = manual_active, bit1 = tracking
 *        [3..4]   manual_omega  int16 LE  (user units, signed)
 *        [5..6]   err_n         int16 LE  (normalised error  ×10000, ±1.0)
 *        [7..8]   derr_n        int16 LE  (normalised d/dt    ×1000, per sec)
 *        [9..10]  kp_x100       int16 LE  (Kp × 100)
 *        [11..12] max_omega     int16 LE  (speed cap, user units)
 *        [13..14] td_x1000      int16 LE  (derivative time Td × 1000, sec)
 *        [15] xor   XOR of bytes [0..14]
 *  • Runs the control law ON THE MCU (like the Arduino did) — Python only
 *    supplies normalised camera inputs + tuning knobs:
 *        manual_active → omega = manual_omega
 *        tracking      → drive = clamp((err_n + Td·derr_n)·Kp, ±1)
 *                        omega = drive · max_omega        (PD on camera error)
 *        otherwise     → omega = 0 (coils released)
 *  • Prints the decoded command + packet stats over the USART2 VCP — see
 *    `task fw_serial`.
 *  • Drives the stepper: spin direction = sign(omega), step rate ∝ |omega|.
 *
 * Why SPI2 (not SPI1)
 * -------------------
 *  SPI1's SCK is PA5 — the same pin as the on-board green LED (LD2), and its
 *  MISO is PA6 which we used for a button. SPI2 lives entirely on free pins
 *  of the ST-morpho header CN10, so we keep the LED and all motor pins.
 *
 *  SPI2 slave wiring (Pi 4 SPI0 master ──► Nucleo CN10):
 *      signal     Pi 4 (BCM / hdr-pin)        Nucleo STM32 pin   CN10 pin
 *      ---------  --------------------------  -----------------  --------
 *      MOSI       GPIO10 / pin 19    ───────► PB15 (SPI2_MOSI)   CN10-26
 *      MISO       GPIO9  / pin 21    ◄─────── PB14 (SPI2_MISO)   CN10-28
 *      SCLK       GPIO11 / pin 23    ───────► PB13 (SPI2_SCK)    CN10-30
 *      CE0/NSS    GPIO8  / pin 24    ───────► PB12 (SPI2_NSS)    CN10-16
 *      GND        any GND (pin 20/25)──────── GND                CN10-20
 *  Both sides are 3.3 V — wire like-named signals directly, no level shift.
 *  A COMMON GROUND between Pi and Nucleo is mandatory.
 *
 * Motor pins (unchanged from the jog test — see table below)
 * ----------------------------------------------------------
 *      Arduino name | Nucleo conn | STM32 pin
 *      D5  (dir)    | CN9-6       | PB4
 *      D9  (step)   | CN5-2       | PC7
 *      D10 (en)     | CN5-3       | PB6   (active-low: LOW = enabled)
 *      LED_BUILTIN  | onboard LD2 | PA5
 *  NB: the two jog buttons (PA6/PA8) are no longer read — manual control
 *  now comes from the Pi over SPI. The pins are left untouched.
 *
 * SPI implementation note
 * -----------------------
 *  stm32duino's Arduino `SPI` object is master-only, so we drop to the
 *  ST HAL/LL (available inside the Arduino framework) for slave mode: a
 *  raw RXNE interrupt feeds a tiny sync-word state machine. The state
 *  machine re-aligns on the 0xAA 0x55 preamble after any glitch, and the
 *  XOR checksum rejects corrupt frames — so we never permanently desync
 *  even if a byte is dropped. NSS is a hardware input (CE0 from the Pi):
 *  the peripheral only clocks while the Pi has the slave selected.
 */

#include <Arduino.h>

// ──────────────────────────────────────────────────────────────────────────
// Motor pin configuration (Arduino-header numbering — see file header table).
// ──────────────────────────────────────────────────────────────────────────
constexpr int stepPin = 9;     // PC7
constexpr int dirPin  = 5;     // PB4
constexpr int enPin   = 10;    // PB6 — active-low: LOW = driver enabled

// Map |omega| (user units) → step frequency. omega=40 → 1000 Hz reproduces
// the old MANUAL_STEP_HZ; the legacy firmware clamped omega to ±200, so the
// top end here is 200×25 = 5000 Hz. Clamp keeps us inside what a bit-banged
// loop can emit reliably.
constexpr uint32_t STEP_HZ_PER_UNIT = 25;
constexpr uint32_t STEP_HZ_MIN      = 50;
constexpr uint32_t STEP_HZ_MAX      = 6000;

// How often to echo the latest decoded command + packet stats to Serial.
constexpr uint32_t HEARTBEAT_MS = 250;

// ──────────────────────────────────────────────────────────────────────────
// SPI framing protocol (must match firmware/.../tools/spi_test.py and the
// Stm32SpiHandler in hardware.py). 16-byte fixed frame, MSB-first, SPI mode 0.
// ──────────────────────────────────────────────────────────────────────────
constexpr uint8_t SPI_SYNC0   = 0xAA;
constexpr uint8_t SPI_SYNC1   = 0x55;
constexpr uint8_t SPI_PKT_LEN = 16;

// Fixed-point scales agreed with the Pi side. Keep in sync with hardware.py.
constexpr float ERR_N_SCALE  = 10000.0f;   // err_n   int16  → [-1.0, +1.0]
constexpr float DERR_N_SCALE = 1000.0f;    // derr_n  int16  → per-second
constexpr float KP_SCALE     = 100.0f;     // kp_x100 int16  → Kp
constexpr float TD_SCALE     = 1000.0f;    // td_x1000 int16 → Td seconds

// Decoded command — written by the SPI ISR, read by loop(). `volatile` + a
// short critical section (noInterrupts/interrupts) in loop() is enough: the
// fields are tiny and the ISR is the only writer.
static volatile bool     g_manualActive = false;
static volatile bool     g_tracking     = false;
static volatile int16_t  g_manualOmega  = 0;
static volatile int16_t  g_errN         = 0;
static volatile int16_t  g_derrN        = 0;
static volatile int16_t  g_kpX100       = 0;
static volatile int16_t  g_maxOmega     = 0;
static volatile int16_t  g_tdX1000      = 0;
static volatile uint32_t g_goodPackets  = 0;
static volatile uint32_t g_badPackets   = 0;

// HAL handle for SPI2. Initialised in spiSlaveBegin().
static SPI_HandleTypeDef hspi2;

// ──────────────────────────────────────────────────────────────────────────
// SPI receive state machine. Called once per received byte from the ISR.
// Hunts for the 0xAA 0x55 preamble, collects the body, verifies the XOR
// checksum, and on success publishes {manualActive, manualOmega}.
// ──────────────────────────────────────────────────────────────────────────
static inline int16_t le16(const uint8_t* p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline void spiFeedByte(uint8_t b) {
    static uint8_t state = 0;
    static uint8_t buf[SPI_PKT_LEN];

    switch (state) {
        case 0:                                  // expect sync0
            if (b == SPI_SYNC0) { buf[0] = b; state = 1; }
            break;
        case 1:                                  // expect sync1
            if (b == SPI_SYNC1)      { buf[1] = b; state = 2; }
            else if (b == SPI_SYNC0) { state = 1; }   // 0xAA 0xAA … keep hunting
            else                     { state = 0; }
            break;
        default:                                 // collecting body bytes 2..15
            buf[state] = b;
            if (++state >= SPI_PKT_LEN) {
                state = 0;
                uint8_t cs = 0;
                for (uint8_t i = 0; i < SPI_PKT_LEN - 1; ++i) cs ^= buf[i];
                if (cs == buf[SPI_PKT_LEN - 1]) {
                    g_manualActive = (buf[2] & 0x01) != 0;
                    g_tracking     = (buf[2] & 0x02) != 0;
                    g_manualOmega  = le16(&buf[3]);
                    g_errN         = le16(&buf[5]);
                    g_derrN        = le16(&buf[7]);
                    g_kpX100       = le16(&buf[9]);
                    g_maxOmega     = le16(&buf[11]);
                    g_tdX1000      = le16(&buf[13]);
                    g_goodPackets++;
                } else {
                    g_badPackets++;
                }
            }
            break;
    }
}

// Raw SPI2 interrupt. We don't use HAL_SPI_Receive_IT (it's length-counted
// and would desync on a dropped byte); instead we service RXNE ourselves and
// let spiFeedByte() handle framing. Reading DR clears the RXNE flag.
extern "C" void SPI2_IRQHandler(void) {
    if (__HAL_SPI_GET_FLAG(&hspi2, SPI_FLAG_RXNE)) {
        const uint8_t b = (uint8_t)(hspi2.Instance->DR);
        spiFeedByte(b);
        // Keep a fresh status byte queued on MISO so the Pi gets live
        // feedback (low byte of the good-packet count). Harmless if TXE
        // isn't ready yet — we just skip and refresh next time.
        if (__HAL_SPI_GET_FLAG(&hspi2, SPI_FLAG_TXE)) {
            hspi2.Instance->DR = (uint8_t)(g_goodPackets & 0xFF);
        }
    }
    // Clear a possible overrun (reading DR then SR). Without this an OVR
    // latches RXNE off and the link silently dies after one hiccup.
    if (__HAL_SPI_GET_FLAG(&hspi2, SPI_FLAG_OVR)) {
        volatile uint32_t tmp = hspi2.Instance->DR;
        tmp = hspi2.Instance->SR;
        (void)tmp;
    }
}

// Configure SPI2 as an 8-bit, mode-0, MSB-first slave with hardware NSS,
// then enable the RXNE interrupt. GPIO: PB13/PB15 inputs (SCK/MOSI driven by
// master), PB14 AF push-pull (MISO), PB12 NSS input with pull-up.
static void spiSlaveBegin(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_SPI2_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};

    // SCK (PB13) + MOSI (PB15): inputs, no pull — actively driven by the Pi.
    gpio.Pin  = GPIO_PIN_13 | GPIO_PIN_15;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &gpio);

    // MISO (PB14): alternate-function push-pull output.
    gpio.Pin   = GPIO_PIN_14;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    // NSS (PB12): hardware input, pull-up so the slave reads "deselected"
    // when the Pi isn't actively driving CE0 low.
    gpio.Pin   = GPIO_PIN_12;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    hspi2.Instance               = SPI2;
    hspi2.Init.Mode              = SPI_MODE_SLAVE;
    hspi2.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity       = SPI_POLARITY_LOW;     // SPI mode 0 (CPOL=0)
    hspi2.Init.CLKPhase          = SPI_PHASE_1EDGE;      // SPI mode 0 (CPHA=0)
    hspi2.Init.NSS               = SPI_NSS_HARD_INPUT;   // CE0 from the Pi
    hspi2.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;  // ignored in slave
    HAL_SPI_Init(&hspi2);

    // Preload MISO with a recognisable idle byte before any transfer.
    hspi2.Instance->DR = 0x42;

    __HAL_SPI_ENABLE_IT(&hspi2, SPI_IT_RXNE);
    __HAL_SPI_ENABLE(&hspi2);

    HAL_NVIC_SetPriority(SPI2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(SPI2_IRQn);
}

void setup() {
    // /EN HIGH (driver disabled) before becoming an output — no power-up glitch.
    digitalWrite(enPin, HIGH);
    pinMode(enPin, OUTPUT);
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
    Serial.println();
    Serial.println(F("[nucleo-f103rb] SPI drive controller (Step 3.1)"));
    Serial.println(F("  SPI2 slave: MOSI=PB15 MISO=PB14 SCK=PB13 NSS=PB12 (CN10)"));
    Serial.println(F("  waiting for 16-byte packets from the Pi (spidev0.0) ..."));

    spiSlaveBegin();
}

// Non-blocking step pulse generator: toggles stepPin at the requested half
// period using micros(). Returns immediately so the SPI ISR is never starved.
static void runStepper(bool active, int16_t omega) {
    static bool     stepLevel   = false;
    static uint32_t lastEdgeUs  = 0;

    if (!active || omega == 0) {
        digitalWrite(enPin, HIGH);          // release coils
        digitalWrite(stepPin, LOW);
        digitalWrite(LED_BUILTIN, LOW);
        stepLevel = false;
        return;
    }

    // Direction from sign; flip this ternary if the motor jogs the wrong way.
    digitalWrite(dirPin, omega > 0 ? LOW : HIGH);
    digitalWrite(enPin, LOW);               // engage coils
    digitalWrite(LED_BUILTIN, HIGH);        // "driving" indicator

    uint32_t mag = (omega > 0) ? (uint32_t)omega : (uint32_t)(-omega);
    uint32_t hz  = mag * STEP_HZ_PER_UNIT;
    if (hz < STEP_HZ_MIN) hz = STEP_HZ_MIN;
    if (hz > STEP_HZ_MAX) hz = STEP_HZ_MAX;
    const uint32_t halfPeriodUs = 500000UL / hz;

    const uint32_t now = micros();
    if ((uint32_t)(now - lastEdgeUs) >= halfPeriodUs) {
        lastEdgeUs = now;
        stepLevel = !stepLevel;
        digitalWrite(stepPin, stepLevel ? HIGH : LOW);
    }
}

// Compute the effective omega target from the latest decoded packet, mirroring
// the legacy Arduino priority: manual override first, then camera tracking,
// otherwise rest. The PD law matches camera_control_v2.ino's intent:
//     drive = clamp((err + Td·derr) · Kp, ±1);  omega = drive · max_omega
// `err`/`derr` arrive already normalised to ±1 (per second), so the law is
// resolution-independent — the Pi did the pixel→fraction conversion.
static int16_t computeOmega(bool manualActive, bool tracking, int16_t manualOmega,
                            int16_t errN, int16_t derrN, int16_t kpX100,
                            int16_t maxOmega, int16_t tdX1000) {
    if (manualActive) {
        return manualOmega;
    }
    if (!tracking) {
        return 0;
    }
    const float err  = (float)errN  / ERR_N_SCALE;
    const float derr = (float)derrN / DERR_N_SCALE;
    const float kp   = (float)kpX100 / KP_SCALE;
    const float td   = (float)tdX1000 / TD_SCALE;

    float drive = (err + td * derr) * kp;
    if (drive >  1.0f) drive =  1.0f;
    if (drive < -1.0f) drive = -1.0f;

    return (int16_t)lroundf(drive * (float)maxOmega);
}

void loop() {
    // Snapshot the ISR-owned command atomically.
    noInterrupts();
    const bool    manualActive = g_manualActive;
    const bool    tracking     = g_tracking;
    const int16_t manualOmega  = g_manualOmega;
    const int16_t errN         = g_errN;
    const int16_t derrN        = g_derrN;
    const int16_t kpX100       = g_kpX100;
    const int16_t maxOmega     = g_maxOmega;
    const int16_t tdX1000      = g_tdX1000;
    interrupts();

    const int16_t omega  = computeOmega(manualActive, tracking, manualOmega,
                                        errN, derrN, kpX100, maxOmega, tdX1000);
    const bool    active = (manualActive || tracking) && (omega != 0);

    runStepper(active, omega);

    // Heartbeat: print latest command + packet counters so the dev can see
    // the SPI bytes arriving and confirm checksum health.
    static uint32_t lastBeat = 0;
    static int16_t  lastShownOmega = 0x7FFF;
    const uint32_t now = millis();
    const bool changed = (omega != lastShownOmega);
    if (changed || (now - lastBeat >= HEARTBEAT_MS)) {
        lastBeat = now;
        lastShownOmega = omega;
        noInterrupts();
        const uint32_t good = g_goodPackets;
        const uint32_t bad  = g_badPackets;
        interrupts();
        Serial.print(F("[spi] man="));
        Serial.print(manualActive ? 1 : 0);
        Serial.print(F(" trk="));
        Serial.print(tracking ? 1 : 0);
        Serial.print(F(" omega="));
        Serial.print(omega);
        Serial.print(F("  good="));
        Serial.print(good);
        Serial.print(F(" bad="));
        Serial.println(bad);
    }
}
