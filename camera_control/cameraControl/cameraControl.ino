// ---------------------------------------------------------
// Automated Ball Tracking - MOTOR CONTROL (Timer2-driven ramp)
// ---------------------------------------------------------
//
// ARCHITECTURE
// ============
// Timer1 (16-bit): generates step pulses on stepPin via hardware
//                  CTC + COM1A0 auto-toggle. OCR1A = period in μs.
//                  No Timer1 ISR — purely hardware. Zero jitter,
//                  zero CPU cost between matches.
//
// Timer2 (8-bit):  1 kHz tick (CTC, /64 prescaler, OCR2A=249).
//                  ISR walks `cur_v_idx` toward `target_v_idx` once
//                  every `accel_skip` ticks. After each step it
//                  writes `OCR1A = vel_table[|cur_v_idx|]` so the
//                  pulse train smoothly tracks the velocity ramp.
//                  Same ISR also drives dir/en pins.
//
// VELOCITY GRID
// =============
// `cur_v_idx`, `target_v_idx` are SIGNED in [-V_TABLE_N, +V_TABLE_N].
// Magnitude indexes `vel_table[]`; sign drives dirPin. At idx=0 the
// motor is muted (toggle output disabled, enPin HIGH, stepPin LOW).
//
// Velocity at idx i (in user units):  v = max_omega * i / V_TABLE_N
// OCR1A at idx i:                     vel_table[i] = round(11172 / v)
//
// The table is rebuilt whenever max_omega changes (rare — only when
// the Python slider moves). Rebuild blocks interrupts for ~2 ms which
// can drop a couple of step pulses during the transition; acceptable.
//
// ANY-TO-ANY TRANSITIONS
// ======================
// The ISR always pulls cur_v_idx toward target_v_idx by ±1. Crossing
// through zero is automatic: at idx=0 we mute the motor for one tick,
// then the next non-zero idx flips the dir pin and accelerates the
// other way. No mechanical jolt because we always pass through v=0.
//
// ACCELERATION CONTROL
// ====================
// Only one knob: `accel_skip` ∈ [1, 250]. Effective α (user/sec²) =
//
//     α = (max_omega / V_TABLE_N) * 1000 / accel_skip
//
// At accel_skip=1 (one idx per ms) we get the maximum α the current
// max_omega allows. Python sends desired α; firmware computes skip.
// Cap at max_omega=100, V_TABLE_N=200 → α_max ≈ 500 user/sec².
// At max_omega=40 → α_max ≈ 200. Document this in README.
//
// SERIAL PROTOCOL (additions)
// ===========================
// Existing 7-field CSV packet stays unchanged for backward compat.
// Three new line-prefixed commands handle drive-tuning state:
//
//     A<float>\n      set acceleration α (user-units per sec²)
//     M<0|1>\n        manual-omega override on/off
//     O<float>\n      manual-omega target (user units, signed)
//
// When manual override is on the camera P-control branch is bypassed;
// physical jog buttons still take precedence over both.

const int stepPin = 9;          // OC1A — Timer1 hardware-toggles this
const int dirPin = 5;           // PD5
const int enPin = 10;           // PB2  (active-low: LOW = driver enabled)
const int buttonPin1 = 12;
const int buttonPin2 = 7;

// ---- Scope-debug pulse on every completed serial packet ----------------
// Toggles D8 (PB0) the instant the parser sees a '\n'. Hook a scope or
// logic analyser to D8 and you'll see one edge per incoming packet (so
// 120 packets/sec = 60 Hz square wave). Useful to confirm the actual
// rate Python achieves end-to-end (USB framing + Python loop + UART)
// without trusting the printed "Logic FPS" number.
//
// Set to 0 to disable and free PB0 for other use.
#define DEBUG_RX_PULSE 1
#if DEBUG_RX_PULSE
const int debugPin = 8;         // PB0
#endif

// --- Camera P-control inputs (set by Python every CSV packet) ------------
float angleX = 0, angleY = 0;
float normX = 0.0, normY = 0.0;
float Kp = 1.0;
bool isTracking = false;
float max_omega = 40.0;
const float SENSOR_HALF_WIDTH_PX = 320.0;
const float manual_speed = 10.0;

// --- Drive-tuning inputs (set by Python out-of-band commands) ------------
volatile uint8_t accel_skip = 5;     // 1..250; 1 = max accel for current max_omega
float last_alpha = 100.0;            // last α requested via "A<...>"; used to
                                     // recompute accel_skip when max_omega changes
bool manual_active = false;          // true => use manual_omega instead of camera
float manual_omega = 0.0;            // user-units, signed

// --- Watchdog: zero target if Python goes silent -------------------------
uint32_t last_packet_time = 0;
const uint32_t timeout_ms = 250;

String inputBuffer = "";

// --- Velocity grid -------------------------------------------------------
// V_TABLE_N is the discretisation of the [0, max_omega] range. 200 is a
// nice middle-ground: at max_omega=100 each idx ≈ 0.5 user-unit
// (≈45 step/sec), invisible to the human eye. Memory: 2 × (N+1) bytes.
const int16_t V_TABLE_N = 200;
uint16_t vel_table[V_TABLE_N + 1];

volatile int16_t cur_v_idx = 0;       // ISR-owned current speed index
volatile int16_t target_v_idx = 0;    // loop-owned target speed index

// Cached so we don't toggle the dir pin every Timer2 tick when nothing
// changed (cheap, but keeps the line quiet for scope debugging). Init
// must MATCH the actual physical pin state set in setup() — otherwise
// the very first non-zero idx skips the dir pin write and the motor
// spins the wrong way for one ramp cycle.
volatile bool last_dir_high = false;

void recompute_accel_skip() {
    // skip = round(1000 * (max_omega / V_TABLE_N) / last_alpha), clamped.
    // Called both from the A-command handler and from the CSV branch when
    // max_omega changes — so α stays physically the same in user-units/sec²
    // regardless of which knob moved.
    float alpha = last_alpha;
    if (alpha < 1.0) alpha = 1.0;
    float ms_per_idx = 1000.0 * (max_omega / (float)V_TABLE_N) / alpha;
    int s = (int)(ms_per_idx + 0.5);
    if (s < 1) s = 1;
    if (s > 250) s = 250;
    accel_skip = (uint8_t)s;
}

void rebuild_vel_table() {
    // Index 0 is a sentinel — we never read it in normal operation
    // (motor muted at idx=0). Fill it with a safe huge period anyway.
    vel_table[0] = 65535;
    for (int16_t i = 1; i <= V_TABLE_N; i++) {
        float omega_user = max_omega * (float)i / (float)V_TABLE_N;
        // OCR1A = pulse period in μs = 11172 / omega_user (legacy
        // calibration: 1 user-unit ≈ 89.5 step/sec — see README).
        float ocr = 11172.0 / omega_user;
        if (ocr > 65535.0) ocr = 65535.0;
        if (ocr < 20.0) ocr = 20.0;        // hard floor — driver max rate
        vel_table[i] = (uint16_t)ocr;
    }
}

void setup() {
    Serial.begin(115200);
    inputBuffer.reserve(64);

    pinMode(buttonPin1, INPUT_PULLUP);
    pinMode(buttonPin2, INPUT_PULLUP);
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enPin, OUTPUT);
    digitalWrite(enPin, HIGH);     // disabled until first non-zero idx
    digitalWrite(dirPin, LOW);     // matches `last_dir_high = false` cache

#if DEBUG_RX_PULSE
    pinMode(debugPin, OUTPUT);
    digitalWrite(debugPin, LOW);
#endif

    rebuild_vel_table();

    // ---- Timer1: step-pulse generator (CTC + COM1A0 toggle) ----
    // We do NOT enable a Timer1 interrupt — the pin toggles are pure
    // hardware. OCR1A is updated from the Timer2 ISR.
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1B |= (1 << WGM12);        // CTC mode (TOP = OCR1A)
    TCCR1B |= (1 << CS11);         // prescaler /8 → tick = 0.5 μs
    OCR1A = 65535;                 // very long period, motor effectively idle

    // ---- Timer2: 1 kHz velocity-update ISR ----
    // f_tick = 16 MHz / 64 / (OCR2A+1). OCR2A=249 → 1000 Hz exact.
    TCCR2A = (1 << WGM21);         // CTC (TOP = OCR2A)
    TCCR2B = (1 << CS22);          // prescaler /64
    OCR2A = 249;
    TIMSK2 |= (1 << OCIE2A);       // enable compare-A interrupt

    sei();
}

ISR(TIMER2_COMPA_vect) {
    // ---- Step 1: maybe advance cur_v_idx toward target_v_idx ----
    static uint8_t skip = 0;
    if (++skip >= accel_skip) {
        skip = 0;
        if (cur_v_idx < target_v_idx) cur_v_idx++;
        else if (cur_v_idx > target_v_idx) cur_v_idx--;
    }

    // ---- Step 2: drive motor based on cur_v_idx ----
    int16_t v = cur_v_idx;
    if (v == 0) {
        // Motor at rest: mute pulse output and disable driver to save
        // current. Pin LOW guarantees no half-pulse glitch on resume.
        TCCR1A &= ~(1 << COM1A0);
        PORTB |= (1 << PB2);       // enPin HIGH (active-low: disabled)
        PORTB &= ~(1 << PB1);      // stepPin LOW (idle level)
        return;
    }

    // Motor moving: ensure driver enabled and dir pin matches sign.
    PORTB &= ~(1 << PB2);          // enPin LOW (enabled)
    bool want_high = (v > 0);
    if (want_high != last_dir_high) {
        if (want_high) PORTD |= (1 << PD5);
        else           PORTD &= ~(1 << PD5);
        last_dir_high = want_high;
    }

    uint16_t mag = (v > 0) ? (uint16_t)v : (uint16_t)(-v);
    if (mag > V_TABLE_N) mag = V_TABLE_N;     // safety clamp

    OCR1A = vel_table[mag];
    TCCR1A |= (1 << COM1A0);       // hardware toggle ON (resume pulses)
}

void loop() {
    // ---- 1. SERIAL READ ----
    while (Serial.available() > 0) {
        char inChar = (char)Serial.read();
        if (inChar == '\n') {
#if DEBUG_RX_PULSE
            // Atomic 1-cycle toggle of PB0 (D8): writing 1 to a PIN
            // register bit on AVR XOR-toggles the corresponding PORT
            // bit. Faster and safer than digitalWrite() — no chance of
            // a partial read-modify-write race with the Timer2 ISR.
            PINB = (1 << PB0);
#endif
            parseIncomingData(inputBuffer);
            inputBuffer = "";
            last_packet_time = millis();
        } else {
            inputBuffer += inChar;
        }
    }

    // ---- 2. STATUS ----
    bool python_active = (millis() - last_packet_time < timeout_ms);

    // ---- 3. COMPUTE TARGET OMEGA (user units) ----
    bool btnLeft  = (digitalRead(buttonPin1) == LOW);
    bool btnRight = (digitalRead(buttonPin2) == LOW);
    float omega_target;

    if (btnLeft) {
        // Physical jog buttons override software completely.
        omega_target = -manual_speed;
    } else if (btnRight) {
        omega_target = manual_speed;
    } else if (python_active && manual_active) {
        // Drive-tuning mode: take the omega target straight from the
        // Python slider, ignoring camera input. Used to characterise the
        // motor (max speed, ramp limits) with the shaft disconnected.
        omega_target = manual_omega;
    } else if (python_active && isTracking) {
        // Camera mode: normalised P-control on pixel error.
        float err = normX / SENSOR_HALF_WIDTH_PX;
        omega_target = err * max_omega * Kp;
    } else {
        omega_target = 0;
    }

    // Clamp to [-max_omega, +max_omega] (matters when Kp > 1.0 or for
    // overzealous manual_omega slider input).
    if (omega_target >  max_omega) omega_target =  max_omega;
    if (omega_target < -max_omega) omega_target = -max_omega;

    // ---- 4. CONVERT omega → idx and publish target_v_idx atomically ----
    int16_t new_target = 0;
    if (max_omega > 0.1) {
        new_target = (int16_t)(omega_target * (float)V_TABLE_N / max_omega);
    }
    if (new_target >  V_TABLE_N) new_target =  V_TABLE_N;
    if (new_target < -V_TABLE_N) new_target = -V_TABLE_N;

    // 16-bit write needs guarding from concurrent ISR read.
    cli();
    target_v_idx = new_target;
    sei();
}

void parseIncomingData(String line) {
    line.trim();
    if (line.length() == 0) return;

    char prefix = line.charAt(0);

    // ---- A<float>: acceleration in user-units / sec² ----
    if (prefix == 'A') {
        last_alpha = line.substring(1).toFloat();
        recompute_accel_skip();
        return;
    }

    // ---- M<0|1>: manual-omega override flag ----
    if (prefix == 'M') {
        manual_active = (line.length() > 1 && line.charAt(1) == '1');
        return;
    }

    // ---- O<float>: manual-omega target ----
    if (prefix == 'O') {
        manual_omega = line.substring(1).toFloat();
        return;
    }

    // ---- otherwise: legacy 7-field CSV (camera P-control) ----
    int idx1 = line.indexOf(',');
    int idx2 = line.indexOf(',', idx1 + 1);
    int idx3 = line.indexOf(',', idx2 + 1);
    int idx4 = line.indexOf(',', idx3 + 1);
    int idx5 = line.indexOf(',', idx4 + 1);
    int idx6 = line.indexOf(',', idx5 + 1);

    if (idx1 > 0 && idx6 > 0) {
        angleX     = line.substring(0, idx1).toFloat();
        angleY     = line.substring(idx1 + 1, idx2).toFloat();
        normX      = line.substring(idx2 + 1, idx3).toFloat();
        normY      = line.substring(idx3 + 1, idx4).toFloat();
        Kp         = line.substring(idx4 + 1, idx5).toFloat();
        isTracking = (line.substring(idx5 + 1, idx6).toInt() == 1);

        float new_max_omega = line.substring(idx6 + 1).toFloat();
        if (new_max_omega > 0.1 &&
            fabs(new_max_omega - max_omega) > 0.05) {
            // Significant change → rebuild the table. We disable
            // interrupts so the ISR doesn't read a half-rebuilt entry.
            // Cost: ~2 ms with V_TABLE_N=200 float divisions. Drops a
            // couple of step pulses but the user is moving a slider,
            // so a brief stutter is acceptable.
            max_omega = new_max_omega;
            cli();
            rebuild_vel_table();
            // accel_skip depends on (max_omega / V_TABLE_N) / last_alpha,
            // so recompute it here — otherwise the *physical* α would
            // silently drift when the user moves the Max Speed slider.
            recompute_accel_skip();
            sei();
        } else {
            max_omega = new_max_omega;
        }
    }
}
