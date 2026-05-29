// --- Test triangle generator for err_raw ---
int test_err_raw = -10;
int test_err_dir = 1;
unsigned long last_err_update = 0;
// --- Debug: safe OCR1A printout ---
volatile uint16_t last_ocr1a = 0;
unsigned long last_print_time = 0;
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
// Velocity at idx i (rad/s):          v = max_omega_rad_s * i / V_TABLE_N
// OCR1A at idx i:                     vel_table[i] = round(11172 / v)
//
// The table is rebuilt when max_omega_rad_s changes (rare in test mode).
// Rebuild blocks interrupts briefly and may drop a couple of pulses.
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
// Only one knob: `accel_skip` ∈ [1, 250]. Effective α (rad/sec²) =
//
//     α = (max_omega_rad_s / V_TABLE_N) * 1000 / accel_skip
//
// At accel_skip=1 (one idx per ms) we get the maximum α the current
// max_omega_rad_s allows.
//
// TEST MODE I/O
// =============
// This sketch is intentionally standalone (no Python sync / serial PID data).
//
// A0: task (setpoint) analog input, [0..1023], 512 = zero angle
// A1: feedback angle sensor,      [0..1023], 512 = zero angle
//
// Control law (P only):
//   err_counts = A0 - A1
//   err_rad    = err_counts * RAD_PER_ADC_COUNT
//   omega_target_rad_s = Kp * err_rad
//
// Kp is in [1/sec], so omega_target is in [rad/sec].

const int stepPin = 9;          // OC1A — Timer1 hardware-toggles this
const int dirPin = 5;           // PD5
const int enPin = 10;           // PB2  (active-low: LOW = driver enabled)
const int buttonPin1 = 12;
const int buttonPin2 = 7;
const int taskPin = A0;
const int feedbackPin = A1;

// ---- Scope-debug analog readout of the regulator command ---------------
// 8-bit PWM on D6 (OC0A, Timer0 — shared with millis(), so we use
// analogWrite() and DON'T touch the timer config). Output represents
// `omega_target_rad_s` AFTER clamp to [-max_omega_rad_s, +max_omega_rad_s], mapped:
//
//     omega_target_rad_s = +max_omega_rad_s → PWM 255  ( duty ≈ 100% )
//     omega_target_rad_s =               0 → PWM 128  ( duty ≈  50% )
//     omega_target_rad_s = -max_omega_rad_s → PWM   1  ( duty ≈   0% )
//
// I.e. signed 8-bit value in [-127, +127] offset by 128. With a
// simple RC low-pass (e.g. R=1k, C=10µF → fc≈16 Hz) you get an analog
// trace you can compare on the scope against ball position / strobe.
// Without RC the scope sees the 976 Hz square wave whose duty encodes
// the command — average it visually.
//
// Set to 0 to disable and free PD6 for other use.
#define DEBUG_OMEGA_PWM 1
#if DEBUG_OMEGA_PWM
const int omegaPwmPin = 6;      // PD6 — OC0A (Timer0 8-bit PWM)
#endif

// --- Analog P-control tuning ---------------------------------------------
// Mapping assumption: full ADC span (1024 counts) equals 2*pi rad.
// If your sensor/command potentiometer has a different physical range,
// tune this constant.
const float RAD_PER_ADC_COUNT = 6.28318530718f / 1024.0f;
float Kp = 0.25f;                  // [1/sec], P gain for omega = Kp * err_rad
float max_omega_rad_s = 200.0f;    // speed clamp in [rad/sec]
const float manual_speed = 10.0;
int err_raw = 0;

// 1..250; 1 = max accel for current max_omega_rad_s
volatile uint8_t accel_skip = 5;

// --- Velocity grid -------------------------------------------------------
// V_TABLE_N discretises the [0, max_omega_rad_s] range.
// Memory: 2 × (N+1) bytes.
// Таблица скоростей не нужна — вычисляем OCR1A по формуле на лету

volatile int16_t cur_v_idx = 0;       // ISR-owned current speed index
volatile int16_t target_v_idx = 0;    // loop-owned target speed index

// Cached so we don't toggle the dir pin every Timer2 tick when nothing
// changed (cheap, but keeps the line quiet for scope debugging). Init
// must MATCH the actual physical pin state set in setup() — otherwise
// the very first non-zero idx skips the dir pin write and the motor
// spins the wrong way for one ramp cycle.
volatile bool last_dir_high = false;


void setup() {
    Serial.begin(9600);
    pinMode(buttonPin1, INPUT_PULLUP);
    pinMode(buttonPin2, INPUT_PULLUP);
    pinMode(taskPin, INPUT);
    pinMode(feedbackPin, INPUT);
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enPin, OUTPUT);
    digitalWrite(enPin, LOW);     // disabled until first non-zero idx
    digitalWrite(dirPin, LOW);     // matches `last_dir_high = false` cache

    #if DEBUG_OMEGA_PWM
        pinMode(omegaPwmPin, OUTPUT);
        analogWrite(omegaPwmPin, 128);  // start at "zero omega" duty
    #endif

    // Таблица скоростей не используется

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
    // --- RAMP LOGIC DISABLED: используем мгновенную ошибку --- 
    // static uint8_t skip = 0;
    // if (++skip >= accel_skip) {
    //     skip = 0;
    //     if (cur_v_idx < target_v_idx) cur_v_idx++;
    //     else if (cur_v_idx > target_v_idx) cur_v_idx--;
    // }

    int16_t v = err_raw; // или target_v_idx, если нужен масштаб
    if (v == 0) {
        TCCR1A &= ~(1 << COM1A0);
        PORTB &= ~(1 << PB1);
        return;
    }

    bool want_high = (v > 0);
    if (want_high != last_dir_high) {
        if (want_high) PORTD |= (1 << PD5);
        else           PORTD &= ~(1 << PD5);
        last_dir_high = want_high;
    }

    int16_t mag = (v > 0) ? v : -v;
    float ocr;
    if (mag == 0) {
        ocr = 65535.0f;
    } else {
        ocr = 32000.0f / (float)mag;
        if (ocr > 65535.0f) ocr = 65535.0f;
        if (ocr < 20.0f) ocr = 20.0f;
    }
    OCR1A = (uint16_t)ocr;
    last_ocr1a = OCR1A;
    TCCR1A |= (1 << COM1A0);
}

void loop() {
    // ---- 1. COMPUTE TARGET OMEGA [rad/s] FROM ANALOG TASK/FEEDBACK ----
    bool btnLeft  = (digitalRead(buttonPin1) == LOW);
    bool btnRight = (digitalRead(buttonPin2) == LOW);
    float omega_target_rad_s;

    if (btnLeft) {
        // Physical jog buttons override software completely.
        omega_target_rad_s = -manual_speed;
    } else if (btnRight) {
        omega_target_rad_s = manual_speed;
    } else {
        // --- Обычный режим: ошибка = A0 - A1 ---
        // Для теста треугольной ошибки раскомментировать ниже:
        // unsigned long now = millis();
        // if (now - last_err_update >= 1000) {
        //     test_err_raw += test_err_dir;
        //     if (test_err_raw >= 20) {
        //         test_err_raw = 20;
        //         test_err_dir = -1;
        //     } else if (test_err_raw <= -20) {
        //         test_err_raw = -20;
        //         test_err_dir = 1;
        //     }
        //     last_err_update = now;
        // }
        // err_raw = test_err_raw;

        err_raw = analogRead(taskPin) - analogRead(feedbackPin);
        float err = (float)(err_raw) * RAD_PER_ADC_COUNT;
        omega_target_rad_s = Kp * err;
    }

    // Clamp to [-max_omega_rad_s, +max_omega_rad_s].
    if (omega_target_rad_s >  max_omega_rad_s) omega_target_rad_s =  max_omega_rad_s;
    if (omega_target_rad_s < -max_omega_rad_s) omega_target_rad_s = -max_omega_rad_s;

#if DEBUG_OMEGA_PWM
    // Map omega_target → 8-bit PWM duty, centred at 128. We avoid the
    // exact 0 and 256 codes (rare but possible due to FP rounding) by
    // clamping into [1, 255] — keeps the output strictly bipolar and
    // matches the docstring at the top of the file.
    {
        float k = (max_omega_rad_s > 0.1f) ? (omega_target_rad_s / max_omega_rad_s) : 0.0f;
        if (k >  1.0f) k =  1.0f;
        if (k < -1.0f) k = -1.0f;
        int pwm = 128 + (int)(k * 127.0f);
        if (pwm < 1)   pwm = 1;
        if (pwm > 255) pwm = 255;
                // Serial.println(pwm);
        // analogWrite(omegaPwmPin, pwm);
    }
#endif

    // ---- 2. CONVERT omega_target_rad_s → idx and publish atomically ----
    // Fixed gain here keeps small error changes visible at the motor.
    // 1 raw ADC count now produces about 1 idx step when Kp=10.

    // Абсолютно линейная чувствительность: target_v_idx = err_raw
    int16_t new_target = err_raw;
    if (new_target >  200) new_target =  200;
    if (new_target < -200) new_target = -200;

    // 16-bit write needs guarding from concurrent ISR read.
    cli();
    target_v_idx = new_target;
    sei();

    // Безопасный вывод OCR1A не чаще 20 раз/сек
    unsigned long now = millis();
    if (now - last_print_time > 50) {
        uint16_t ocr;
        cli();
        ocr = last_ocr1a;
        sei();
        Serial.println(ocr);
                Serial.println(err_raw);
        last_print_time = now;
    }
}
