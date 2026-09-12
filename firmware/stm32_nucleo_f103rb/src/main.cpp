/**
 * STM32 Nucleo-F103RB — SPI drive controller with pot feedback (Step 3.2).
 *
 * Что делает эта прошивка
 * -----------------------
 *  • Работает SPI-slave на SPI2 (Pi 4 — master через /dev/spidev0.0).
 *  • Принимает 20-байтовый запрос (control-frame) и ОДНОВРЕМЕННО (full-duplex)
 *    отдаёт 20-байтовый ответ с телеметрией: сырое значение потенциометра,
 *    текущую команду omega, оценку скорости вала, счётчики good/bad-пакетов.
 *  • Читает потенциометр на PA0 (12-битный ADC), EMA-сглаживает и оценивает
 *    угловую скорость вала — реальный «тахо-фидбэк» вместо шумной производной
 *    ошибки от камеры.
 *  • Считает закон управления НА МК (Python шлёт только нормированные входы):
 *        manual_active → omega = manual_omega
 *        tracking      → drive = clamp((err_n + Td·derr_n - Kd_pot·pot_vel_n)·Kp, ±1)
 *                        omega = drive · max_omega
 *        иначе         → omega = 0 (катушки отпущены)
 *  • Печатает heartbeat в USART2 VCP: значения команды + pot_raw + счётчики.
 *  • Гоняет драйвер шагового двигателя: направление = sign(omega), частота
 *    шагов ∝ |omega|.
 *
 * SPI PROTOCOL (must match Stm32SpiHandler in hardware.py + tools/spi_test.py)
 * ---------------------------------------------------------------------------
 * REQUEST — Pi → STM32, 20 байт, MSB-first, SPI mode 0:
 *      [0]      0xAA        preamble
 *      [1]      0x55        preamble
 *      [2]      flags       bit0=manual_active, bit1=tracking
 *      [3..4]   manual_omega  int16 LE   (user units, знаковая)
 *      [5..6]   err_n         int16 LE   (нормир. ошибка ×10000, ±1.0)
 *      [7..8]   derr_n        int16 LE   (нормир. производная ошибки ×1000/с)
 *      [9..10]  kp_x100       int16 LE   (Kp × 100)
 *      [11..12] max_omega     int16 LE   (потолок скорости, user units)
 *      [13..14] td_x1000      int16 LE   (Td × 1000, сек — вес derr_n от камеры)
 *      [15..16] pot_center    uint16 LE  (0..4095, зарезервировано для soft-limits)
 *      [17..18] kd_pot_x1000  int16 LE   (Kd для скорости вала, знаковый — direction)
 *      [19]     xor           XOR байтов [0..18]
 *
 * RESPONSE — STM32 → Pi, 20 байт (shifts out по MISO одновременно с запросом):
 *      [0]      0xBB        preamble
 *      [1]      0x66        preamble
 *      [2..3]   pot_raw        uint16 LE  (0..4095, EMA-сглажено)
 *      [4..5]   omega_out      int16 LE   (текущая команда, что реально едет в мотор)
 *      [6..7]   pot_vel        int16 LE   (скорость вала, ADC-units/сек, кламп int16)
 *      [8..11]  good_packets   uint32 LE  (счётчик валидных запросов от Pi)
 *      [12..15] bad_packets    uint32 LE  (сбои XOR / потеря синхронизации)
 *      [16]     status         bit0=manual, bit1=tracking, bit2=driving
 *      [17..18] reserved       (0)
 *      [19]     xor            XOR байтов [0..18]
 *
 *  Синхронизация tx-буфера: сброс g_txIdx на приёме 0xAA (первый байт кадра).
 *  Между кадрами DR предзаряжен байтом tx[0]=0xBB — Pi всегда видит корректный
 *  preamble в позиции 0 своего returned-массива. Дальше следует остальная
 *  часть кадра. При десинке (XOR fail, дроп байта) FSM пересинхронится на
 *  следующем 0xAA, что автоматически перевыравнивает и tx-канал.
 *
 * SPI2 slave wiring (Pi 4 SPI0 master ──► Nucleo CN10):
 *      signal     Pi 4 (BCM / hdr-pin)        Nucleo STM32 pin   CN10 pin
 *      MOSI       GPIO10 / pin 19    ───────► PB15 (SPI2_MOSI)   CN10-26
 *      MISO       GPIO9  / pin 21    ◄─────── PB14 (SPI2_MISO)   CN10-28
 *      SCLK       GPIO11 / pin 23    ───────► PB13 (SPI2_SCK)    CN10-30
 *      CE0/NSS    GPIO8  / pin 24    ───────► PB12 (SPI2_NSS)    CN10-16
 *      GND        any GND (pin 20/25)──────── GND                CN10-20
 *
 * Motor pins (см. таблицу вокруг stepPin/dirPin/enPin ниже).
 *
 * Feedback pin
 * ------------
 *  Потенциометр — на Arduino-header A0 = PA0 (ADC1_IN0). Ремённая связь с
 *  валом камеры, диапазон ~0..4095 (12-битный ADC). Средняя точка задаётся
 *  из UI кнопкой «Set center» и хранится на стороне Pi (для UI-индикации);
 *  прошивка сама pot_center пока не использует, но получает — на будущее для
 *  мягких концевых ограничителей.
 *
 *  STROBE-вход был убран: у нашей UVC-камеры STRB пульсирует построчно
 *  (kHz-диапазон), а не «раз на кадр», поэтому для замера возраста кадра
 *  он не годится. Пин PA1 снова свободен.
 */

#include <Arduino.h>

// ──────────────────────────────────────────────────────────────────────────
// Motor pin configuration (Arduino-header numbering — see file header table).
// ──────────────────────────────────────────────────────────────────────────
constexpr int stepPin = 9;     // PC7
constexpr int dirPin  = 5;     // PB4
constexpr int enPin   = 10;    // PB6 — active-low: LOW = driver enabled
constexpr int potPin  = PA0;   // ADC1_IN0, 12-битный

// Map |omega| (user units) → step frequency. omega=40 → 1000 Hz reproduces
// the old MANUAL_STEP_HZ; legacy firmware clamped omega to ±200, so the top
// end here is 200×25 = 5000 Hz. Clamp keeps us inside what a bit-banged
// loop can emit reliably.
constexpr uint32_t STEP_HZ_PER_UNIT = 25;
constexpr uint32_t STEP_HZ_MIN      = 50;
constexpr uint32_t STEP_HZ_MAX      = 6000;

// Heartbeat в USART2 VCP каждые N мс (плюс на каждое изменение omega).
constexpr uint32_t HEARTBEAT_MS = 250;

// ──────────────────────────────────────────────────────────────────────────
// SPI framing protocol (must match firmware/.../tools/spi_test.py и
// Stm32SpiHandler в hardware.py). 20-байтовый фиксированный фрейм, MSB-first.
// ──────────────────────────────────────────────────────────────────────────
constexpr uint8_t  SPI_SYNC0    = 0xAA;
constexpr uint8_t  SPI_SYNC1    = 0x55;
constexpr uint8_t  SPI_TX_SYNC0 = 0xBB;   // preamble ответа (MISO)
constexpr uint8_t  SPI_TX_SYNC1 = 0x66;
constexpr uint8_t  SPI_PKT_LEN  = 20;

// Fixed-point scales, согласованные с Pi. Синхронно править в hardware.py.
constexpr float ERR_N_SCALE  = 10000.0f;   // err_n    int16  → [-1.0, +1.0]
constexpr float DERR_N_SCALE = 1000.0f;    // derr_n   int16  → per-second
constexpr float KP_SCALE     = 100.0f;     // kp_x100  int16  → Kp
constexpr float TD_SCALE     = 1000.0f;    // td_x1000 int16  → Td (сек)
constexpr float KD_POT_SCALE = 1000.0f;    // kd_pot_x1000 int16 → Kd_pot

// Нормировка pot-скорости. При «полном ходе» вала ~ADC diff = 4096 единиц.
// POT_VEL_NORM = 4096 units/sec → нормированная pot-скорость = 1.0. Значит
// Kd_pot=0.1 при быстром движении вала даёт вклад в drive ~0.1 — комфортный
// стартовый масштаб для настройки в UI.
constexpr float POT_VEL_NORM = 4096.0f;

// Как часто семплируем ADC (µs). 1 кГц — с запасом: пропускная способность
// ADC у F103 ~1 Msps, а нам достаточно 1 kHz, чтобы производную не «зубить».
constexpr uint32_t POT_SAMPLE_US = 1000;

// Коэффициенты EMA. α (numerator) / (α + β):
//   raw:  α=1, β=3   → time-constant ~4 семпла (~4 мс)
//   vel:  α=1, β=7   → сильнее гасим шум производной (~8 мс)
// В целочисленной форме: new = (old*β + sample*α) / (α+β).
constexpr uint16_t POT_EMA_A = 1;
constexpr uint16_t POT_EMA_B = 3;
constexpr int32_t  VEL_EMA_A = 1;
constexpr int32_t  VEL_EMA_B = 7;

// ──────────────────────────────────────────────────────────────────────────
// Shared state (volatile: writers in ISR, readers в loop() под noInterrupts).
// ──────────────────────────────────────────────────────────────────────────
static volatile bool     g_manualActive = false;
static volatile bool     g_tracking     = false;
static volatile int16_t  g_manualOmega  = 0;
static volatile int16_t  g_errN         = 0;
static volatile int16_t  g_derrN        = 0;
static volatile int16_t  g_kpX100       = 0;
static volatile int16_t  g_maxOmega     = 0;
static volatile int16_t  g_tdX1000      = 0;
static volatile uint16_t g_potCenter    = 2048;  // пока не используется в законе
static volatile int16_t  g_kdPotX1000   = 0;
static volatile uint32_t g_goodPackets  = 0;
static volatile uint32_t g_badPackets   = 0;

// Обновляется в main loop, читается в ISR (для укладки в tx-буфер).
static volatile uint16_t g_potRaw = 2048;
static volatile int16_t  g_potVel = 0;
static volatile int16_t  g_omegaOut = 0;      // последняя команда мотору
static volatile uint8_t  g_statusOut = 0;

// ──────────────────────────────────────────────────────────────────────────
// TX-буфер ответа: заполняется в main loop, читается в SPI ISR по 1 байту.
// g_txIdx — индекс СЛЕДУЮЩЕГО байта, который будет положен в DR.
// ──────────────────────────────────────────────────────────────────────────
static volatile uint8_t g_txBuf[SPI_PKT_LEN];
static volatile uint8_t g_txIdx = 1;   // tx[0] изначально предзаряжен в DR

// HAL handle для SPI2. Инициализируется в spiSlaveBegin().
static SPI_HandleTypeDef hspi2;

// ──────────────────────────────────────────────────────────────────────────
// Утилиты little-endian.
// ──────────────────────────────────────────────────────────────────────────
static inline int16_t le16s(const uint8_t* p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static inline uint16_t le16u(const uint8_t* p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static inline void write16(volatile uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}
static inline void write32(volatile uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

// ──────────────────────────────────────────────────────────────────────────
// FSM разбора входного пакета. Вызывается по 1 байту из ISR.
// Ищет 0xAA 0x55 preamble, собирает тело, проверяет XOR, публикует поля.
// На старте нового кадра (byte 0 = 0xAA @ state 0) синхронизирует tx-канал:
// сбрасывает g_txIdx на 1, так что ISR положит tx[1] в DR для следующего
// байта — гарантируя, что позиция 0 у Pi всегда tx[0]=0xBB.
// ──────────────────────────────────────────────────────────────────────────
static inline void spiFeedByte(uint8_t b) {
    static uint8_t state = 0;
    static uint8_t buf[SPI_PKT_LEN];

    switch (state) {
        case 0:                                  // ищем sync0
            if (b == SPI_SYNC0) {
                buf[0] = b;
                state = 1;
                g_txIdx = 1;                     // resync tx-канала на границе кадра
            }
            break;
        case 1:                                  // ищем sync1
            if (b == SPI_SYNC1)      { buf[1] = b; state = 2; }
            else if (b == SPI_SYNC0) { state = 1; g_txIdx = 1; }
            else                     { state = 0; }
            break;
        default:                                 // тело 2..PKT_LEN-1
            buf[state] = b;
            if (++state >= SPI_PKT_LEN) {
                state = 0;
                uint8_t cs = 0;
                for (uint8_t i = 0; i < SPI_PKT_LEN - 1; ++i) cs ^= buf[i];
                if (cs == buf[SPI_PKT_LEN - 1]) {
                    g_manualActive = (buf[2] & 0x01) != 0;
                    g_tracking     = (buf[2] & 0x02) != 0;
                    g_manualOmega  = le16s(&buf[3]);
                    g_errN         = le16s(&buf[5]);
                    g_derrN        = le16s(&buf[7]);
                    g_kpX100       = le16s(&buf[9]);
                    g_maxOmega     = le16s(&buf[11]);
                    g_tdX1000      = le16s(&buf[13]);
                    g_potCenter    = le16u(&buf[15]);
                    g_kdPotX1000   = le16s(&buf[17]);
                    g_goodPackets++;
                } else {
                    g_badPackets++;
                }
            }
            break;
    }
}

// SPI2 IRQ. RXNE → read DR → FSM. TXE → положить следующий tx-байт.
extern "C" void SPI2_IRQHandler(void) {
    if (__HAL_SPI_GET_FLAG(&hspi2, SPI_FLAG_RXNE)) {
        const uint8_t b = (uint8_t)(hspi2.Instance->DR);
        spiFeedByte(b);
        if (__HAL_SPI_GET_FLAG(&hspi2, SPI_FLAG_TXE)) {
            hspi2.Instance->DR = g_txBuf[g_txIdx];
            g_txIdx = (uint8_t)((g_txIdx + 1) % SPI_PKT_LEN);
        }
    }
    // Очистка возможного overrun (иначе RXNE залипает выкл. навсегда).
    if (__HAL_SPI_GET_FLAG(&hspi2, SPI_FLAG_OVR)) {
        volatile uint32_t tmp = hspi2.Instance->DR;
        tmp = hspi2.Instance->SR;
        (void)tmp;
    }
}

// Настроить SPI2 как 8-бит, mode-0, MSB-first slave с hardware NSS. GPIO:
// PB13/PB15 inputs, PB14 AF push-pull (MISO), PB12 NSS input pull-up.
static void spiSlaveBegin(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_SPI2_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};

    gpio.Pin  = GPIO_PIN_13 | GPIO_PIN_15;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin   = GPIO_PIN_14;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin   = GPIO_PIN_12;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    hspi2.Instance               = SPI2;
    hspi2.Init.Mode              = SPI_MODE_SLAVE;
    hspi2.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi2.Init.NSS               = SPI_NSS_HARD_INPUT;
    hspi2.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;  // игнорируется в slave
    HAL_SPI_Init(&hspi2);

    // Предзарядить DR первым байтом ответа (0xBB), чтобы Pi на самом первом
    // такте увидел корректный preamble. Дальнейшая укладка — в ISR.
    hspi2.Instance->DR = SPI_TX_SYNC0;

    __HAL_SPI_ENABLE_IT(&hspi2, SPI_IT_RXNE);
    __HAL_SPI_ENABLE(&hspi2);

    HAL_NVIC_SetPriority(SPI2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(SPI2_IRQn);
}

// Периодический замер потенциометра + EMA + оценка скорости вала.
// Вызывается каждую итерацию loop(); дросселируется до 1 кГц через micros().
static void samplePot(void) {
    static uint32_t lastUs = 0;
    const uint32_t now = micros();
    const uint32_t dtUs = now - lastUs;
    if (dtUs < POT_SAMPLE_US) return;
    lastUs = now;

    const uint16_t raw = (uint16_t)analogRead(potPin);

    // EMA-сглаживание raw-значения (снижаем джиттер ADC).
    static uint16_t emaRaw = 2048;
    emaRaw = (uint16_t)(((uint32_t)emaRaw * POT_EMA_B + (uint32_t)raw * POT_EMA_A)
                        / (POT_EMA_A + POT_EMA_B));

    // Скорость: (Δpot) / (Δt) в единицах ADC/сек. dt ~ 1000 µs → множитель
    // 1e6/dtUs. EMA сверху ещё раз сглаживает высокочастотный шум.
    const int32_t deltaU = (int32_t)emaRaw - (int32_t)g_potRaw;
    int32_t vps = (deltaU * 1000000) / (int32_t)dtUs;
    static int32_t emaVel = 0;
    emaVel = (emaVel * VEL_EMA_B + vps * VEL_EMA_A) / (VEL_EMA_A + VEL_EMA_B);

    if (emaVel >  32767) emaVel =  32767;
    if (emaVel < -32768) emaVel = -32768;

    g_potRaw = emaRaw;
    g_potVel = (int16_t)emaVel;
}

// Собрать 20-байтовый ответ для MISO. Вызывается в main loop; ISR сам
// раскладывает g_txBuf по одному байту в SPI DR.
static void buildTxFrame(void) {
    // Читаем разделяемые поля с блокировкой прерываний — коротко.
    noInterrupts();
    const uint16_t potRaw    = g_potRaw;
    const int16_t  potVel    = g_potVel;
    const int16_t  omegaOut  = g_omegaOut;
    const uint32_t good      = g_goodPackets;
    const uint32_t bad       = g_badPackets;
    const uint8_t  status    = g_statusOut;
    interrupts();

    uint8_t local[SPI_PKT_LEN];
    local[0] = SPI_TX_SYNC0;
    local[1] = SPI_TX_SYNC1;
    local[2] = (uint8_t)(potRaw & 0xFF);
    local[3] = (uint8_t)((potRaw >> 8) & 0xFF);
    local[4] = (uint8_t)((uint16_t)omegaOut & 0xFF);
    local[5] = (uint8_t)(((uint16_t)omegaOut >> 8) & 0xFF);
    local[6] = (uint8_t)((uint16_t)potVel & 0xFF);
    local[7] = (uint8_t)(((uint16_t)potVel >> 8) & 0xFF);
    local[8]  = (uint8_t)(good & 0xFF);
    local[9]  = (uint8_t)((good >> 8) & 0xFF);
    local[10] = (uint8_t)((good >> 16) & 0xFF);
    local[11] = (uint8_t)((good >> 24) & 0xFF);
    local[12] = (uint8_t)(bad & 0xFF);
    local[13] = (uint8_t)((bad >> 8) & 0xFF);
    local[14] = (uint8_t)((bad >> 16) & 0xFF);
    local[15] = (uint8_t)((bad >> 24) & 0xFF);
    local[16] = status;
    local[17] = 0;
    local[18] = 0;
    uint8_t cs = 0;
    for (uint8_t i = 0; i < SPI_PKT_LEN - 1; ++i) cs ^= local[i];
    local[SPI_PKT_LEN - 1] = cs;

    // Атомарная перезапись buf (внутри ISR читает по одному байту; частичное
    // рассогласование выправится на следующем кадре — Pi перечитает через
    // ~10 мс). Копируем «пакетно», прерывания коротко выключены.
    noInterrupts();
    for (uint8_t i = 0; i < SPI_PKT_LEN; ++i) g_txBuf[i] = local[i];
    interrupts();
}

void setup() {
    digitalWrite(enPin, HIGH);
    pinMode(enPin, OUTPUT);
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    // 12-битный ADC для потенциометра (по умолчанию у stm32duino 10-bit).
    analogReadResolution(12);
    pinMode(potPin, INPUT_ANALOG);

    // Инициализируем tx-буфер валидным пустым фреймом (пока main loop не
    // успел вызвать buildTxFrame): preamble+нули+корректный XOR.
    for (uint8_t i = 0; i < SPI_PKT_LEN; ++i) g_txBuf[i] = 0;
    g_txBuf[0] = SPI_TX_SYNC0;
    g_txBuf[1] = SPI_TX_SYNC1;
    uint8_t cs = 0;
    for (uint8_t i = 0; i < SPI_PKT_LEN - 1; ++i) cs ^= g_txBuf[i];
    g_txBuf[SPI_PKT_LEN - 1] = cs;

    Serial.begin(115200);
    Serial.println();
    Serial.println(F("[nucleo-f103rb] SPI drive controller w/ pot feedback (Step 3.2)"));
    Serial.println(F("  SPI2 slave: MOSI=PB15 MISO=PB14 SCK=PB13 NSS=PB12 (CN10)"));
    Serial.println(F("  pot input:  A0=PA0 (ADC1_IN0, 12-bit)"));
    Serial.println(F("  ожидаю 20-байтовые пакеты от Pi (spidev0.0) ..."));

    spiSlaveBegin();
}

// Non-blocking step pulse generator: toggles stepPin при заданной частоте.
static void runStepper(bool active, int16_t omega) {
    static bool     stepLevel   = false;
    static uint32_t lastEdgeUs  = 0;

    if (!active || omega == 0) {
        digitalWrite(enPin, HIGH);
        digitalWrite(stepPin, LOW);
        digitalWrite(LED_BUILTIN, LOW);
        stepLevel = false;
        return;
    }

    digitalWrite(dirPin, omega > 0 ? LOW : HIGH);
    digitalWrite(enPin, LOW);
    digitalWrite(LED_BUILTIN, HIGH);

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

// PD-закон с опциональной velocity-обратной связью от вала.
//   drive = clamp((err_n + Td·derr_n_cam - Kd_pot·pot_vel_n) · Kp, ±1)
//   omega = drive · max_omega
// Направление вклада pot_vel задаётся знаком Kd_pot (если ремень «в другую
// сторону» — просто ставим отрицательный Kd_pot в UI).
static int16_t computeOmega(bool manualActive, bool tracking,
                            int16_t manualOmega,
                            int16_t errN, int16_t derrN,
                            int16_t kpX100, int16_t maxOmega, int16_t tdX1000,
                            int16_t kdPotX1000, int16_t potVel) {
    if (manualActive) {
        return manualOmega;
    }
    if (!tracking) {
        return 0;
    }
    const float err     = (float)errN   / ERR_N_SCALE;
    const float derr    = (float)derrN  / DERR_N_SCALE;
    const float kp      = (float)kpX100 / KP_SCALE;
    const float td      = (float)tdX1000 / TD_SCALE;
    const float kdPot   = (float)kdPotX1000 / KD_POT_SCALE;
    const float potVelN = (float)potVel / POT_VEL_NORM;

    float drive = (err + td * derr - kdPot * potVelN) * kp;
    if (drive >  1.0f) drive =  1.0f;
    if (drive < -1.0f) drive = -1.0f;

    return (int16_t)lroundf(drive * (float)maxOmega);
}

void loop() {
    // Обновляем pot_raw и pot_vel из ADC (дросселировано до 1 кГц).
    samplePot();

    // Снимок ISR-полей.
    noInterrupts();
    const bool     manualActive = g_manualActive;
    const bool     tracking     = g_tracking;
    const int16_t  manualOmega  = g_manualOmega;
    const int16_t  errN         = g_errN;
    const int16_t  derrN        = g_derrN;
    const int16_t  kpX100       = g_kpX100;
    const int16_t  maxOmega     = g_maxOmega;
    const int16_t  tdX1000      = g_tdX1000;
    const int16_t  kdPotX1000   = g_kdPotX1000;
    const int16_t  potVel       = g_potVel;
    const uint16_t potRawLocal  = g_potRaw;
    interrupts();

    const int16_t omega  = computeOmega(manualActive, tracking, manualOmega,
                                        errN, derrN, kpX100, maxOmega, tdX1000,
                                        kdPotX1000, potVel);
    const bool    active = (manualActive || tracking) && (omega != 0);

    runStepper(active, omega);

    // Публикуем текущее состояние для tx-фрейма.
    uint8_t st = 0;
    if (manualActive) st |= 0x01;
    if (tracking)     st |= 0x02;
    if (active)       st |= 0x04;
    g_omegaOut  = omega;
    g_statusOut = st;

    // Обновляем tx-буфер (значения актуальны на момент следующего запроса).
    buildTxFrame();

    // Heartbeat в USART2.
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
        Serial.print(F(" pot="));
        Serial.print(potRawLocal);
        Serial.print(F(" pvel="));
        Serial.print(potVel);
        Serial.print(F("  good="));
        Serial.print(good);
        Serial.print(F(" bad="));
        Serial.println(bad);
    }
}
