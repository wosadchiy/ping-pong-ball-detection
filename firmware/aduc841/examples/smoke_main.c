/* ============================================================================
 * main.c — ADuC841 SMOKE TEST
 *
 * Цель: убедиться, что весь конвейер собирается, заливается и стартует.
 * Что делает прошивка:
 *   1. Конфигурирует UART на 9600 baud (T1 mode 2, кварц 11.0592 MHz).
 *   2. После ресета шлёт строку  "ADuC841 ALIVE\r\n"  → видим в screen / miniterm.
 *   3. В цикле  echo  каждого принятого байта (что пришло — то ушло).
 *   4. Параллельно гонит  пилу 0..4095..0  на DAC0  → видим на осциллографе.
 *   5. Мигает  LED на P3.4  как индикатор "main loop alive".
 *
 * Как только этот smoke-test работает — переписываем `main.c` под реальную
 * задачу: парсер `dx,dy\n` из Python и запись dx в DAC0 для замера задержки.
 *
 * Кварц: жёстко считаем 11.0592 MHz. Если у вашей платы другой — поменяйте
 * `XTAL_HZ` ниже и пересчитайте TH1 (а лучше скажите мне, я поправлю).
 * ========================================================================= */

#include <mcs51/8052.h>
#include "aduc841_sfr.h"

#define XTAL_HZ        11059200UL
#define UART_BAUD      9600UL

/* ⚠ ADuC841 is a SINGLE-CYCLE 8052 — datasheet (page ~62):
 *     "the divide-by-12 prescaler is NOT present on the single-cycle core"
 *     "timers increment at the same rate as the core clock"
 *
 * So Timer-1 in mode 2 ticks at core_clk (NOT core_clk/12 like on a stock 8051).
 * Provided we set PLLCON = 0 (core_clk = OSC), the baud formula is simply:
 *
 *     baud = OSC / (32 × (256 − TH1))      when SMOD = 0
 *     TH1  = 256 − OSC / (32 × baud)
 *
 * For 11.0592 MHz / 9600 baud:
 *     256 − (11_059_200 / (32 × 9600)) = 256 − 36 = 220 = 0xDC
 *
 * Earlier this file had TH1 = 0xFD (the standard-8051 value), which on the
 * ADuC841 produced ×12 wrong baud (115_200 instead of 9600) — that's why
 * the previous firmware echoed pure garbage. */
#define TH1_RELOAD     ((unsigned char)(256U - (XTAL_HZ / (32UL * UART_BAUD))))

/* ─── UART ─────────────────────────────────────────────────────────────── */

static void uart_init(void) {
    /* CRITICAL #1 — force core to full speed.
     * Power-on default of PLLCON is 0x53 → CD bits = 3 → core_clk = OSC/8
     * (only 1.38 MHz for 11.0592 MHz xtal). The bootloader sets PLLCON=0
     * for itself, but a HW-reset without going through the bootloader leaves
     * us at /8. Force /1 here so our baud formula is correct in both cases. */
    PLLCON = 0x00;     /* CD=000 → core_clk = OSC = full xtal speed          */

    /* CRITICAL #2 — undo bootloader leftovers (AN-1074 Table 13). The loader
     * uses Timer-3 as its own baud generator; if T3CON.TR3 is left =1 it
     * keeps racing our Timer-1 and steals UART clock. */
    T3CON = 0x00;
    T3FD  = 0x00;
    PCON &= 0x7F;      /* SMOD = 0  (use the /32 prescaler, not /16)         */

    /* Standard 8052 UART init via Timer 1 mode 2 (auto-reload).
     * No CKCON on the ADuC841 — the single-cycle core has no /12 prescaler
     * and timers always tick at core_clk. */
    SCON = 0x50;       /* mode 1 (8-bit UART), receiver enabled              */
    TMOD = (TMOD & 0x0F) | 0x20;  /* Timer-1 mode 2 (8-bit auto-reload)      */
    TH1  = TH1_RELOAD;
    TL1  = TH1_RELOAD;
    TR1  = 1;          /* start Timer-1 — UART clock running                 */
    TI   = 1;          /* "previous tx complete" so first putchar doesn't hang */
}

static void uart_putc(unsigned char c) {
    while (!TI) { ; }   /* wait for previous TX complete                     */
    TI = 0;
    SBUF = c;
}

static void uart_puts(const char *s) {
    while (*s) {
        uart_putc((unsigned char)*s);
        s++;
    }
}

static unsigned char uart_rx_ready(void) {
    return RI;
}

static unsigned char uart_getc(void) {
    unsigned char c;
    while (!RI) { ; }
    c = SBUF;
    RI = 0;
    return c;
}

/* ─── DAC ──────────────────────────────────────────────────────────────── */

static void dac_init(void) {
    /* DACCON = 0b 0 1 1 1 1 1 1 1 = 0x7F
     *           |M|R|R|C|C|S|P|P|
     *           |O|N|N|L|L|Y|D|D|
     *           |D|G|G|R|R|N|1|0|
     *           |E|1|0|1|0|C| | |
     *
     * MODE=0 → 12-bit mode (we want full 0..4095 range)
     * RNG1=1 → DAC1 range 0..AVdd
     * RNG0=1 → DAC0 range 0..AVdd  (avoids needing ADC powered for VREF mode)
     * CLR1=1 → DAC1 normal output (not forced to 0V)
     * CLR0=1 → DAC0 normal output
     * SYNC=1 → asynchronous update (DAC latches on every DACxL write)
     * PD1 =1 → DAC1 powered ON
     * PD0 =1 → DAC0 powered ON  ← was 0 in earlier draft (DACs were OFF!)  */
    DACCON = 0x7F;
    DAC0_WRITE(0);
    DAC1_WRITE(0);
}

/* ─── LED (P3.4) ───────────────────────────────────────────────────────── */

__sbit __at(0xB4) LED;   /* P3.4 = bit 4 in port 3 (SFR 0xB0), so 0xB0+4 */

/* ─── main ─────────────────────────────────────────────────────────────── */

void main(void) {
    unsigned int  dac_val = 0;
    unsigned long heartbeat = 0;

    uart_init();
    dac_init();
    LED = 1;          /* off (active-low on most EVAL boards)                */

    uart_puts("\r\nADuC841 ALIVE\r\n");
    uart_puts("- echo enabled (type a char, see it back)\r\n");
    uart_puts("- DAC0 = ramp 0..4095\r\n");
    uart_puts("- heartbeat: '.' every ~2 s\r\n\r\n");

    for (;;) {
        /* echo any received byte */
        if (uart_rx_ready()) {
            unsigned char c = uart_getc();
            uart_putc(c);
        }

        /* DAC ramp: increment by 16 every iteration → 256 steps per period */
        DAC0_WRITE(dac_val);
        dac_val += 16;
        if (dac_val >= 4096) dac_val = 0;

        /* heartbeat — even if the monitor was opened AFTER the chip started,
         * the user will still see evidence of life within ~2 seconds. */
        heartbeat++;
        if ((heartbeat & 0x1FFFFUL) == 0) {
            uart_putc('.');
            LED = !LED;
        }
    }
}
