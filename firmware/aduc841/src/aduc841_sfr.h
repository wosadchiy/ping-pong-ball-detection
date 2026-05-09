/* ============================================================================
 * aduc841_sfr.h — ADuC841/842/843 specific SFR declarations for SDCC
 *
 * SDCC's stock <mcs51/8052.h> covers the standard 8052 core (SBUF, SCON,
 * TMOD, TH1, TR1, IE, etc.). This file adds only the ADuC841-specific
 * peripheral registers that we actually touch from C: DAC0/DAC1, ADC,
 * the second timer (T3) and a couple of misc bits.
 *
 * Addresses are taken from the ADuC841/842/843 datasheet, Table "Special
 * Function Register Locations" (page 16-17 of the datasheet, Rev. 0).
 *
 * Use under SDCC only — Keil C51 syntax (`sfr DACCON = 0xFD;`) won't compile.
 * ========================================================================= */

#ifndef ADUC841_SFR_H
#define ADUC841_SFR_H

/* ─── DAC ────────────────────────────────────────────────────────────────── */
/* 12-bit DACs, two channels. Buffered output, range = Vref or AVdd.
 * Addresses verified against ADuC841/842/843 datasheet, Rev. 0, page ~80.
 */
__sfr __at(0xFD) DACCON;        /* DAC control          (POR default = 0x04)  */
__sfr __at(0xF9) DAC0L;         /* DAC0 data low  (bits 7..0)                 */
__sfr __at(0xFA) DAC0H;         /* DAC0 data high (bits 11..8) in low nibble  */
__sfr __at(0xFB) DAC1L;         /* DAC1 data low                              */
__sfr __at(0xFC) DAC1H;         /* DAC1 data high                             */

/* DACCON bit layout — datasheet Table 16 (verified):
 *   bit 7  MODE   — 1 = 8-bit, 0 = 12-bit
 *   bit 6  RNG1   — DAC1 range (0 = 0..VREF, 1 = 0..AVdd)
 *   bit 5  RNG0   — DAC0 range (0 = 0..VREF, 1 = 0..AVdd)
 *   bit 4  CLR1   — DAC1 clear (1 = normal output, 0 = forced to 0 V)
 *   bit 3  CLR0   — DAC0 clear (1 = normal output, 0 = forced to 0 V)
 *   bit 2  SYNC   — update mode  (1 = update on DACxL write, 0 = sync via SYNC)
 *   bit 1  PD1    — DAC1 power   (1 = ON, 0 = OFF)
 *   bit 0  PD0    — DAC0 power   (1 = ON, 0 = OFF)
 *
 * ⚠ Earlier draft of this file had bit names INVERTED — that's why the DAC
 * was silently powered off. The correct value for "12-bit, AVdd range,
 * normal, async, both DACs ON" is 0b01111111 = 0x7F.
 */

/* ─── ADC (addresses verified against datasheet) ────────────────────────── */
__sfr __at(0xEF) ADCCON1;       /* ADC control 1 (timing, mode)   ← was 0xD8 */
__sfr __at(0xD8) ADCCON2;       /* ADC control 2 (channel select) ← was 0xAC */
__sfr __at(0xF5) ADCCON3;       /* ADC control 3 (busy/done)      ← was 0xEF */
__sfr __at(0xD9) ADCDATAL;      /* ADC result low                             */
__sfr __at(0xDA) ADCDATAH;      /* ADC result high (bits 11..8 + ch in upper) */

/* ─── T3 (dedicated baud-rate timer, used by the bootloader) ────────────── */
__sfr __at(0x9E) T3CON;         /* T3 control                                 */
__sfr __at(0x9D) T3FD;          /* T3 fractional divider                      */

/* ─── PLL / config ──────────────────────────────────────────────────────── */
/* IMPORTANT: ADuC841 power-on default = 0x53 → CD bits = 3 → core runs at
 * crystal/8. The bootloader sets PLLCON=0 for itself; if the chip is HW-reset
 * without going through the bootloader, the core falls back to /8 (1.38 MHz
 * for an 11.0592 MHz xtal), which kills our baud calculation. Always write
 * PLLCON = 0x00 in firmware init — that gives core_clk = OSC = full speed. */
__sfr __at(0xD7) PLLCON;        /* PLL/clock divider (POR default = 0x53)     */
__sfr __at(0xAF) CFG841;        /* extra config (POR default = 0x10 for 11.05)*/

/* NOTE: ADuC841 does NOT have a CKCON register (that's a Maxim DS80C320
 * thing). On a single-cycle 8052 the timers always tick at core_clk —
 * there is no /12 prescaler to enable/disable. Don't add CKCON back here. */

/* ─── Watchdog / power ──────────────────────────────────────────────────── */
__sfr __at(0xC0) WDCON;         /* watchdog control                           */
__sfr __at(0x87) PCON;           /* power control (re-declared from 8052.h)   */

/* ─── Timer 2 capture/reload (already in 8052.h, but bit 0xC8 SFR is T2CON  *
 *     and we sometimes want explicit names)                                  */

/* ─── Helpers ───────────────────────────────────────────────────────────── */

/* Write a 12-bit value (0..4095) to DAC0. The high nibble of `v` ends up in
 * DAC0H[3..0]; the low byte goes verbatim into DAC0L. Datasheet says DAC
 * latches on write to DAC0L, so write H first and then L. */
#define DAC0_WRITE(v) do {                                          \
    DAC0H = (unsigned char)(((unsigned int)(v) >> 8) & 0x0F);       \
    DAC0L = (unsigned char)((unsigned int)(v) & 0xFF);              \
} while (0)

#define DAC1_WRITE(v) do {                                          \
    DAC1H = (unsigned char)(((unsigned int)(v) >> 8) & 0x0F);       \
    DAC1L = (unsigned char)((unsigned int)(v) & 0xFF);              \
} while (0)

#endif /* ADUC841_SFR_H */
