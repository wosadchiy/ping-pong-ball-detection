/* ============================================================================
 * main.c — ADuC841 LATENCY MIRROR
 *
 * Назначение
 * ----------
 * Принимаем по UART строки вида  "dx,dy\n"  (signed десятичные пиксели от
 * Python-host'а) и копируем их на DAC0 / DAC1. Аналоговые выходы заводим
 * на осциллограф вместе со стробом тарелки и измеряем end-to-end задержку
 * конвейера: камера → OpenCV → EMA → pyserial → MCU → DAC.
 *
 * Протокол
 * --------
 *   "<dx>,<dy>\n"      ASCII, signed decimal, диапазон ±DX_MAX (по умолчанию
 *                      ±1024 px — с запасом относительно реального ±320 для VGA).
 *   '\r' игнорируется (CRLF из терминала тоже работает).
 *   Любая мусорная байта → состояние ST_ERR, всё до '\n' пропускается,
 *   DAC сохраняет последнее валидное значение.
 *
 * Маппинг
 * -------
 *   DAC code = clamp(val, ±DX_MAX) * 2048/DX_MAX + 2048   (12-bit, 0..4095)
 *   То есть нулевое смещение шарика → DAC=2048 → ~1.65 V на AVdd=3.3 V,
 *   полный левый край → 0 V, полный правый → AVdd.
 *
 * Индикация
 * ---------
 *   LED (P3.4) toggles на каждый успешно распарсенный пакет — глазами
 *   видно, что RX живой и пакеты доходят. Если LED не моргает,
 *   но что-то приходит — формат пакета сломан (см. ST_ERR).
 *
 * Тайминги
 * --------
 *   На 9600 baud один байт ≈ 1.04 ms. Пакет "dx,dy\n" типично 8-12 байт
 *   = ~10-12 ms на проводе. Это стабильный потолок ~80 пакетов/с —
 *   с большим запасом над частотой кадров камеры (30-60 Hz). Поднять
 *   до 115200 baud просто: поменять UART_BAUD ниже и пересчитать
 *   (TH1_RELOAD у нас параметризован).
 *
 * История smoke-теста (UART echo + DAC ramp + LED blink) сохранена в
 *   firmware/aduc841/examples/smoke_main.c
 * для отладки железа после, например, пересадки кварца или замены платы.
 * ========================================================================= */

#include <mcs51/8052.h>
#include "aduc841_sfr.h"

#define XTAL_HZ        11059200UL
#define UART_BAUD      115200UL

/* См. развёрнутый комментарий в examples/smoke_main.c — на single-cycle
 * 8052 (что и есть ADuC841) Timer-1 тикает на core_clk, формула:
 *     TH1 = 256 − OSC / (32 × baud)     при SMOD=0 и PLLCON=0.
 *   Для 11.0592 MHz / 9600 baud   →  256 − 36 = 220 = 0xDC
 *   Для 11.0592 MHz / 115200 baud →  256 −  3 = 253 = 0xFD
 *
 * 115200 выбран для борьбы с накоплением задержки в OS-буфере FTDI:
 * на 9600 baud пакеты ~10 байт уходили за 10 ms каждый, а Apple-драйвер
 * буферизовал TX до 64 КБ → задержка от Python до DAC накапливалась
 * до 5-8 секунд. На 115200 baud один пакет ~1 ms, накопление физически
 * невозможно — latency ≈ 1-2 ms + время кадра камеры. */
#define TH1_RELOAD     ((unsigned char)(256U - (XTAL_HZ / (32UL * UART_BAUD))))

/* Реальный диапазон ошибки трекера для VGA-камеры: ±320 px (половина
 * ширины кадра). Берём DX_MAX = 320, чтобы ±320 px разворачивались
 * на ВЕСЬ размах DAC (0..4095). Тогда:
 *   v =    0  →  DAC = 2048 (≈ AVdd/2)        — центр кадра
 *   v = +320  →  DAC = 4095 (= AVdd)          — правый край кадра
 *   v = −320  →  DAC =    0 (= 0 V)           — левый край кадра
 * Значения вне ±320 клампятся в map_to_dac(). */
#define DX_MAX         320
#define DAC_MID        2048

/* ─── UART ─────────────────────────────────────────────────────────────── */

static void uart_init(void) {
    /* CRITICAL #1 — full core speed. См. комментарий в smoke_main.c. */
    PLLCON = 0x00;
    /* CRITICAL #2 — снять Timer-3, оставленный включённым бутлоадером. */
    T3CON = 0x00;
    T3FD  = 0x00;
    PCON &= 0x7F;

    SCON = 0x50;
    TMOD = (TMOD & 0x0F) | 0x20;
    TH1  = TH1_RELOAD;
    TL1  = TH1_RELOAD;
    TR1  = 1;
    TI   = 1;
}

static void uart_putc(unsigned char c) {
    while (!TI) { ; }
    TI = 0;
    SBUF = c;
}

static void uart_puts(const char *s) {
    while (*s) {
        uart_putc((unsigned char)*s);
        s++;
    }
}

/* ─── DAC ──────────────────────────────────────────────────────────────── */

static void dac_init(void) {
    /* 12-bit, AVdd range, normal output, async update, both DACs ON. */
    DACCON = 0x7F;
    DAC0_WRITE(DAC_MID);
    DAC1_WRITE(DAC_MID);
}

/* ─── LED (P3.4) — RX-индикатор ────────────────────────────────────────── */

__sbit __at(0xB4) LED;

/* ─── Парсер dx,dy\n ───────────────────────────────────────────────────── */

enum parser_state { ST_DX, ST_DY, ST_ERR };

static enum parser_state pstate;
static int               acc;       /* безknаковый накопитель текущего поля */
static unsigned char     neg;       /* знак текущего поля (1=минус) */
static unsigned char     in_frac;   /* 1 = после '.', цифры игнорируются */
static int               dx_val;    /* зафиксированный dx после ',' */

/* Линейный маппинг знакового значения [-DX_MAX..+DX_MAX] в 12-bit DAC code. */
static unsigned int map_to_dac(int v) {
    long x;
    if (v >  DX_MAX) v =  (int)DX_MAX;
    if (v < -DX_MAX) v = -(int)DX_MAX;
    /* (v / DX_MAX) ∈ [-1..+1]  → DAC ∈ [0..4096], смещаем серединой 2048.
     * Делаем через long чтобы 16-bit умножение не переполнилось. */
    x = (long)v * 2048L / (long)DX_MAX + (long)DAC_MID;
    if (x < 0)    x = 0;
    if (x > 4095) x = 4095;
    return (unsigned int)x;
}

static void reset_field(void) {
    acc = 0;
    neg = 0;
    in_frac = 0;
}

static void parser_reset(void) {
    pstate = ST_DX;
    reset_field();
}

static void commit_packet(void) {
    int dy_val;
    unsigned int code_x, code_y;

    dy_val = neg ? -acc : acc;

    code_x = map_to_dac(dx_val);
    code_y = map_to_dac(dy_val);

    /* DAC1 first, DAC0 last — DAC0 latches на запись DAC0L и обновится
     * последним. Это стабилизирует фазу x-канала на сцоупе (триггеримся
     * по dx, ds увидим как y лежит относительно него). */
    DAC1_WRITE(code_y);
    DAC0_WRITE(code_x);

    LED = !LED;
}

static void feed(unsigned char c) {
    /* И \n, и \r считаем "конец пакета" — это позволяет интерактивно
     * проверять прошивку из `screen` (он шлёт только \r на Enter), а CRLF
     * из miniterm/Python обрабатывается корректно: на \r коммитим и сразу
     * парсер сбрасывается в ST_DX, на следующий \n находимся в ST_DX без
     * данных и просто делаем ещё один сброс — это безопасно. */
    if (pstate == ST_ERR) {
        if (c == '\n' || c == '\r') parser_reset();
        return;
    }

    if (c == '\n' || c == '\r') {
        if (pstate == ST_DY) commit_packet();
        parser_reset();
        return;
    }

    if (c == ',') {
        if (pstate == ST_DX) {
            dx_val = neg ? -acc : acc;
            pstate = ST_DY;
            reset_field();
        } else {
            pstate = ST_ERR;
        }
        return;
    }

    if (c == '-') {
        if (acc == 0 && !neg) {
            neg = 1;
        } else {
            pstate = ST_ERR;
        }
        return;
    }

    if (c == '+') {
        if (acc != 0 || neg) pstate = ST_ERR;
        return;
    }

    if (c >= '0' && c <= '9') {
        if (in_frac) return;     /* дробная часть — тихо проглатываем */
        acc = acc * 10 + (int)(c - '0');
        if (acc > 16000) {
            /* грубая защита от переполнения int (16-bit signed = ±32767).
             * Реальные значения детектора ≤ ±320, всё что выше клампит
             * map_to_dac(); валим только заведомый мусор. */
            pstate = ST_ERR;
        }
        return;
    }

    if (c == ' ' || c == '\t') return;   /* терпимо к пробелам */

    /* Десятичная точка — переходим в режим "ignore frac digits" до
     * следующего разделителя (',' или '\n'). Host шлёт уже целое, так
     * что эта ветка — страховка от форматов вида "320.5". */
    if (c == '.') {
        in_frac = 1;
        return;
    }

    pstate = ST_ERR;
}

/* ─── main ─────────────────────────────────────────────────────────────── */

void main(void) {
    uart_init();
    dac_init();
    parser_reset();
    LED = 1;

    uart_puts("\r\nADuC841 LATENCY MIRROR ready\r\n");
    uart_puts("- format: 'dx,dy\\n' signed decimal pixels (+/- 1024)\r\n");
    uart_puts("- DAC0=dx, DAC1=dy, midscale=center (~AVdd/2)\r\n");
    uart_puts("- LED toggles per valid packet\r\n\r\n");

    for (;;) {
        if (RI) {
            unsigned char c = SBUF;
            RI = 0;
            feed(c);
        }
    }
}
