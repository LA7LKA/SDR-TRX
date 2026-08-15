#include "oled.h"
#include <string.h>

#define OLED_I2C_ADDR   (0x3C << 1)
#define OLED_WIDTH      128
#define OLED_PAGES      8
#define OLED_FB_SIZE    (OLED_WIDTH * OLED_PAGES)

static I2C_HandleTypeDef *oled_i2c;
static uint8_t framebuf[OLED_FB_SIZE];

/*
 * The framebuffer push is ~1 KB; blocking I2C at these speeds takes tens of
 * ms. On F746 (single core, HMI sharing the audio-DSP core) that was long
 * enough to stall past an audio block's deadline, so that port used
 * HAL_I2C_Master_Transmit_IT. On this H7 port HMI runs on CM4, a separate
 * core from CM7's audio pipeline - the deadline this was working around
 * doesn't exist here, and IT mode needs I2C1's NVIC interrupt wired up
 * (it wasn't - no HAL_NVIC_EnableIRQ(I2C1_EV_IRQn), no IRQHandler), which
 * left transfers stalling after their first few bytes: exactly the
 * "random dots, no image" symptom seen on hardware. Plain blocking
 * transmit avoids the whole class of bug.
 */
static uint8_t txbuf[1 + OLED_FB_SIZE];

/*
 * Standard SSD1306 128x64 init sequence - the same handful of commands
 * every SSD1306 driver (Adafruit, u8g2, ...) sends, transcribed here rather
 * than pulling in a library for one test screen.
 */
static const uint8_t init_cmds[] = {
    0xAE,             /* display off */
    0xD5, 0x80,       /* clock divide */
    0xA8, 0x3F,       /* multiplex ratio = 64-1 */
    0xD3, 0x00,       /* display offset = 0 */
    0x40,             /* start line = 0 */
    0x8D, 0x14,       /* charge pump on */
    0x20, 0x00,       /* horizontal addressing mode */
    0xA1,             /* segment remap */
    0xC8,             /* COM scan direction remapped */
    0xDA, 0x12,       /* COM pins config, 128x64 */
    0x81, 0x7F,       /* contrast */
    0xD9, 0xF1,       /* pre-charge */
    0xDB, 0x40,       /* VCOMH deselect level */
    0xA4,             /* resume RAM content display */
    0xA6,             /* normal (not inverted) */
    0xAF,             /* display on */
};

static void oled_cmd(uint8_t cmd)
{
    uint8_t buf[2] = { 0x00, cmd };
    HAL_I2C_Master_Transmit(oled_i2c, OLED_I2C_ADDR, buf, sizeof(buf), 100);
}

/*
 * 5x7 font, column-major (one byte per column, bit0 = top pixel). Only the
 * characters this project's HMI bring-up tests print - add more as needed.
 */
typedef struct { char c; uint8_t col[5]; } glyph_t;

static const glyph_t font[] = {
    { ' ', {0x00, 0x00, 0x00, 0x00, 0x00} },
    { 'A', {0x7C, 0x12, 0x11, 0x12, 0x7C} },
    { 'B', {0x7F, 0x49, 0x49, 0x49, 0x36} },
    { 'F', {0x7F, 0x09, 0x09, 0x09, 0x01} },
    { 'G', {0x3E, 0x41, 0x49, 0x49, 0x3A} },
    { 'I', {0x00, 0x41, 0x7F, 0x41, 0x00} },
    { 'J', {0x20, 0x40, 0x41, 0x41, 0x3F} },
    { 'K', {0x7F, 0x08, 0x14, 0x22, 0x41} },
    { 'M', {0x7F, 0x02, 0x04, 0x02, 0x7F} },
    { 'P', {0x7F, 0x09, 0x09, 0x09, 0x06} },
    { 'Q', {0x3E, 0x41, 0x51, 0x21, 0x5E} },
    { 'S', {0x46, 0x49, 0x49, 0x49, 0x31} },
    { 'V', {0x0F, 0x30, 0x40, 0x30, 0x0F} },
    { 'X', {0x41, 0x22, 0x1C, 0x22, 0x41} },
    { 'Y', {0x01, 0x02, 0x7C, 0x02, 0x01} },
    { 'Z', {0x61, 0x51, 0x49, 0x45, 0x43} },
    { '.', {0x00, 0x60, 0x60, 0x00, 0x00} },
    { '0', {0x3E, 0x51, 0x49, 0x45, 0x3E} },
    { '1', {0x00, 0x42, 0x7F, 0x40, 0x00} },
    { '2', {0x42, 0x61, 0x51, 0x49, 0x47} },
    { '3', {0x22, 0x41, 0x49, 0x49, 0x36} },
    { '4', {0x18, 0x14, 0x12, 0x7F, 0x10} },
    { '5', {0x27, 0x45, 0x45, 0x45, 0x39} },
    { '6', {0x3C, 0x4A, 0x49, 0x49, 0x30} },
    { '7', {0x01, 0x71, 0x09, 0x05, 0x03} },
    { '8', {0x36, 0x49, 0x49, 0x49, 0x36} },
    { '9', {0x06, 0x49, 0x49, 0x29, 0x1E} },
    { 'C', {0x3E, 0x41, 0x41, 0x41, 0x22} },
    { 'D', {0x7F, 0x41, 0x41, 0x41, 0x3E} },
    { 'E', {0x7F, 0x49, 0x49, 0x49, 0x41} },
    { 'H', {0x7F, 0x08, 0x08, 0x08, 0x7F} },
    { 'L', {0x7F, 0x40, 0x40, 0x40, 0x40} },
    { 'N', {0x7F, 0x02, 0x0C, 0x10, 0x7F} },
    { 'O', {0x3E, 0x41, 0x41, 0x41, 0x3E} },
    { 'R', {0x7F, 0x09, 0x19, 0x29, 0x46} },
    { 'T', {0x01, 0x01, 0x7F, 0x01, 0x01} },
    { 'U', {0x3F, 0x40, 0x40, 0x40, 0x3F} },
    { 'W', {0x7F, 0x20, 0x18, 0x20, 0x7F} },
    { ':', {0x00, 0x00, 0x12, 0x00, 0x00} },
    { '-', {0x08, 0x08, 0x08, 0x08, 0x08} },
    { '+', {0x08, 0x08, 0x3E, 0x08, 0x08} },
};
#define FONT_N (sizeof(font) / sizeof(font[0]))

static const uint8_t *glyph_for(char c)
{
    if (c >= 'a' && c <= 'z')
        c = (char)(c - 'a' + 'A');   /* font is uppercase-only, e.g. mode_name()'s "FreeDV 1600" */

    for (unsigned i = 0; i < FONT_N; i++)
        if (font[i].c == c)
            return font[i].col;
    return NULL;   /* unsupported character: skip it, don't guess */
}

void oled_init(I2C_HandleTypeDef *hi2c)
{
    oled_i2c = hi2c;
    for (unsigned i = 0; i < sizeof(init_cmds); i++)
        oled_cmd(init_cmds[i]);
    oled_clear();
}

void oled_clear(void)
{
    memset(framebuf, 0, sizeof(framebuf));
}

void oled_draw_text(uint8_t col, uint8_t page, const char *s)
{
    if (page >= OLED_PAGES)
        return;

    while (*s && col + 5 <= OLED_WIDTH)
    {
        const uint8_t *g = glyph_for(*s);
        if (g)
            memcpy(&framebuf[page * OLED_WIDTH + col], g, 5);
        col += 6;   /* 5 px glyph + 1 px spacing */
        s++;
    }
}

/* Doubles a glyph column's bottom/top nibble into a full 8-bit page column,
   each source pixel becoming a 2x2 block - the mechanism behind
   oled_draw_text_2x() below. Reuses the existing 5x7 font bitmaps rather
   than needing a second, hand-authored large font. */
static uint8_t expand_nibble(uint8_t g, int hi)
{
    uint8_t out = 0;
    for (int b = 0; b < 4; b++)
        if (g & (1u << (b + (hi ? 4 : 0))))
            out |= (uint8_t)(3u << (b * 2));
    return out;
}

/*
 * Same font as oled_draw_text(), rendered at 2x scale (10px wide, 16px/2
 * pages tall) for the one glanceable "big number" line the frequency
 * readout needs. page is the TOP of the two pages the glyph spans.
 */
void oled_draw_text_2x(uint8_t col, uint8_t page, const char *s)
{
    if (page + 1 >= OLED_PAGES)
        return;

    while (*s && col + 10 <= OLED_WIDTH)
    {
        const uint8_t *g = glyph_for(*s);
        if (g)
        {
            for (int i = 0; i < 5; i++)
            {
                uint8_t top = expand_nibble(g[i], 0);
                uint8_t bot = expand_nibble(g[i], 1);
                uint8_t dcol = (uint8_t)(col + i * 2);

                framebuf[page * OLED_WIDTH + dcol]           = top;
                framebuf[page * OLED_WIDTH + dcol + 1]       = top;
                framebuf[(page + 1) * OLED_WIDTH + dcol]     = bot;
                framebuf[(page + 1) * OLED_WIDTH + dcol + 1] = bot;
            }
        }
        col += 11;   /* 10 px glyph + 1 px spacing */
        s++;
    }
}

void oled_display(void)
{
    /* Reset the write window to the whole screen, then stream the
       framebuffer as one long data write (control byte 0x40). These command
       writes are 2-3 bytes each, negligible even blocking. */
    oled_cmd(0x21); oled_cmd(0);   oled_cmd(OLED_WIDTH - 1);   /* col range */
    oled_cmd(0x22); oled_cmd(0);   oled_cmd(OLED_PAGES - 1);   /* page range */

    txbuf[0] = 0x40;
    memcpy(&txbuf[1], framebuf, OLED_FB_SIZE);

    /* 1025 bytes at ~100 kHz Standard Mode I2C is ~91 ms on the wire alone
       - a 100 ms timeout left near-zero margin for clock stretching/HAL
       overhead, and cutting the transfer short partway through leaves the
       later pages showing stale/uncleared GDDRAM content ("white dots"
       below a correctly-updated top of the screen, since early pages get
       written before the timeout hits). 500 ms gives real headroom. */
    HAL_I2C_Master_Transmit(oled_i2c, OLED_I2C_ADDR, txbuf, sizeof(txbuf), 500);
}
