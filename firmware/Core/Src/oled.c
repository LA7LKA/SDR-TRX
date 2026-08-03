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
 * ms, long enough to stall the main loop past an audio block's deadline and
 * come out as an audible glitch. Send it with HAL_I2C_Master_Transmit_IT
 * instead, and skip a display() call outright if the previous one hasn't
 * finished yet rather than blocking to wait for it.
 */
static volatile uint8_t oled_busy = 0;
static uint8_t txbuf[1 + OLED_FB_SIZE];

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == oled_i2c)
        oled_busy = 0;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == oled_i2c)
        oled_busy = 0;
}

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

void oled_display(void)
{
    /* A push is already in flight - drop this one rather than block waiting
       for it. Whatever called us again once it's done will show the latest
       framebuffer content anyway. */
    if (oled_busy)
        return;

    /* Reset the write window to the whole screen, then stream the
       framebuffer as one long data write (control byte 0x40). These command
       writes are 2-3 bytes each, negligible even blocking. */
    oled_cmd(0x21); oled_cmd(0);   oled_cmd(OLED_WIDTH - 1);   /* col range */
    oled_cmd(0x22); oled_cmd(0);   oled_cmd(OLED_PAGES - 1);   /* page range */

    txbuf[0] = 0x40;
    memcpy(&txbuf[1], framebuf, OLED_FB_SIZE);

    oled_busy = 1;
    if (HAL_I2C_Master_Transmit_IT(oled_i2c, OLED_I2C_ADDR, txbuf, sizeof(txbuf)) != HAL_OK)
        oled_busy = 0;
}
