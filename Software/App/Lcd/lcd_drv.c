
/******************************************************************************
 * lcd_drv.c — ST7735 160x80 RGB565 LCD driver.
 *
 * Replaces the original LcdAscii/lcd_drv.c:
 * - Font table (lcd_font.c) replaces the 47-case LCD_GlyphRow switch;
 *   supports full printable ASCII (32..126) including lowercase.
 * - Consistent INCLUSIVE coordinate convention for ALL public functions
 *   ((x1,y1)-(x2,y2) inclusive).
 * - Bounds checking: coordinates are clipped to the screen dimensions.
 * - Fast horizontal/vertical/rectangle fills using a single Address_Set +
 *   batched SPI transmit (no per-pixel CS toggling for borders).
 * - Dirty-line-ready: LCD_ShowString is the sole text primitive, same API
 *   as before so motor_ui.c compiles unchanged.
 * - All SPI / GPIO / delay access goes through the board port (board.h); this
 *   driver is HAL-free and names no specific SPI peripheral or pins.
 ******************************************************************************/

#include "lcd_drv.h"
#include "lcd_font.h"

/* ===================== Static helpers ===================== */

static void lcd_send_byte(uint8_t dat)
{
    LCD_CS_Clr();
    board_lcd_spi_write(&dat, 1U);
    LCD_CS_Set();
}

static void lcd_write_cmd(uint8_t cmd)
{
    LCD_DC_Clr();
    lcd_send_byte(cmd);
    LCD_DC_Set();
}

static void lcd_write_data8(uint8_t dat)
{
    lcd_send_byte(dat);
}

static void lcd_write_data16(uint16_t dat)
{
    lcd_send_byte((uint8_t)(dat >> 8));
    lcd_send_byte((uint8_t)(dat & 0xFFU));
}

/* Clip coordinates to the screen (inclusive bounds). */
static void lcd_clip(uint16_t *x1, uint16_t *y1, uint16_t *x2, uint16_t *y2)
{
    if (*x1 > *x2) { uint16_t t = *x1; *x1 = *x2; *x2 = t; }
    if (*y1 > *y2) { uint16_t t = *y1; *y1 = *y2; *y2 = t; }
    if (*x1 >= LCD_W) *x1 = LCD_W - 1;
    if (*x2 >= LCD_W) *x2 = LCD_W - 1;
    if (*y1 >= LCD_H) *y1 = LCD_H - 1;
    if (*y2 >= LCD_H) *y2 = LCD_H - 1;
}

/* Set the GRAM write window (inclusive coordinates). Orientation offset logic
 * matches the original USE_HORIZONTAL=3 branch (landscape 160x80). */
static void lcd_set_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
#if LCD_ORIENTATION == 0
    lcd_write_cmd(0x2A);
    lcd_write_data16((uint16_t)(x1 + 26U));
    lcd_write_data16((uint16_t)(x2 + 26U));
    lcd_write_cmd(0x2B);
    lcd_write_data16((uint16_t)(y1 + 1U));
    lcd_write_data16((uint16_t)(y2 + 1U));
#elif LCD_ORIENTATION == 1
    lcd_write_cmd(0x2A);
    lcd_write_data16((uint16_t)(x1 + 26U));
    lcd_write_data16((uint16_t)(x2 + 26U));
    lcd_write_cmd(0x2B);
    lcd_write_data16((uint16_t)(y1 + 1U));
    lcd_write_data16((uint16_t)(y2 + 1U));
#elif LCD_ORIENTATION == 2
    lcd_write_cmd(0x2A);
    lcd_write_data16((uint16_t)(x1 + 1U));
    lcd_write_data16((uint16_t)(x2 + 1U));
    lcd_write_cmd(0x2B);
    lcd_write_data16((uint16_t)(y1 + 26U));
    lcd_write_data16((uint16_t)(y2 + 26U));
#else /* orientation 3 — landscape 160x80 (current setting) */
    lcd_write_cmd(0x2A);
    lcd_write_data16(x1);
    lcd_write_data16(x2);
    lcd_write_cmd(0x2B);
    lcd_write_data16((uint16_t)(y1 + 24U));
    lcd_write_data16((uint16_t)(y2 + 24U));
#endif
    lcd_write_cmd(0x2C);
}

/* Transmit a solid color fill chunk. CS is held low. */
static void lcd_send_fill(uint32_t pixels, uint16_t color)
{
    uint8_t buf[128]; /* 64 pixels × 2 bytes */
    uint32_t chunk;

    LCD_CS_Clr();
    while (pixels > 0U)
    {
        chunk = (pixels > 64U) ? 64U : pixels;
        for (uint32_t i = 0U; i < chunk; i++)
        {
            buf[i * 2U]       = (uint8_t)(color >> 8);
            buf[i * 2U + 1U]  = (uint8_t)(color & 0xFFU);
        }
        board_lcd_spi_write(buf, (uint16_t)(chunk * 2U));
        pixels -= chunk;
    }
    LCD_CS_Set();
}

/* ===================== Public API ===================== */

void LCD_Init(void)
{
    LCD_RES_Clr();
    board_delay_ms(100U);
    LCD_RES_Set();
    board_delay_ms(200U);

    lcd_write_cmd(0x11);                   /* sleep out */
    board_delay_ms(120U);
    lcd_write_cmd(0x20);                   /* display inversion off (ST7735) */

    /* Frame rate / power / gamma (identical to original) */
    lcd_write_cmd(0xB1); lcd_write_data8(0x05); lcd_write_data8(0x3C); lcd_write_data8(0x3C);
    lcd_write_cmd(0xB2); lcd_write_data8(0x05); lcd_write_data8(0x3C); lcd_write_data8(0x3C);
    lcd_write_cmd(0xB3); lcd_write_data8(0x05); lcd_write_data8(0x3C); lcd_write_data8(0x3C);
                         lcd_write_data8(0x05); lcd_write_data8(0x3C); lcd_write_data8(0x3C);
    lcd_write_cmd(0xB4); lcd_write_data8(0x03);
    lcd_write_cmd(0xC0); lcd_write_data8(0xAB); lcd_write_data8(0x0B); lcd_write_data8(0x04);
    lcd_write_cmd(0xC1); lcd_write_data8(0xC5);
    lcd_write_cmd(0xC2); lcd_write_data8(0x0D); lcd_write_data8(0x00);
    lcd_write_cmd(0xC3); lcd_write_data8(0x8D); lcd_write_data8(0x6A);
    lcd_write_cmd(0xC4); lcd_write_data8(0x8D); lcd_write_data8(0xEE);
    lcd_write_cmd(0xC5); lcd_write_data8(0x0F);

    /* Positive gamma */
    lcd_write_cmd(0xE0);
    lcd_write_data8(0x07); lcd_write_data8(0x0E); lcd_write_data8(0x08); lcd_write_data8(0x07);
    lcd_write_data8(0x10); lcd_write_data8(0x07); lcd_write_data8(0x02); lcd_write_data8(0x07);
    lcd_write_data8(0x09); lcd_write_data8(0x0F); lcd_write_data8(0x25); lcd_write_data8(0x36);
    lcd_write_data8(0x00); lcd_write_data8(0x08); lcd_write_data8(0x04); lcd_write_data8(0x10);

    /* Negative gamma */
    lcd_write_cmd(0xE1);
    lcd_write_data8(0x0A); lcd_write_data8(0x0D); lcd_write_data8(0x08); lcd_write_data8(0x07);
    lcd_write_data8(0x0F); lcd_write_data8(0x07); lcd_write_data8(0x02); lcd_write_data8(0x07);
    lcd_write_data8(0x09); lcd_write_data8(0x0F); lcd_write_data8(0x25); lcd_write_data8(0x35);
    lcd_write_data8(0x00); lcd_write_data8(0x09); lcd_write_data8(0x04); lcd_write_data8(0x10);

    /* RGB565, MADCTL orientation */
    lcd_write_cmd(0x3A); lcd_write_data8(0x05);
    lcd_write_cmd(0x36);
#if LCD_ORIENTATION == 0
    lcd_write_data8(0x08);
#elif LCD_ORIENTATION == 1
    lcd_write_data8(0xC8);
#elif LCD_ORIENTATION == 2
    lcd_write_data8(0x78);
#else
    lcd_write_data8(0xA8);
#endif

    LCD_Fill(0U, 0U, LCD_W - 1U, LCD_H - 1U, LCD_BLACK);
    lcd_write_cmd(0x29);                    /* display on */
}

/* Fill a rectangle with a solid color. INCLUSIVE bounds. */
void LCD_Fill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    lcd_clip(&x1, &y1, &x2, &y2);
    uint32_t w = (uint32_t)(x2 - x1 + 1U);
    uint32_t h = (uint32_t)(y2 - y1 + 1U);
    lcd_set_window(x1, y1, x2, y2);
    lcd_send_fill(w * h, color);
}

/* Horizontal line (optimized batched fill). */
void LCD_DrawHLine(uint16_t x1, uint16_t x2, uint16_t y, uint16_t color)
{
    if (y >= LCD_H) return;
    if (x1 > x2) { uint16_t t = x1; x1 = x2; x2 = t; }
    if (x1 >= LCD_W) x1 = LCD_W - 1;
    if (x2 >= LCD_W) x2 = LCD_W - 1;
    lcd_set_window(x1, y, x2, y);
    lcd_send_fill((uint32_t)(x2 - x1 + 1U), color);
}

/* Vertical line (optimized batched fill). */
void LCD_DrawVLine(uint16_t x, uint16_t y1, uint16_t y2, uint16_t color)
{
    if (x >= LCD_W) return;
    if (y1 > y2) { uint16_t t = y1; y1 = y2; y2 = t; }
    if (y1 >= LCD_H) y1 = LCD_H - 1;
    if (y2 >= LCD_H) y2 = LCD_H - 1;
    lcd_set_window(x, y1, x, y2);
    lcd_send_fill((uint32_t)(y2 - y1 + 1U), color);
}

/* Hollow rectangle (4 lines — batched, not per-pixel). */
void LCD_DrawRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    LCD_DrawHLine(x1, x2, y1, color);
    LCD_DrawHLine(x1, x2, y2, color);
    LCD_DrawVLine(x1, y1, y2, color);
    LCD_DrawVLine(x2, y1, y2, color);
}

/* Draw a NUL-terminated string at (x, y). Every char gets a filled background
 * rectangle (same behavior as the original — bg fill erases stale content).
 * Bounds-clipped: a row whose cell would fall off the bottom is skipped, and
 * the string is truncated at the right edge instead of wrapping/garbling. */
void LCD_ShowString(uint16_t x, uint16_t y, const char *s, uint16_t fc, uint16_t bc)
{
    /* lcd_font_5x7 is declared in lcd_font.h (already included). */
    uint16_t cx = x;
    uint16_t cy = y;

    /* If the cell row does not fully fit vertically, there is nothing safe to
     * draw (avoids off-screen GRAM windows). */
    if ((uint32_t)cy + LCD_FONT_H > LCD_H)
    {
        return;
    }

    while (*s != '\0')
    {
        char ch = *s;

        /* Stop once the next cell would cross the right edge — truncate rather
         * than wrap or write past LCD_W (the original cause of the garbled
         * overflow). */
        if ((uint32_t)cx + LCD_FONT_W > LCD_W)
        {
            break;
        }

        /* Fetch glyph from the font table. Out-of-range chars render as '?'. */
        unsigned int glyph_idx;
        if (ch >= ' ' && ch <= '~')
        {
            glyph_idx = (unsigned int)(ch - ' ');
        }
        else
        {
            glyph_idx = (unsigned int)('?' - ' ');
        }
        const uint8_t *glyph = &lcd_font_5x7[glyph_idx][0];

        /* Set the write window for this 6×8 cell (5+1px gap, 7+1px gap). */
        lcd_set_window(cx, cy, (uint16_t)(cx + LCD_FONT_W - 1U), (uint16_t)(cy + LCD_FONT_H - 1U));

        /* Build the cell in a stack buffer and transmit in one shot. */
        uint8_t buf[96];  /* 6 * 8 * 2 = 96 bytes */
        uint16_t pos = 0U;
        for (uint8_t row = 0U; row < 8U; row++)
        {
            uint8_t bits = (row < 7U) ? glyph[row] : 0U;
            for (uint8_t col = 0U; col < 6U; col++)
            {
                uint16_t color = (col < 5U && (bits & (uint8_t)(1U << (4U - col))) != 0U) ? fc : bc;
                buf[pos++] = (uint8_t)(color >> 8);
                buf[pos++] = (uint8_t)(color & 0xFFU);
            }
        }

        LCD_CS_Clr();
        board_lcd_spi_write(buf, pos);
        LCD_CS_Set();

        cx = (uint16_t)(cx + LCD_FONT_W);
        s++;
    }
}
