#ifndef LCD_DRV_H
#define LCD_DRV_H

#include <stdint.h>
#include "board.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Orientation: 3 = landscape 160x80 (same as original USE_HORIZONTAL=3) */
#define LCD_ORIENTATION 3

#if LCD_ORIENTATION == 0 || LCD_ORIENTATION == 1
  #define LCD_W 80
  #define LCD_H 160
#else
  #define LCD_W 160
  #define LCD_H 80
#endif

/* ---------- Control lines (routed through the board port; pins live in
 *            board_config.h, not here) ---------- */
#define LCD_RES_Clr()  board_lcd_reset(0U)
#define LCD_RES_Set()  board_lcd_reset(1U)
#define LCD_DC_Clr()   board_lcd_dc(0U)
#define LCD_DC_Set()   board_lcd_dc(1U)
#define LCD_CS_Clr()   board_lcd_cs(0U)
#define LCD_CS_Set()   board_lcd_cs(1U)

/* ---------- RGB565 colors ---------- */
#define LCD_WHITE       0xFFFF
#define LCD_BLACK       0x0000
#define LCD_BLUE        0x001F
#define LCD_RED         0xF800
#define LCD_GREEN       0x07E0
#define LCD_CYAN        0x7FFF
#define LCD_YELLOW      0xFFE0
#define LCD_MAGENTA     0xF81F
#define LCD_GRAY        0x8430
#define LCD_DARKBLUE    0x01CF
#define LCD_LIGHTBLUE   0x7D7C
#define LCD_GRAYBLUE    0x5458
#define LCD_LIGHTGREEN  0x841F
#define LCD_LGRAY       0xC618
#define LCD_LGRAYBLUE   0xA651
#define LCD_ROSE_PINK   0xFCF3
#define LCD_PINK        0xFE19

/* ---------- Font metrics (5x7 monospace) ---------- */
#define LCD_FONT_W      6U    /* advance per char (5 px + 1 gap) */
#define LCD_FONT_H      8U    /* total cell height */
#define LCD_CHARS_PER_LINE  ((uint16_t)(LCD_W / LCD_FONT_W))   /* 26 at 160 */

/* ---------- Public API ---------- */
void LCD_Init(void);
void LCD_Fill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void LCD_DrawHLine(uint16_t x1, uint16_t x2, uint16_t y, uint16_t color);
void LCD_DrawVLine(uint16_t x, uint16_t y1, uint16_t y2, uint16_t color);
void LCD_DrawRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void LCD_ShowString(uint16_t x, uint16_t y, const char *s, uint16_t fc, uint16_t bc);

#ifdef __cplusplus
}
#endif

#endif /* LCD_DRV_H */
