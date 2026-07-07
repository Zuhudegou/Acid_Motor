/******************************************************************************
 * motor_ui.c — LCD UI for STM32G431 FOC motor firmware.
 * Dashboard home + menus (SENS).
 ******************************************************************************/

#include <string.h>
#include <stdio.h>

#include "motor_ui.h"

#include "lcd_drv.h"
#include "motor.h"
#include "motor_hal.h"      /* motor_hal_enable_output */
#include "motor_util.h"
#include "board.h"          /* board_key_*, board_millis */

#define REFRESH_MS          200U
#define LINE_H              LCD_FONT_H
#define TEXT_W              LCD_CHARS_PER_LINE
#define UI_ROWS             (LCD_H / LCD_FONT_H)
#define DIRTY_ALL           ((uint16_t)((1U << UI_ROWS) - 1U))

#define DEBOUNCE_MS         25U
#define LONG_MS             650U

#define EV_NEXT  0x01U
#define EV_PREV  0x02U
#define EV_OK    0x04U
#define EV_BACK  0x08U

#define COL_BLACK       LCD_BLACK
#define COL_WHITE       LCD_WHITE
#define COL_CYAN        LCD_CYAN
#define COL_RED         LCD_RED
#define COL_GREEN       LCD_GREEN
#define COL_BLUE        LCD_BLUE
#define COL_YELLOW      LCD_YELLOW
#define COL_MAGENTA     LCD_MAGENTA
#define COL_GRAY        LCD_GRAY
#define COL_DARKBLUE    LCD_DARKBLUE
#define COL_GRAYBLUE    LCD_GRAYBLUE
#define COL_LIGHTGREEN  LCD_LIGHTGREEN
#define COL_ROSE_PINK   LCD_ROSE_PINK

/* ===================== Button helper ===================== */

typedef uint8_t (*key_read_fn)(void);   /* returns 1 when the key is pressed */

typedef struct {
    key_read_fn read;
    uint8_t raw, stable, long_sent;
    uint32_t change_tick, press_tick;
} btn_t;

static btn_t btn_n, btn_e;

static void btn_init(btn_t *b, key_read_fn read)
{
    uint8_t p = read();
    b->read = read;
    b->raw = b->stable = p; b->long_sent = 0U;
    b->change_tick = b->press_tick = board_millis();
}

static uint8_t btn_poll(btn_t *b, uint32_t now, uint8_t se, uint8_t le)
{
    uint8_t r = b->read();
    uint8_t ev = 0U;
    if (r != b->raw) {
        b->raw = r; b->change_tick = now;
        if (r) {
            b->stable = 1U; b->press_tick = now; b->long_sent = 0U;
        } else {
            if (b->stable && !b->long_sent && (now - b->press_tick) >= DEBOUNCE_MS) ev |= se;
            b->stable = 0U; b->long_sent = 0U;
        }
    }
    if (b->stable && !b->long_sent && (now - b->press_tick) >= LONG_MS)
        { b->long_sent = 1U; ev |= le; }
    return ev;
}

/* ===================== State ===================== */

static uint32_t last_tick = 0U;
static uint16_t dirty = 0U;
static uint8_t  init_pending = 1U;
static uint8_t  menu_page = 0U;
static uint8_t  menu_cursor = 0U;

typedef enum { EDIT_NONE, EDIT_SPEED, EDIT_CURRENT, EDIT_POSITION } edit_t;
static edit_t  edit_mode = EDIT_NONE;
static float   ed_val_f;
static int     ed_val_i;

static motor_telemetry_t tel;
static float   ui_cmd_speed;
static float   ui_cmd_current;
static float   ui_cmd_angle;
static char     row_cache[UI_ROWS][TEXT_W + 1U];
static uint16_t row_fg_cache[UI_ROWS];
static uint16_t row_bg_cache[UI_ROWS];
static uint8_t  row_cache_valid[UI_ROWS];

#define R(n)  ((uint16_t)(1u << (n)))

/* ===================== Helpers ===================== */

static void ui_refresh_telemetry(void)
{
    motor_get_telemetry(&g_motor, &tel);
    ui_cmd_speed = motor_get_target_speed(&g_motor);
    ui_cmd_current = motor_get_target_current(&g_motor);
    ui_cmd_angle = motor_get_target_angle(&g_motor);
}

static void ui_invalidate_rows(void)
{
    memset(row_cache_valid, 0, sizeof(row_cache_valid));
}

static uint16_t ui_row_y(uint8_t row)
{
    return (uint16_t)(row * LINE_H);
}

static void ui_copy_at(char *dst, uint8_t pos, const char *src)
{
    while (src != NULL && *src != '\0' && pos < TEXT_W)
    {
        dst[pos++] = *src++;
    }
}

static const char *ui_state_label(void)
{
    switch (tel.state) {
        case MOTOR_STOP:       return "STOP";
        case MOTOR_ERROR:      return "ERR";
        case MOTOR_IDENTIFY:   return "ID";
        case MOTOR_SENSORUSE:  return "SENS";
        default:               return "ADC";
    }
}

static const char *ui_mode_label(void)
{
    switch (tel.run_mode) {
        case SPEED_CURRENT_LOOP:                 return "SPD";
        case CURRENT_CLOSE_LOOP:                 return "CUR";
        case POS_SPEED_CURRENT_LOOP:             return "POS";
        default:                                 return "?";
    }
}

static void draw_row_bg(uint8_t row, const char *s, uint16_t color, uint16_t bg)
{
    char line[TEXT_W + 1U];
    uint8_t i = 0U;
    uint16_t y;

    if (row >= UI_ROWS) return;
    if ((dirty & R(row)) == 0U) return;

    memset(line, ' ', TEXT_W);
    line[TEXT_W] = '\0';
    while (s[i] != '\0' && i < TEXT_W) {
        line[i] = s[i];
        i++;
    }

    if (row_cache_valid[row] != 0U &&
        row_fg_cache[row] == color &&
        row_bg_cache[row] == bg &&
        memcmp(row_cache[row], line, (size_t)(TEXT_W + 1U)) == 0)
    {
        dirty &= (uint16_t)~R(row);
        return;
    }

    y = ui_row_y(row);
    LCD_Fill(1U, y, LCD_W - 2U, (uint16_t)(y + LCD_FONT_H - 1U), bg);
    LCD_ShowString(2U, y, line, color, bg);
    memcpy(row_cache[row], line, (size_t)(TEXT_W + 1U));
    row_fg_cache[row] = color;
    row_bg_cache[row] = bg;
    row_cache_valid[row] = 1U;
    dirty &= (uint16_t)~R(row);
}

static void draw_row(uint8_t row, const char *s, uint16_t color)
{
    draw_row_bg(row, s, color, COL_BLACK);
}

/* ===================== Dashboard ===================== */

static void dash_header(void)
{
    uint16_t hc = COL_GREEN;
    char b[32];

    if (tel.state == MOTOR_ERROR) hc = COL_RED;
    else if (tel.state == MOTOR_STOP || tel.state == ADC_CALIB) hc = COL_YELLOW;

    if (tel.state == MOTOR_ERROR)
    {
        snprintf(b, sizeof(b), "FOC %-4s %-5s E%-2d",
                 ui_state_label(), ui_mode_label(), (int)tel.error_code);
    }
    else
    {
        snprintf(b, sizeof(b), "FOC %-4s %-5s OK",
                 ui_state_label(), ui_mode_label());
    }
    draw_row_bg(0U, b, hc, COL_DARKBLUE);
}

static void dash_speed_bus(void)
{
    char b[32];

    draw_row(1U, "Voltage", COL_GRAY);
    snprintf(b, sizeof(b), "Bus       %5.1f V", tel.bus_real);
    draw_row(2U, b, tel.state == MOTOR_ERROR ? COL_RED : COL_GREEN);

    draw_row(3U, "Speed", COL_GRAY);
    snprintf(b, sizeof(b), "Set     %+6ld rpm", (long)((int32_t)tel.speed_set));
    draw_row(4U, b, COL_WHITE);

    snprintf(b, sizeof(b), "Now     %+6ld rpm", (long)((int32_t)tel.speed_fb));
    draw_row(5U, b, COL_CYAN);
}

static void dash_encoder(void)
{
    char b[32];

    draw_row(6U, "Angle", COL_GRAY);
    snprintf(b, sizeof(b), "Now      %+6.1f deg", tel.position_angle_deg);
    draw_row(7U, b, COL_WHITE);

    snprintf(b, sizeof(b), "Target   %+6.1f deg", tel.position_target_deg);
    draw_row(8U, b, COL_CYAN);
}

static void dash_footer(void)
{
    draw_row_bg(9U, "K1 Menu  K2 OK  Hold Back", COL_GRAY, COL_BLACK);
}

static void draw_dash(void)
{
    ui_refresh_telemetry();
    dash_header();
    dash_speed_bus();
    dash_encoder();
    dash_footer();
}

/* ===================== Menu system ===================== */

enum { MENU_TOP = 1, MENU_SENS };

static const char *menu_title(uint8_t pg)
{
    switch (pg) {
        case MENU_TOP:   return "MENU";
        case MENU_SENS:  return "SENS";
        default:         return "";
    }
}

static uint8_t menu_items(uint8_t pg)
{
    switch (pg) {
        case MENU_TOP:   return 4;
        case MENU_SENS:  return 5; /* SPEED SET-SPD CUR POS BACK */
        default:         return 0;
    }
}

static const char *menu_label(uint8_t pg, uint8_t item)
{
    switch (pg) {
        case MENU_TOP:
            switch (item) {
                case 0: return "SENS";
                case 1: return "POT";
                case 2: return "ZERO";
                case 3: return "DASH";
                default: return "";
            }
        case MENU_SENS:
            switch (item) {
                case 0: return "SPEED";
                case 1: return "SET SPD";
                case 2: return "CURRENT";
                case 3: return "POSITION";
                case 4: return "BACK";
                default: return "";
            }
        default: return "";
    }
}

static void menu_hint(uint8_t pg, uint8_t item, char *dst, size_t len)
{
    float spd = tel.use_adc_target ? tel.speed_set : ui_cmd_speed;

    if (len == 0U) return;
    dst[0] = '\0';

    switch (pg) {
        case MENU_TOP:
            switch (item) {
                case 0:
                    snprintf(dst, len, "%s", tel.state == MOTOR_SENSORUSE ? "ON" : "");
                    break;
                case 1:
                    snprintf(dst, len, "%s", tel.use_adc_target ? "ON" : "OFF");
                    break;
                case 2:
                    snprintf(dst, len, "HOME");
                    break;
                case 3:
                    snprintf(dst, len, "BACK");
                    break;
                default:
                    break;
            }
            break;

        case MENU_SENS:
            switch (item) {
                case 0:
                    snprintf(dst, len, "%s", tel.use_adc_target ? "POT" : "CMD");
                    break;
                case 1:
                    snprintf(dst, len, "%+ldrpm", (long)((int32_t)spd));
                    break;
                case 2:
                    snprintf(dst, len, "%+3.1fA", ui_cmd_current);
                    break;
                case 3:
                    snprintf(dst, len, "%3lddeg", (long)((int32_t)ui_cmd_angle));
                    break;
                case 4:
                    snprintf(dst, len, "BACK");
                    break;
                default:
                    break;
            }
            break;

        default:
            break;
    }
}

static void menu_act(uint8_t pg, uint8_t item)
{
    switch (pg) {
        case MENU_TOP:
            switch (item) {
                case 0: menu_page = MENU_SENS; menu_cursor = 0; dirty = DIRTY_ALL; return;
                case 1:
                    motor_use_adc_target(&g_motor, 1U);
                    motor_enter_mode(&g_motor, MOTOR_SENSORUSE, SPEED_CURRENT_LOOP);
                    motor_hal_enable_output(&g_motor, 1U);
                    menu_page = 0; dirty = DIRTY_ALL; init_pending = 1U; return;
                case 2:
                    motor_home_position(&g_motor);
                    menu_page = 0; dirty = DIRTY_ALL; init_pending = 1U; return;
                case 3: menu_page = 0; dirty = DIRTY_ALL; init_pending = 1U; return;
            }
            break;
        case MENU_SENS:
            switch (item) {
                case 0:
                    motor_enter_mode(&g_motor, MOTOR_SENSORUSE, SPEED_CURRENT_LOOP);
                    motor_hal_enable_output(&g_motor, 1U);
                    menu_page = 0; dirty = DIRTY_ALL; init_pending = 1U;
                    return;
                case 1:
                    edit_mode = EDIT_SPEED;
                    ed_val_f = tel.use_adc_target ? tel.speed_set : motor_get_target_speed(&g_motor);
                    motor_use_adc_target(&g_motor, 0U);
                    motor_set_target_speed(&g_motor, ed_val_f);
                    motor_enter_mode(&g_motor, MOTOR_SENSORUSE, SPEED_CURRENT_LOOP);
                    motor_hal_enable_output(&g_motor, 1U);
                    dirty = DIRTY_ALL;
                    return;
                case 2:
                    motor_enter_mode(&g_motor, MOTOR_SENSORUSE, CURRENT_CLOSE_LOOP);
                    motor_hal_enable_output(&g_motor, 1U);
                    edit_mode = EDIT_CURRENT;
                    ed_val_f = motor_get_target_current(&g_motor);
                    motor_use_adc_target(&g_motor, 0U);
                    motor_set_target_current(&g_motor, ed_val_f);
                    dirty = DIRTY_ALL;
                    return;
                case 3:
                    motor_enter_mode(&g_motor, MOTOR_SENSORUSE, POS_SPEED_CURRENT_LOOP);
                    motor_hal_enable_output(&g_motor, 1U);
                    edit_mode = EDIT_POSITION;
                    ed_val_i = (int)motor_get_target_angle(&g_motor);
                    motor_use_adc_target(&g_motor, 0U);
                    motor_set_target_angle(&g_motor, (float)ed_val_i);
                    dirty = DIRTY_ALL;
                    return;
                case 4: menu_page = MENU_TOP; menu_cursor = 0; dirty = DIRTY_ALL; return;
            }
            break;
    }
    dirty = DIRTY_ALL;
}

static void draw_menu(uint8_t pg)
{
    char b[TEXT_W + 1U];
    uint8_t cnt = menu_items(pg);
    uint8_t first = 0U;
    uint8_t visible_rows = (UI_ROWS > 3U) ? (uint8_t)(UI_ROWS - 3U) : 1U;

    if (cnt == 0) return;
    if (menu_cursor >= cnt) menu_cursor = (uint8_t)(cnt - 1);
    if (cnt > visible_rows && menu_cursor >= visible_rows)
    {
        first = (uint8_t)(menu_cursor - visible_rows + 1U);
    }

    snprintf(b, sizeof(b), "Menu %-6s", menu_title(pg));
    draw_row_bg(0U, b, COL_GREEN, COL_DARKBLUE);

    for (uint8_t r = 0; r < visible_rows; r++) {
        uint8_t item = (uint8_t)(first + r);
        uint8_t selected = (item == menu_cursor && item < cnt) ? 1U : 0U;
        char hint[12];
        memset(b, ' ', TEXT_W); b[TEXT_W] = '\0';
        b[0] = selected ? '>' : ' ';
        const char *lbl = item < cnt ? menu_label(pg, item) : "";
        ui_copy_at(b, 2U, lbl);
        if (item < cnt)
        {
            menu_hint(pg, item, hint, sizeof(hint));
            ui_copy_at(b, 15U, hint);
        }
        draw_row_bg((uint8_t)(r + 1U), b,
                    selected ? COL_WHITE : COL_CYAN,
                    selected ? COL_GRAYBLUE : COL_BLACK);
    }
    draw_row((uint8_t)(UI_ROWS - 2U), "K1 Move   K2 Select", COL_GRAY);
    draw_row((uint8_t)(UI_ROWS - 1U), "Hold K1 -  Hold K2 Back", COL_GRAY);
}

/* ===================== Edit pages ===================== */

static void draw_edit(void)
{
    char b[TEXT_W + 1U];
    switch (edit_mode) {
        case EDIT_SPEED:
            draw_row_bg(0U, "SET SPEED", COL_GREEN, COL_DARKBLUE);
            snprintf(b, sizeof(b), "Target %+ld rpm", (long)((int32_t)ed_val_f));
            draw_row_bg(1U, b, COL_CYAN, COL_BLACK);
            snprintf(b, sizeof(b), "Fbk    %+ld rpm", (long)((int32_t)tel.speed_fb));
            draw_row(2U, b, COL_WHITE);
            snprintf(b, sizeof(b), "Applied%+ld rpm", (long)((int32_t)tel.speed_set));
            draw_row(3U, b, COL_GRAY);
            draw_row(4U, "K1 +100 rpm", COL_WHITE);
            draw_row(5U, "Hold K1 -100 rpm", COL_WHITE);
            break;
        case EDIT_CURRENT:
            draw_row_bg(0U, "SET CURRENT", COL_GREEN, COL_DARKBLUE);
            snprintf(b, sizeof(b), "Target %+3.1f A", ed_val_f);
            draw_row_bg(1U, b, COL_CYAN, COL_BLACK);
            snprintf(b, sizeof(b), "Iq     %+4.2f A", tel.iq);
            draw_row(2U, b, COL_WHITE);
            snprintf(b, sizeof(b), "Applied%+4.2f A", tel.current_set);
            draw_row(3U, b, COL_GRAY);
            draw_row(4U, "K1 +0.1 A", COL_WHITE);
            draw_row(5U, "Hold K1 -0.1 A", COL_WHITE);
            break;
        case EDIT_POSITION:
            draw_row_bg(0U, "SET ANGLE", COL_GREEN, COL_DARKBLUE);
            snprintf(b, sizeof(b), "Target %d deg", ed_val_i);
            draw_row_bg(1U, b, COL_CYAN, COL_BLACK);
            snprintf(b, sizeof(b), "Now    %+5.1f deg", tel.position_angle_deg);
            draw_row(2U, b, COL_WHITE);
            snprintf(b, sizeof(b), "Cnt%+6ld T%+6ld",
                     (long)tel.position_raw, (long)tel.position_target);
            draw_row(3U, b, COL_GRAY);
            draw_row(4U, "K1 +5 deg", COL_WHITE);
            draw_row(5U, "Hold K1 -5 deg", COL_WHITE);
            break;
        default: return;
    }
    draw_row(6U, "K2 Done", COL_YELLOW);
    draw_row(7U, "Hold K2 Exit", COL_GRAY);
    draw_row(8U, tel.use_adc_target ? "Live target Source POT" : "Live target Source CMD", COL_GRAY);
    draw_row(9U, "", COL_WHITE);
}

/* ===================== Input dispatch ===================== */

static void apply_edit_target(void)
{
    motor_use_adc_target(&g_motor, 0U);
    switch (edit_mode) {
        case EDIT_SPEED:
            motor_set_target_speed(&g_motor, ed_val_f);
            break;
        case EDIT_CURRENT:
            motor_set_target_current(&g_motor, ed_val_f);
            break;
        case EDIT_POSITION:
            motor_set_target_angle(&g_motor, (float)ed_val_i);
            break;
        default:
            break;
    }
}

static void do_nav(int32_t delta)
{
    if (edit_mode != EDIT_NONE) {
        switch (edit_mode) {
            case EDIT_SPEED:
                ed_val_f = motor_util_clampf(ed_val_f + (float)delta * 100.0f, -12000.0f, 12000.0f);
                break;
            case EDIT_CURRENT:
                ed_val_f = motor_util_clampf(ed_val_f + (float)delta * 0.1f, -3.0f, 3.0f);
                break;
            case EDIT_POSITION:
                ed_val_i += (int)(delta * 5);
                while (ed_val_i < 0) ed_val_i += 360;
                while (ed_val_i >= 360) ed_val_i -= 360;
                break;
            default: break;
        }
        apply_edit_target();
        dirty = DIRTY_ALL;
        return;
    }
    if (menu_page == 0) { menu_page = MENU_TOP; menu_cursor = 0; dirty = DIRTY_ALL; init_pending = 1U; return; }
    int16_t c = (int16_t)menu_cursor + (int16_t)delta;
    uint8_t cnt = menu_items(menu_page);
    if (c < 0) c = (int16_t)(cnt - 1);
    if (c >= (int16_t)cnt) c = 0;
    menu_cursor = (uint8_t)c;
    dirty = DIRTY_ALL;
}

static void do_ok(void)
{
    if (edit_mode != EDIT_NONE) {
        apply_edit_target();
        edit_mode = EDIT_NONE; dirty = DIRTY_ALL; init_pending = 1U;
        return;
    }
    if (menu_page >= MENU_TOP) menu_act(menu_page, menu_cursor);
}

static void do_back(void)
{
    if (edit_mode != EDIT_NONE) { edit_mode = EDIT_NONE; dirty = DIRTY_ALL; init_pending = 1U; return; }
    if (menu_page == 0) return;
    if (menu_page == MENU_TOP) { menu_page = 0; dirty = DIRTY_ALL; init_pending = 1U; return; }
    menu_page = MENU_TOP; menu_cursor = 0; dirty = DIRTY_ALL;
}

static void process_input(uint32_t now)
{
    uint8_t ev = 0;
    ev |= btn_poll(&btn_n, now, EV_NEXT, EV_PREV);
    ev |= btn_poll(&btn_e, now, EV_OK, EV_BACK);
    if (ev & EV_NEXT) do_nav(1);
    if (ev & EV_PREV) do_nav(-1);
    if (ev & EV_OK) do_ok();
    if (ev & EV_BACK) do_back();
}

/* ===================== Frame ===================== */

static void draw_frame(void)
{
    LCD_Fill(0U, 0U, LCD_W - 1U, LCD_H - 1U, COL_BLACK);
    LCD_DrawRect(0U, 0U, LCD_W - 1U, LCD_H - 1U, COL_DARKBLUE);
    LCD_DrawHLine(0U, LCD_W - 1U, LINE_H - 1U, COL_GRAYBLUE);
    ui_invalidate_rows();
}

static void draw_page(void)
{
    if (init_pending) { draw_frame(); init_pending = 0U; }
    if (edit_mode != EDIT_NONE) { ui_refresh_telemetry(); draw_edit(); }
    else if (menu_page == 0)    { draw_dash(); }
    else                        { ui_refresh_telemetry(); draw_menu(menu_page); }
    dirty = 0U;
}

/* ===================== Public API ===================== */

void Motor_UI_Init(void)
{
    LCD_Init();
    btn_init(&btn_n, board_key_next_pressed);
    btn_init(&btn_e, board_key_enter_pressed);
    init_pending = 1U; dirty = DIRTY_ALL; menu_page = 0; menu_cursor = 0; edit_mode = EDIT_NONE;
    last_tick = board_millis();
    Motor_UI_ForceRefresh();
}

void Motor_UI_ForceRefresh(void)
{
    init_pending = 1U; dirty = DIRTY_ALL;
    draw_page();
}

void Motor_UI_Task(void)
{
    uint32_t now = board_millis();
    process_input(now);
    if (init_pending || dirty) { draw_page(); last_tick = now; return; }
    if ((now - last_tick) < REFRESH_MS) return;
    last_tick = now;
    if (menu_page != 0 && edit_mode == EDIT_NONE) return;
    dirty = DIRTY_ALL;
    draw_page();
}
