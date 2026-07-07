/******************************************************************************
 * motor_command.c — serial command dispatcher for STM32G431 FOC motor firmware.
 *
 * - Table-driven dispatch (cmd_entry_t[]), not if-ladders.
 * - SPSC ring buffer for RX (no __disable_irq). The board UART RX ISR calls
 *   Motor_Command_OnRxByte (push); foreground Motor_Command_Task pops bytes,
 *   assembles lines, and dispatches.
 * - Blocking TX via board_uart_tx (no TX DMA yet).
 * - Bounded string builders; no unbounded strcpy.
 * - Uses motor_t* API (motor_get_telemetry / motor_set_*) instead of direct MC.*
 * - Help / command-list printed via the table; add a command = add one table entry.
 *
 * All UART hardware access goes through the board port (board.h); this file is
 * HAL-free and does not name a specific UART peripheral.
 ******************************************************************************/

#include "motor_command.h"

#include <string.h>
#include <stdio.h>

#include "motor.h"
#include "motor_hal.h"
#include "motor_ui.h"             /* Motor_UI_ForceRefresh */
#include "motor_util.h"           /* spsc_ring_t, motor_util_append_* */
#include "board.h"                /* board_uart_tx / board_uart_rx_arm */

/* ===================== Constants ===================== */

#define CMD_RING_SIZE      256U       /* must be power of two */
#define CMD_LINE_MAX       64U        /* max chars per line (incl NUL) */
#define TX_BUF_SIZE        128U

/* ===================== Ring buffer + line assembly ===================== */

static uint8_t           cmd_ring_data[CMD_RING_SIZE];
static motor_ring_t      cmd_ring;
static char              cmd_line_buf[CMD_LINE_MAX];
static uint16_t          cmd_line_len = 0U;
static char              cmd_tx_buf[TX_BUF_SIZE];

/* ===================== TX (non-blocking preferred) ===================== */

static void cmd_send(const char *text)
{
    uint16_t len = (uint16_t)strnlen(text, (size_t)(TX_BUF_SIZE - 1U));
    if (len == 0U) return;

    /* Copy to TX buffer (bounded). */
    memcpy(cmd_tx_buf, text, len);
    cmd_tx_buf[len] = '\0';

    /* Blocking TX via the board UART (no TX DMA configured yet). */
    board_uart_tx((const uint8_t *)cmd_tx_buf, len);
}

/* ===================== Handlers ===================== */

static uint8_t cmd_run(motor_t *m, const char *arg)
{
    (void)arg;
    motor_enter_mode(m, MOTOR_SENSORUSE, SPEED_CURRENT_LOOP);
    motor_hal_enable_output(&g_motor, 1U);
    cmd_send("OK RUN SENS SPD\r\n");
    return 1U;
}

static uint8_t cmd_stop(motor_t *m, const char *arg)
{
    (void)arg;
    motor_hal_stop(&g_motor);
    cmd_send("OK STOP\r\n");
    return 1U;
}

static uint8_t cmd_status(motor_t *m, const char *arg)
{
    (void)arg;
    motor_telemetry_t tel;
    motor_get_telemetry(m, &tel);

    /* Build into tx buffer with bounded helpers. */
    int n = 0;
    n = snprintf(cmd_tx_buf, TX_BUF_SIZE,
        "OK STATUS state=%u mode=%u err=%u bus=%.1fV dir=%s pos=%.0f target=%.0f\r\n",
        (unsigned)tel.state, (unsigned)tel.run_mode,
        (unsigned)tel.error_code, tel.bus_real,
        tel.speed_dir >= 0 ? "CW" : "CCW",
        tel.position_angle_deg, tel.position_target_deg);
    if (n > 0) board_uart_tx((const uint8_t *)cmd_tx_buf, (uint16_t)n);
    return 1U;
}

/* Resolve a sensored sub-token (or empty/default) to a run mode + reply label.
 * Returns 1 on a recognized token, 0 otherwise. */
static uint8_t sens_sub_to_mode(const char *sub, uint8_t *mode, const char **label)
{
    if (sub[0] == '\0' || strcmp(sub, "SPD") == 0 || strcmp(sub, "SPEED") == 0)
    {
        *mode = SPEED_CURRENT_LOOP;     *label = "SENS SPD"; return 1U;
    }
    if (strcmp(sub, "CUR") == 0 || strcmp(sub, "CURRENT") == 0)
    {
        *mode = CURRENT_CLOSE_LOOP;     *label = "SENS CUR"; return 1U;
    }
    if (strcmp(sub, "POS") == 0 || strcmp(sub, "POSITION") == 0)
    {
        *mode = POS_SPEED_CURRENT_LOOP; *label = "SENS POS"; return 1U;
    }
    return 0U;
}

/* Parse `/mode <token> [sub]` and enter the resolved sensored run mode. */
static uint8_t cmd_mode(motor_t *m, const char *arg)
{
    /* Normalize to uppercase for table lookup (simple ASCII toupper). */
    char mode_buf[16];
    char sub_buf[16];
    uint8_t i = 0;
    uint8_t j = 0;

    while (arg[i] == ' ')
    {
        i++;
    }
    while (arg[i] != '\0' && arg[i] != ' ' && j < (uint8_t)(sizeof(mode_buf) - 1U))
    {
        char c = arg[i];
        if (c >= 'a' && c <= 'z') c = (char)(c - ('a' - 'A'));
        mode_buf[j] = c;
        i++;
        j++;
    }
    mode_buf[j] = '\0';

    while (arg[i] == ' ')
    {
        i++;
    }
    j = 0;
    while (arg[i] != '\0' && arg[i] != ' ' && j < (uint8_t)(sizeof(sub_buf) - 1U))
    {
        char c = arg[i];
        if (c >= 'a' && c <= 'z') c = (char)(c - ('a' - 'A'));
        sub_buf[j] = c;
        i++;
        j++;
    }
    sub_buf[j] = '\0';

    /* ---- Single resolution path: (mode token [+ sub token]) -> mode/label.
     * Sensorless modes were removed; only sensored modes remain.
     *   /mode cur  ==  /mode sens cur   ->  SENS CUR                              */
    uint8_t mode = SPEED_CURRENT_LOOP;
    const char *label = NULL;
    uint8_t resolved = 0U;

    if (mode_buf[0] == '\0')
    {
        cmd_send("ERR MODE. USAGE: /mode sens [spd|cur|pos] | cur|spd|pos\r\n");
        return 0U;
    }

    if (strcmp(mode_buf, "SENS") == 0 || strcmp(mode_buf, "SENSORED") == 0)
    {
        if (!sens_sub_to_mode(sub_buf, &mode, &label))
        {
            cmd_send("ERR MODE. USAGE: /mode sens [spd|cur|pos]\r\n");
            return 0U;
        }
        resolved = 1U;
    }
    else if (sens_sub_to_mode(mode_buf, &mode, &label))
    {
        /* Bare sensored alias: /mode cur|spd|pos (trailing sub-token ignored). */
        resolved = 1U;
    }

    if (!resolved)
    {
        cmd_send("ERR MODE. USAGE: /mode sens [spd|cur|pos] | cur|spd|pos\r\n");
        return 0U;
    }

    /* ---- Single apply + reply path. Power stage is enabled only once the state
     * machine actually reached MOTOR_SENSORUSE (else report it is still pending). */
    motor_enter_mode(m, MOTOR_SENSORUSE, mode);
    Motor_UI_ForceRefresh();

    if (motor_get_state(m) == MOTOR_SENSORUSE)
    {
        motor_hal_enable_output(&g_motor, 1U);
        cmd_send("OK MODE "); cmd_send(label); cmd_send("\r\n");
        return 1U;
    }

    cmd_send("OK MODE WAIT "); cmd_send(label); cmd_send("\r\n");
    return 1U;
}

static uint8_t cmd_set(motor_t *m, const char *arg)
{
    /* Parse first token */
    char tok[16];
    uint8_t i = 0;
    while (arg[i] != '\0' && arg[i] != ' ' && i < (uint8_t)(sizeof(tok) - 1U))
    {
        char c = arg[i];
        if (c >= 'a' && c <= 'z') c = (char)(c - ('a' - 'A'));
        tok[i] = c;
        i++;
    }
    tok[i] = '\0';
    const char *val_str = arg + i;
    while (*val_str == ' ') val_str++;

    if (strcmp(tok, "SPD") == 0 || strcmp(tok, "SPEED") == 0)
    {
        float fv;
        if (motor_util_parse_float(val_str, &fv) > 0)
        {
            motor_use_adc_target(&g_motor, 0U);
            motor_set_target_speed(&g_motor, fv);
            motor_enter_mode(m, MOTOR_SENSORUSE, SPEED_CURRENT_LOOP);
            motor_hal_enable_output(&g_motor, 1U);
            Motor_UI_ForceRefresh();
            cmd_send(motor_get_state(m) == MOTOR_SENSORUSE ? "OK SPD\r\n" : "OK SPD WAIT\r\n");
            return 1U;
        }
        cmd_send("ERR SPD. USAGE: /set spd <rpm>\r\n");
        return 0U;
    }

    if (strcmp(tok, "CUR") == 0 || strcmp(tok, "CURRENT") == 0)
    {
        float fv;
        if (motor_util_parse_float(val_str, &fv) > 0)
        {
            motor_enter_mode(m, MOTOR_SENSORUSE, CURRENT_CLOSE_LOOP);
            motor_hal_enable_output(&g_motor, 1U);
            motor_use_adc_target(&g_motor, 0U);
            motor_set_target_current(&g_motor, fv);
            Motor_UI_ForceRefresh();
            cmd_send(motor_get_state(m) == MOTOR_SENSORUSE ? "OK CUR\r\n" : "OK CUR WAIT\r\n");
            return 1U;
        }
        cmd_send("ERR CUR. USAGE: /set cur <amp>\r\n");
        return 0U;
    }

    if (strcmp(tok, "POS") == 0 || strcmp(tok, "POSITION") == 0)
    {
        int32_t iv;
        if (motor_util_parse_int(val_str, &iv) > 0)
        {
            motor_enter_mode(m, MOTOR_SENSORUSE, POS_SPEED_CURRENT_LOOP);
            motor_hal_enable_output(&g_motor, 1U);
            motor_use_adc_target(&g_motor, 0U);
            motor_set_target_position(&g_motor, iv);
            cmd_send("OK POS\r\n");
            return 1U;
        }
        cmd_send("ERR POS. USAGE: /set pos <cnt>\r\n");
        return 0U;
    }

    /* Angle command: degrees -> one-turn mechanical encoder target. */
    if (strcmp(tok, "ANG") == 0 || strcmp(tok, "ANGLE") == 0)
    {
        float fv;
        if (motor_util_parse_float(val_str, &fv) > 0)
        {
            motor_enter_mode(m, MOTOR_SENSORUSE, POS_SPEED_CURRENT_LOOP);
            motor_hal_enable_output(&g_motor, 1U);
            motor_use_adc_target(&g_motor, 0U);
            motor_set_target_angle(m, fv);
            cmd_send("OK ANG\r\n");
            return 1U;
        }
        cmd_send("ERR ANG. USAGE: /set ang <deg>\r\n");
        return 0U;
    }

    if (strcmp(tok, "DIR") == 0)
    {
        if (strcmp(val_str, "CW") == 0 || strcmp(val_str, "1") == 0)
        {
            motor_set_speed_dir(m, 1);
            cmd_send("OK DIR CW\r\n");
            return 1U;
        }
        if (strcmp(val_str, "CCW") == 0 || strcmp(val_str, "REV") == 0 || strcmp(val_str, "-1") == 0)
        {
            motor_set_speed_dir(m, -1);
            cmd_send("OK DIR CCW\r\n");
            return 1U;
        }
        cmd_send("ERR DIR. USAGE: /set dir cw|ccw\r\n");
        return 0U;
    }

    if (strcmp(tok, "POT") == 0 || strcmp(tok, "ADC") == 0)
    {
        if (strcmp(val_str, "ON") == 0 || strcmp(val_str, "1") == 0)
        {
            motor_use_adc_target(&g_motor, 1U);
            cmd_send("OK POT ON\r\n");
            return 1U;
        }
        if (strcmp(val_str, "OFF") == 0 || strcmp(val_str, "0") == 0)
        {
            motor_use_adc_target(&g_motor, 0U);
            cmd_send("OK POT OFF\r\n");
            return 1U;
        }
        cmd_send("ERR POT. USAGE: /set pot on|off\r\n");
        return 0U;
    }

    cmd_send("ERR SET TOKEN. USAGE: /set spd|cur|pos|ang|dir|pot <value>\r\n");
    return 0U;
}

static uint8_t cmd_clear(motor_t *m, const char *arg)
{
    (void)arg;
    motor_clear_error(m);
    Motor_UI_ForceRefresh();
    cmd_send("OK CLEAR\r\n");
    return 1U;
}

static uint8_t cmd_home(motor_t *m, const char *arg)
{
    (void)arg;
    motor_home_position(m);
    Motor_UI_ForceRefresh();
    cmd_send("OK HOME\r\n");
    return 1U;
}

static uint8_t cmd_help(motor_t *m, const char *arg);

/* ===================== Stream on/off ===================== */

static uint8_t stream_enabled = 0U;
static uint16_t stream_counter = 0U;

static uint8_t cmd_stream(motor_t *m, const char *arg)
{
    char buf[16];
    uint8_t i = 0;
    while (arg[i] != '\0' && arg[i] != ' ' && i < (uint8_t)(sizeof(buf) - 1U))
    {
        char c = arg[i];
        if (c >= 'a' && c <= 'z') c = (char)(c - ('a' - 'A'));
        buf[i] = c;
        i++;
    }
    buf[i] = '\0';

    if (strcmp(buf, "ON") == 0 || strcmp(buf, "1") == 0)
    {
        stream_enabled = 1U;
        stream_counter = 0U;
        cmd_send("OK STREAM ON\r\n");
        return 1U;
    }
    if (strcmp(buf, "OFF") == 0 || strcmp(buf, "0") == 0)
    {
        stream_enabled = 0U;
        cmd_send("OK STREAM OFF\r\n");
        return 1U;
    }
    cmd_send("ERR STREAM. USAGE: /stream on|off\r\n");
    return 0U;
}

/* Called from Motor_Command_Task every call. Sends telemetry every ~100ms. */
void cmd_stream_tick(motor_t *m)
{
    if (!stream_enabled) return;
    stream_counter++;
    if (stream_counter < 20) return;  /* ~100ms at 5ms task rate */
    stream_counter = 0;

    motor_telemetry_t tel;
    motor_get_telemetry(m, &tel);

    snprintf(cmd_tx_buf, TX_BUF_SIZE,
        "S %d %d %d %.1f %.2f %.0f %.0f %ld\r\n",
        (int)tel.state, (int)tel.run_mode, (int)tel.error_code,
        tel.bus_real, tel.iq, tel.speed_fb, tel.speed_set, (long)tel.encoder);
    uint16_t len = (uint16_t)strnlen(cmd_tx_buf, TX_BUF_SIZE - 1U);
    if (len > 0) board_uart_tx((const uint8_t *)cmd_tx_buf, len);
}

/* ===================== Dispatch table =====================
 * Add new commands by inserting a row here. The handler gets motor_t* + arg
 * string (everything after the command token, trimmed). Return non-zero for
 * "OK" (the handler already sent the OK reply), or 0 for error.
 */
static const cmd_entry_t cmd_table[] = {
    {"RUN",     cmd_run,    "Start sensored speed-current loop"},
    {"STOP",    cmd_stop,   "Stop PWM, disable output"},
    {"STATUS",  cmd_status, "Show run state / mode / error / direction"},
    {"MODE",    cmd_mode,   "sens [spd|cur|pos] | cur|spd|pos"},
    {"SET",     cmd_set,    "spd <rpm>|cur <A>|pos <cnt>|ang <deg>|dir cw|ccw|pot on|off"},
    {"STREAM",  cmd_stream, "on|off — periodic telemetry stream"},
    {"CLEAR",   cmd_clear,  "Clear motor error code"},
    {"CLR",     cmd_clear,  "Alias for CLEAR"},
    {"HOME",    cmd_home,   "Set current position as zero"},
    {"ZERO",    cmd_home,   "Alias for HOME"},
    {"HELP",    cmd_help,   "Show this help"},
    {"MENU",    cmd_help,   "Alias for HELP"},
    {"?",       cmd_help,   "Alias for HELP"},
};

/* ===================== Help handler ===================== */

static uint8_t cmd_help(motor_t *m, const char *arg)
{
    (void)m;
    cmd_send("\r\n==== MOTOR COMMANDS ====\r\n");
    for (uint8_t i = 0; i < (uint8_t)(sizeof(cmd_table)/sizeof(cmd_table[0])); i++)
    {
        /* Skip aliases in the compact view */
        if (strcmp(cmd_table[i].name, "CLR") == 0 ||
            strcmp(cmd_table[i].name, "ZERO") == 0 ||
            strcmp(cmd_table[i].name, "MENU") == 0 ||
            strcmp(cmd_table[i].name, "?") == 0)
        {
            continue;
        }
        cmd_send("/"); cmd_send(cmd_table[i].name);
        cmd_send("  ");
        cmd_send(cmd_table[i].help);
        cmd_send("\r\n");
    }
    cmd_send("=======================\r\n");
    return 1U;
}

/* ===================== Dispatch ===================== */

static void cmd_dispatch(motor_t *m, char *line)
{
    /* Strip leading spaces and '/' */
    char *p = line;
    while (*p == ' ' || *p == '/') p++;

    /* Uppercase */
    char *q = p;
    while (*q != '\0') { if (*q >= 'a' && *q <= 'z') *q = (char)(*q - ('a' - 'A')); q++; }

    /* Extract the first token */
    char cmd_name[16];
    uint8_t ci = 0;
    while (p[ci] != '\0' && p[ci] != ' ' && ci < (uint8_t)(sizeof(cmd_name) - 1U))
    {
        cmd_name[ci] = p[ci];
        ci++;
    }
    cmd_name[ci] = '\0';

    /* Advance arg to after the token */
    const char *arg = p + ci;
    while (*arg == ' ') arg++;

    /* Search the dispatch table */
    for (uint8_t i = 0; i < (uint8_t)(sizeof(cmd_table)/sizeof(cmd_table[0])); i++)
    {
        if (strcmp(cmd_name, cmd_table[i].name) == 0)
        {
            cmd_table[i].handler(m, arg);
            return;
        }
    }

    cmd_send("ERR UNKNOWN CMD. TYPE /help\r\n");
}

/* ===================== Public API ===================== */

void Motor_Command_Init(void)
{
    motor_ring_init(&cmd_ring, cmd_ring_data, CMD_RING_SIZE);

    board_uart_rx_arm();
    cmd_send("MOTOR READY. TYPE /help\r\n");
}

void Motor_Command_Task(void)
{
    /* Drain the SPSC ring and assemble a line. Process one line per call
     * (same pacing as the original code). */
    uint8_t byte;
    uint8_t have_line = 0U;

    while (motor_ring_pop(&cmd_ring, &byte) != 0)
    {
        if (byte == '\r' || byte == '\n')
        {
            if (cmd_line_len > 0U)
            {
                cmd_line_buf[cmd_line_len] = '\0';
                have_line = 1U;
                cmd_line_len = 0U;
                break;  /* one line per Task call */
            }
            continue;  /* skip leading newlines */
        }
        else if (byte == 0x08U || byte == 0x7FU)
        {
            if (cmd_line_len > 0U) cmd_line_len--;
        }
        else if (byte >= 32U && byte <= 126U)
        {
            if (cmd_line_len < (uint16_t)(CMD_LINE_MAX - 1U))
            {
                cmd_line_buf[cmd_line_len++] = (char)byte;
            }
        }
    }

    if (have_line != 0U)
    {
        cmd_dispatch(&g_motor, cmd_line_buf);
    }

    /* Stream tick (non-blocking, every Task call) */
    cmd_stream_tick(&g_motor);
}

/* ===================== ISR hook ===================== */

/* Called from the board UART RX-complete ISR (board_stm32g431.c) for each
 * received byte. The board layer owns the HAL handle and re-arms RX. */
void Motor_Command_OnRxByte(uint8_t byte)
{
    (void)motor_ring_push(&cmd_ring, byte);
}
