/*
 * OLED I2C Driver - Source
 * Purpose: Implement SSD1306 OLED control and I2C transfer routines.
 * Notes: Include ESP-IDF headers or your own HAL wrappers as needed.
 */

#include <string.h>
#include <stdio.h>
#include "project_config.h"
#include "oled_i2c.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"


static esp_lcd_panel_io_handle_t s_oled_io_handle = NULL;
static esp_lcd_panel_handle_t s_oled_panel_handle = NULL;
static uint8_t s_oled_framebuffer[OLED_WIDTH_PIXELS * OLED_HEIGHT_PIXELS / 8];

static void oled_set_pixel(uint8_t *fb, int x, int y)
{
    if ((x < 0) || (x >= (int)OLED_WIDTH_PIXELS) ||
        (y < 0) || (y >= (int)OLED_HEIGHT_PIXELS))
    {
        return;
    }

    int page = y / 8;
    int bit = y % 8;
    size_t index = (size_t)page * OLED_WIDTH_PIXELS + (size_t)x;
    fb[index] |= (uint8_t)(1U << bit);
}


static const uint8_t *oled_find_glyph(char c)
{
    size_t count = sizeof(s_oled_glyphs) / sizeof(s_oled_glyphs[0]);
    for (size_t i = 0; i < count; ++i)
    {
        if (s_oled_glyphs[i].character == c)
        {
            return s_oled_glyphs[i].rows;
        }
    }
    return NULL;
}


static void oled_draw_char(int x, int y, char c)
{
    const uint8_t *rows = oled_find_glyph(c);
    if (rows == NULL)
    {
        return;
    }

    for (int row = 0; row < OLED_FONT_HEIGHT; ++row)
    {
        uint8_t row_bits = rows[row];
        for (int col = 0; col < OLED_FONT_WIDTH; ++col)
        {
            int bit_index = (OLED_FONT_WIDTH - 1) - col;
            if (row_bits & (uint8_t)(1U << bit_index))
            {
                oled_set_pixel(s_oled_framebuffer, x + col, y + row);
            }
        }
    }
}


static void oled_draw_text(int x, int y, const char *text)
{
    int cursor_x = x;

    while ((text != NULL) && (*text != '\0'))
    {
        if (*text == '\n')
        {
            cursor_x = x;
            y += OLED_FONT_HEIGHT + 1;
        }
        else
        {
            oled_draw_char(cursor_x, y, *text);
            cursor_x += OLED_FONT_WIDTH + OLED_FONT_SPACING_X;
        }
        ++text;
    }
}


static void oled_draw_hline(int x_start, int x_end, int y)
{
    if (y < 0 || y >= (int)OLED_HEIGHT_PIXELS)
    {
        return;
    }
    if (x_start > x_end)
    {
        int tmp = x_start;
        x_start = x_end;
        x_end = tmp;
    }
    if (x_start < 0)
    {
        x_start = 0;
    }
    if (x_end >= (int)OLED_WIDTH_PIXELS)
    {
        x_end = (int)OLED_WIDTH_PIXELS - 1;
    }

    for (int x = x_start; x <= x_end; ++x)
    {
        oled_set_pixel(s_oled_framebuffer, x, y);
    }
}


static void oled_draw_vline(int x, int y_start, int y_end)
{
    if (x < 0 || x >= (int)OLED_WIDTH_PIXELS)
    {
        return;
    }
    if (y_start > y_end)
    {
        int tmp = y_start;
        y_start = y_end;
        y_end = tmp;
    }
    if (y_start < 0)
    {
        y_start = 0;
    }
    if (y_end >= (int)OLED_HEIGHT_PIXELS)
    {
        y_end = (int)OLED_HEIGHT_PIXELS - 1;
    }

    for (int y = y_start; y <= y_end; ++y)
    {
        oled_set_pixel(s_oled_framebuffer, x, y);
    }
}


static void oled_draw_rect_filled(int x, int y, int w, int h)
{
    if (w <= 0 || h <= 0)
    {
        return;
    }

    for (int yy = y; yy < y + h; ++yy)
    {
        for (int xx = x; xx < x + w; ++xx)
        {
            oled_set_pixel(s_oled_framebuffer, xx, yy);
        }
    }
}


static void oled_draw_rect_outline(int x, int y, int w, int h)
{
    if (w <= 0 || h <= 0)
    {
        return;
    }

    oled_draw_hline(x, x + w - 1, y);
    oled_draw_hline(x, x + w - 1, y + h - 1);
    oled_draw_vline(x, y, y + h - 1);
    oled_draw_vline(x + w - 1, y, y + h - 1);
}


/* Draw the triangular debug bar graph composed of outlined squares. */
static void oled_draw_debug_graph(int origin_x, int last_line_y, uint8_t bar_graph_level)
{
    int graph_base_y = last_line_y + (int)OLED_GRAPH_BASE_OFFSET_Y;
    int step = (int)OLED_GRAPH_SQUARE_SIZE + (int)OLED_GRAPH_SQUARE_SPACING;
    int total_boxes = (int)OLED_GRAPH_SQUARE_ROWS * (int)OLED_GRAPH_SQUARE_COLUMNS; /* rectangular grid */

    /* Clamp input percentage and compute how many boxes to fill. */
    if (bar_graph_level > 100U)
    {
        bar_graph_level = 100U;
    }
    int boxes_to_fill = (total_boxes * (int)bar_graph_level + 99) / 100; /* round up to show progress */
    int filled = 0;

    for (int col = 0; col < (int)OLED_GRAPH_SQUARE_COLUMNS; ++col)
    {
        int box_x = origin_x + col * step;
        for (int row = 0; row < (int)OLED_GRAPH_SQUARE_ROWS; ++row)
        {
            int row_y = graph_base_y + ((int)OLED_GRAPH_SQUARE_ROWS - 1 - row) * step; /* bottom to top */
            if (filled < boxes_to_fill)
            {
                oled_draw_rect_filled(box_x, row_y, (int)OLED_GRAPH_SQUARE_SIZE, (int)OLED_GRAPH_SQUARE_SIZE);
                ++filled;
            }
            oled_draw_rect_outline(box_x, row_y, (int)OLED_GRAPH_SQUARE_SIZE, (int)OLED_GRAPH_SQUARE_SIZE);
        }
    }
}


/* Initialise the 128x64 SSD1306 OLED panel and clear the screen. */
void oled_init(void)
{
    if (s_oled_panel_handle != NULL)
    {
        return;
    }

    esp_lcd_panel_io_i2c_config_t io_config = 
    {
        .dev_addr = OLED_I2C_ADDRESS,
        .scl_speed_hz = I2C_FREQ_HZ,
        .control_phase_bytes = 1,
        .dc_bit_offset = 6,
        .lcd_cmd_bits = OLED_CMD_PARAM_BITS,
        .lcd_param_bits = OLED_CMD_PARAM_BITS,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c_v2(i2c_bus_handle, &io_config, &s_oled_io_handle));

    esp_lcd_panel_dev_config_t panel_config = 
    {
        .bits_per_pixel = 1,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(s_oled_io_handle, &panel_config, &s_oled_panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_oled_panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_oled_panel_handle));

    /* Flip vertically so text is upright, keep horizontal order normal. */
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(s_oled_panel_handle, true, true)); /* This looks normal.*/
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_oled_panel_handle, true));

    memset(s_oled_framebuffer, 0x00, sizeof(s_oled_framebuffer));
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(s_oled_panel_handle, 0, 0, OLED_WIDTH_PIXELS, OLED_HEIGHT_PIXELS, s_oled_framebuffer));
}


void oled_draw_hello(void)
{
    if (s_oled_panel_handle == NULL)
    {
        return;
    }

    memset(s_oled_framebuffer, 0x00, sizeof(s_oled_framebuffer));

    const char *text = "HELLO";
    int text_len = 5;
    int text_width = text_len * OLED_FONT_WIDTH + (text_len - 1) * OLED_FONT_SPACING_X;

    int start_x = ((int)OLED_WIDTH_PIXELS - text_width) / 2;
    int start_y = ((int)OLED_HEIGHT_PIXELS - OLED_FONT_HEIGHT) / 2;

    oled_draw_text(start_x, start_y, text);

    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(s_oled_panel_handle, 0, 0, OLED_WIDTH_PIXELS, OLED_HEIGHT_PIXELS, s_oled_framebuffer));
}


void oled_draw_debug_screen(uint8_t set_speed_percent, bool hl_on, bool rev_on, bool emr_br_active, uint8_t bar_graph_level, uint8_t act_speed_percent, oled_option_select_t selected_option)
{
    if (s_oled_panel_handle == NULL)
    {
        return;
    }

    memset(s_oled_framebuffer, 0x00, sizeof(s_oled_framebuffer));

    if (set_speed_percent > 100U)
    {
        set_speed_percent = 100U;
    }

    /*------------ SET SPEED PERCENT STRINGIFY ------------ */

    /* Build dynamic strings for configuration values. */
    char sp_digits[4];
    uint8_t value = set_speed_percent;
    uint8_t hundreds = value / 100U;     /* Hundred digit : 0 or 1 */
    uint8_t tens = (value % 100U) / 10U; /* Tens digit */
    uint8_t ones = value % 10U;          /* Ones digit */

    /* Format speed as three characters, e.g. "005", "100" */
    sp_digits[0] = (hundreds > 0U) ? (char)('0' + hundreds) : '0';
    sp_digits[1] = (char)('0' + tens);
    sp_digits[2] = (char)('0' + ones);
    sp_digits[3] = '\0';
    /* Concatenate prefix with computed speed digits, e.g. SP:005 */
    char line_sp[8]; /* "SP:" (3) + 3 digits + NUL */
    snprintf(line_sp, sizeof(line_sp), "SP:%c%c%c", sp_digits[0], sp_digits[1], sp_digits[2]);
    

    /*------------ HEADLIGHTS STATUS STRINGIFY ------------ */

    const char *hl_state = hl_on ? "ON" : "OFF";
    char line_hl[8];
    line_hl[0] = 'H';
    line_hl[1] = 'L';
    line_hl[2] = ':';
    line_hl[3] = hl_state[0];
    line_hl[4] = hl_state[1];
    line_hl[5] = hl_state[2];
    line_hl[6] = '\0';

    /*------------ REVERSE STATUS STRINGIFY ------------ */

    const char *rev_state = rev_on ? "ON" : "OFF";
    char line_rev[8];
    line_rev[0] = 'R';
    line_rev[1] = 'E';
    line_rev[2] = 'V';
    line_rev[3] = ':';
    line_rev[4] = rev_state[0];
    line_rev[5] = rev_state[1];
    line_rev[6] = rev_state[2];
    line_rev[7] = '\0';

    /*------------ EMERGENCY BREAK STATUS STRINGIFY ------------ */
    const char *emr_br_state = emr_br_active ? "ACTIVE" : "OFF";
    char line_emr_br[14]; /* "EMR.BR:" (7) + "ACTIVE" (6) + NUL */
    snprintf(line_emr_br, sizeof(line_emr_br), "EMR.BR:%s", emr_br_state);

    /*------------ ACTUAL SPEED STRINGIFY ------------ */
    if (act_speed_percent > 100U)
    {
        act_speed_percent = 100U;
    }

    char act_sp_digits[4];
    value = act_speed_percent;
    hundreds = value / 100U;     /* Hundred digit : 0 or 1 */
    tens = (value % 100U) / 10U; /* Tens digit */
    ones = value % 10U;          /* Ones digit */

    act_sp_digits[0] = (hundreds > 0U) ? (char)('0' + hundreds) : '0';
    act_sp_digits[1] = (char)('0' + tens);
    act_sp_digits[2] = (char)('0' + ones);
    act_sp_digits[3] = '\0';

    char line_act_sp[11]; 
    snprintf(line_act_sp, sizeof(line_act_sp), "ACT.SP:%c%c%c", act_sp_digits[0], act_sp_digits[1], act_sp_digits[2]);

    /*------------ DRAW STATIC AND DYNAMIC CONTENT ------------ */

    /* Draw DEBUG and CONFIG titles for collumns */
    oled_draw_text((int)OLED_CONFIG_HEADER_POS_X, (int)OLED_CONFIG_HEADER_POS_Y, "CONFIG");
    oled_draw_text((int)OLED_DEBUG_HEADER_POS_X, (int)OLED_DEBUG_HEADER_POS_Y, "DEBUG");

    /* Collumn divider lines */
    oled_draw_hline(0, (int)OLED_WIDTH_PIXELS - 1, (int)OLED_HEADER_LINE_Y);
    oled_draw_vline((int)OLED_SEPARATOR_POS_X, 0, (int)OLED_HEIGHT_PIXELS - 1);

    /* Left CONFIG column text */
    int config_y = (int)OLED_CONFIG_TEXT_START_Y + OLED_GRAPH_GAP_Y + 5U;
    oled_draw_text((int)OLED_CONFIG_TEXT_START_X, config_y, line_sp);
    if(selected_option == OLED_CONFIG_OPTION_SPEED)
    {
        oled_draw_hline((int)OLED_CONFIG_TEXT_START_X, OLED_CONFIG_TEXT_START_X + OLED_TEXT_PIXEL_WIDTH("CONFIG"), (int)config_y + OLED_FONT_HEIGHT + 2); //underline
    }
    else
    {
        /* no underline */
    }

    config_y += (int)(OLED_FONT_HEIGHT + OLED_GRAPH_GAP_Y + 5U);
    oled_draw_text((int)OLED_CONFIG_TEXT_START_X, config_y, line_hl);
    if(selected_option == OLED_CONFIG_OPTION_HEADLIGHTS)
    {
        oled_draw_hline((int)OLED_CONFIG_TEXT_START_X, OLED_CONFIG_TEXT_START_X + OLED_TEXT_PIXEL_WIDTH("CONFIG"), (int)config_y + OLED_FONT_HEIGHT + 2); //underline
    }
    else
    {
        /* no underline */
    }

    config_y += (int)(OLED_FONT_HEIGHT + OLED_GRAPH_GAP_Y + 5U);
    oled_draw_text((int)OLED_CONFIG_TEXT_START_X, config_y, line_rev);
    if(selected_option == OLED_CONFIG_OPTION_REVERSE)
    {
        oled_draw_hline((int)OLED_CONFIG_TEXT_START_X, OLED_CONFIG_TEXT_START_X + OLED_TEXT_PIXEL_WIDTH("CONFIG"), (int)config_y + OLED_FONT_HEIGHT + 2); //underline
    }
    else
    {
        /* no underline */
    }

    /* Right DEBUG column text */
    int debug_y = (int)OLED_DEBUG_TEXT_START_Y + OLED_GRAPH_GAP_Y;
    int debug_x = (int)OLED_DEBUG_TEXT_START_X;
    oled_draw_text(debug_x, debug_y, line_emr_br);
    debug_y += (OLED_FONT_HEIGHT + OLED_GRAPH_GAP_Y);
    oled_draw_text(debug_x, debug_y, line_act_sp);

    /* Square "bar graph" under debug text */
    oled_draw_debug_graph(debug_x, debug_y + (int)OLED_GRAPH_GAP_Y, bar_graph_level);

    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(s_oled_panel_handle, 0, 0, OLED_WIDTH_PIXELS, OLED_HEIGHT_PIXELS, s_oled_framebuffer));
}
