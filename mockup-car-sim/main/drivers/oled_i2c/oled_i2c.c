/*
 * OLED I2C Driver - Source
 * Purpose: Implement SSD1306 OLED control and I2C transfer routines.
 * Notes: Include ESP-IDF headers or your own HAL wrappers as needed.
 */

#include <string.h>
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


/* Initialise the 128x64 SSD1306 OLED panel and clear the screen. */
void oled_init(void)
{
    if (s_oled_panel_handle != NULL)
    {
        return;
    }

    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = OLED_I2C_ADDRESS,
        .scl_speed_hz = I2C_FREQ_HZ,
        .control_phase_bytes = 1,
        .dc_bit_offset = 6,
        .lcd_cmd_bits = OLED_CMD_PARAM_BITS,
        .lcd_param_bits = OLED_CMD_PARAM_BITS,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c_v2(i2c_bus_handle,
                                                &io_config,
                                                &s_oled_io_handle));

    esp_lcd_panel_dev_config_t panel_config = {
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


void oled_draw_debug_screen(uint8_t set_speed_percent, bool hl_on, bool rev_on)
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

    /* Build dynamic strings for configuration values. */
    char sp_digits[4];
    uint8_t value = set_speed_percent;
    uint8_t hundreds = value / 100U;
    uint8_t tens = (value % 100U) / 10U;
    uint8_t ones = value % 10U;

    sp_digits[0] = (hundreds > 0U) ? (char)('0' + hundreds) : '0';
    sp_digits[1] = (char)('0' + tens);
    sp_digits[2] = (char)('0' + ones);
    sp_digits[3] = '\0';

    char line_sp[8];
    line_sp[0] = 'S';
    line_sp[1] = 'P';
    line_sp[2] = ':';
    line_sp[3] = sp_digits[0];
    line_sp[4] = sp_digits[1];
    line_sp[5] = sp_digits[2];
    line_sp[6] = '\0';

    const char *hl_state = hl_on ? "ON" : "OFF";
    char line_hl[8];
    line_hl[0] = 'H';
    line_hl[1] = 'L';
    line_hl[2] = ':';
    line_hl[3] = hl_state[0];
    line_hl[4] = hl_state[1];
    line_hl[5] = hl_state[2];
    line_hl[6] = '\0';

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

    /* Column titles */
    oled_draw_text((int)OLED_CONFIG_HEADER_POS_X, (int)OLED_CONFIG_HEADER_POS_Y, "CONFIG");
    oled_draw_text((int)OLED_DEBUG_HEADER_POS_X, (int)OLED_DEBUG_HEADER_POS_Y, "DEBUG");

    /* Divider lines */
    oled_draw_hline(0, (int)OLED_WIDTH_PIXELS - 1, (int)OLED_HEADER_LINE_Y);
    oled_draw_vline((int)OLED_SEPARATOR_POS_X, 0, (int)OLED_HEIGHT_PIXELS - 1);

    /* Left column configuration text */
    int config_y = (int)OLED_CONFIG_TEXT_START_Y;
    oled_draw_text((int)OLED_CONFIG_TEXT_START_X, config_y, line_sp);
    config_y += (int)(OLED_FONT_HEIGHT + 1U);
    oled_draw_text((int)OLED_CONFIG_TEXT_START_X, config_y, line_hl);
    config_y += (int)(OLED_FONT_HEIGHT + 1U);
    oled_draw_text((int)OLED_CONFIG_TEXT_START_X, config_y, line_rev);

    /* Right column debug text */
    int debug_y = (int)OLED_DEBUG_TEXT_START_Y;
    int debug_x = (int)OLED_DEBUG_TEXT_START_X;
    oled_draw_text(debug_x, debug_y, "EMR.BR:ACTIVE");
    debug_y += OLED_FONT_HEIGHT + 1;
    oled_draw_text(debug_x, debug_y, "ACT.SP:100");

    /* Square "bar graph" under debug text */
    const int squares_cols = 8;
    const int squares_rows = 5;
    const int square_size = 4;
    const int square_spacing = 1;

    int graph_base_y = debug_y + (int)OLED_GRAPH_BASE_OFFSET_Y;

    for (int row = 0; row < squares_rows; ++row)
    {
        int boxes_in_row = squares_cols - ((squares_rows - 1) - row);
        int row_y = graph_base_y +
                    (squares_rows - 1 - row) * (square_size + square_spacing);

        for (int col = 0; col < boxes_in_row; ++col)
        {
            int box_x = debug_x +
                        col * (square_size + square_spacing);
            oled_draw_rect_outline(box_x, row_y, square_size, square_size);
        }
    }

    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(s_oled_panel_handle, 0, 0, OLED_WIDTH_PIXELS, OLED_HEIGHT_PIXELS, s_oled_framebuffer));
}
