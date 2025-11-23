#pragma once
/*
 * OLED I2C Driver - Header
 * Purpose: Declare interfaces for an I2C OLED display (e.g., SSD1306/SH1106).
 * Notes: Add function prototypes, types, and configuration here.
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_lcd_types.h"
#include "esp_lcd_panel_io.h"
#include "hal_i2c.h"
#include "esp_lcd_io_i2c.h"
#include "oled_config.h"

/*
 * Framebuffer layout used by oled_set_pixel() + esp_lcd_panel_draw_bitmap():
 *
 * - SSD1306 RAM is organised in "pages" of 8 vertical pixels.
 * - Our buffer size is: OLED_WIDTH_PIXELS * OLED_HEIGHT_PIXELS / 8 bytes.
 * - A pixel at (x, y) maps to:
 *       page  = y / 8          (0..7 for 64px high)
 *       bit   = y % 8          (0 is LSB)
 *       index = page * OLED_WIDTH_PIXELS + x
 *       buffer[index] |= (1u << bit);
 *
 *   Page 0 (top 8 rows), first few bytes (x = 0..3):
 *
 *        y=7  bit7  ┐
 *        ...        │ one byte per column
 *        y=1  bit1  │ (vertical 8‑pixel stripe)
 *        y=0  bit0  ┘
 *         ^
 *         |
 *   buffer[0]  -> column x=0, rows y=0..7
 *   buffer[1]  -> column x=1, rows y=0..7
 *   ...
 *
 * Passing this buffer as color_data to esp_lcd_panel_draw_bitmap() with
 *   (x_start=0, y_start=0, x_end=OLED_WIDTH_PIXELS, y_end=OLED_HEIGHT_PIXELS)
 * sends it 1:1 to the SSD1306 GRAM.
 */

/* SSD1306 oled screen init. */
void oled_init(void);

/* Draw a centered "HELLO" message (demo). */
void oled_draw_hello(void);

/*
 * Draw the CONFIG / DEBUG screen:
 * - Left column: set speed (0-100%), headlights ON/OFF, reverse ON/OFF.
 * - Right column: static debug labels plus a triangular outlined square graph placeholder.
 * - bar_graph_level fills the square graph (0-100%), act_sp shows current speed, emr_br_active flags emergency brake.
 */
void oled_draw_debug_screen(uint8_t set_speed_percent,
                            bool hl_on,
                            bool rev_on,
                            bool emr_br_active,
                            uint8_t bar_graph_level,
                            uint8_t act_speed_percent,
                            oled_option_select_t selected_option);
