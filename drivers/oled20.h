
#ifndef OLED20_H
#define OLED20_H

#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_PAGES 8

#define OLED_COLOR_WHITE 0
#define OLED_COLOR_BLACK 1

#define OLED_TOTAL_PIXELS OLED_WIDTH*OLED_HEIGHT

#include <stdint.h>
#include "nrf_lcd.h"

//OLED FUNCTIONS

#define OLED_PIXEL_WIDTH 128

void oled_write_c(char data);
void oled_write_d(char data);

/**
 * @brief Function to initialize the OLED.
 * @details The application developer should not call this function
 * directly. It is supplied as a callback to the nfr_gfx driver.
 */
ret_code_t oled_init();


void oled_reset(); 
void oled_home();
void oled_goto_line(int line);
void oled_goto_column(int column);
void oled_clear_line(int line);
void oled_pos(int row, int column);
void oled_print(char letter, int font_size); 
void oled_print_string(char string[], int font_size, int start_line, int start_column);
void oled_print_figure(char id, char start_line, int start_column);






/**
 * @brief Function to deinitialize the OLED driver.
 * @details The application developer should not call this function
 * directly. It is supplied as a callback to the nfr_gfx driver.
 */
void oled_uninit(void);

/**
 * @brief Function to draw single pixel into display buffer
 * @details The application developer should not call this function
 * directly. It is supplied as a callback to the nfr_gfx driver.
 *
 * @param[in] x			x-position
 * @param[in] y 		y-position
 * @param[in] color 	either clear or black for OLED
 */
void oled_draw_pixel(uint16_t x, uint16_t y, uint32_t color);

/**
 * @brief Function to draw rectangle into display buffer
 * @details The application developer should not call this function
 * directly. It is supplied as a callback to the nfr_gfx driver.
 *
 * @param[in] x			x-position
 * @param[in] y 		y-position
 * @param[in] width 	width of rectangle
 * @param[in] height 	height of rectangle
 * @param[in] color 	either clear or black for OLED
 */
void oled_draw_rectangle(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint32_t color);

/**
 * @brief Function to draw transfer display buffer to OLED.
 * @details The application developer should not call this function
 * directly. It is supplied as a callback to the nfr_gfx driver.
 */
void oled_display(void);

/**
 * @brief Function to clear the OLED contents.
 * @details The application developer should not call this function
 * directly. It is supplied as a callback to the nfr_gfx driver.
 */
void oled_clear(void);

/**
 * @brief Function to rotate canvas.
 * @details The application developer should not call this function
 * directly. It is supplied as a callback to the nfr_gfx driver.
 * This function has not been implemented.
 *
 * @param[in] rotation 	desired rotation
 */
void oled_dummy_rotation_set(nrf_lcd_rotation_t rotation);

/**
 * @brief Function to invert all drawing operations.
 * @details The application developer should not call this function
 * directly. It is supplied as a callback to the nfr_gfx driver.
 *
 * @param[in] invert 	to invert or not
 */
void oled_invert(bool invert);

#endif //OLED20_H