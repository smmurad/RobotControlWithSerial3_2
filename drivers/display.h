/************************************************************************/
// File:            display.h                                           //
// Author:                                                              //
// Purpose:                                                             //
//                                                                      //
/************************************************************************/

#ifndef DISPLAY_H
#define DISPLAY_H

#include "nrf_lcd.h"
#include "nrf_gfx.h"
#include "oled20.h"
#include "FreeRTOS.h"
#include "queue.h"

#define OLED_MAX_STR_LEN 10

QueueHandle_t queue_display;


typedef struct {
	nrf_gfx_point_t text_position;
	const char* p_string;
	bool wrap;
} display_element_text_t;

typedef struct {
	nrf_gfx_point_t point;
} display_element_point_t;

typedef struct {
	nrf_gfx_line_t line;
} display_element_line_t;

typedef struct {
	nrf_gfx_circle_t circle;
	bool fill;
} display_element_circle_t;

typedef struct {
	nrf_gfx_rect_t rectangle;
	uint16_t thickness;
	bool fill;
        int color;
} display_element_rectangle_t;


typedef enum {
	DISPLAY_TEXT,
	DISPLAY_POINT,
	DISPLAY_LINE,
	DISPLAY_CIRCLE,
	DISPLAY_RECTANGLE,
	DISPLAY_LOG,
	DISPLAY_CLEAR
} DisplayOperationType;




typedef struct {
	DisplayOperationType operation;
	union DisplayElement {
		display_element_text_t text;
		display_element_point_t point;
		display_element_line_t line;
		display_element_circle_t circle;
		display_element_rectangle_t rectangle;
		char log_string[OLED_MAX_STR_LEN];
	} element;
} display_operation_t;



/**
 * @brief Function to initialize the display drivers.
 */
/**
 * @brief Function to initialize the display drivers.
 */
void display_init(void);

/**
 * @brief Function to draw text on display.
 *
 * @param[in] x			x-position
 * @param[in] y			y-position
 * @param[in] p_string	text-string to draw
*/
void display_text(int x, int y, const char* p_string);

/**
*@Function to clear and display text in a specific line
*@parm[in]    line                 line number
@parm[in]     p_string             text-string to draw
*/
void display_text_on_line(int line, char *p_string);

/**
 * @brief Function to draw point on display.
 *
 * @param[in] x			x-position
 * @param[in] y			y-position
*/
void display_point(int x, int y);

/**
 * @brief Function to draw line on display.
 *
 * @param[in] x_start	x start-position
 * @param[in] y_start	y start-position
 * @param[in] x_end		x end-position
 * @param[in] y_end		y end-position
 * @param[in] thickness	thickness in pixels of line to draw
*/
void display_line(int x_start, int y_start, int x_end, int y_end, int thickness);


/**
 * @brief Function to draw circle on display.
 *
 * @param[in] x			x center-position
 * @param[in] y			y center-position
 * @param[in] r			radius of circle
 * @param[in] fill		draw circle filled or not
*/
void display_circle(int x, int y, int r, bool fill);

/**
 * @brief Function to draw rectangle on display.
 *
 * @param[in] x			x position
 * @param[in] y			y position
 * @param[in] width		width of rectangle
 * @param[in] height	height of rectangle
 * @param[in] thickness	line-thickness of rectangle
 * @param[in] fill		draw rectangle filled or not
 * @param[in] color             color of rectangle
*/
void display_rectangle(int x, int y, int width, int height, int thickness, bool fill, int color);

/**
 * @brief Function to handle multiple lines of text in logging applicatoin.
 * @details NOT IMPLEMENTED! The purpose is to have printf-alike functinality
 * with the display, such that the developer does not have to worry about new-
 * lines, positioning and scrolling.
 *
 * @param[in] p_string	string to draw on display
*/
void display_log(const char* p_string);

/**
 * @brief Function to clear the display (to blank).
*/
void display_clear(void);

#endif //DISPLAY_H