#include "display.h"

#include "nrf_spi_mngr.h"
#include "nrf_lcd.h"
#include "nrf_gfx.h"
#include "oled20.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "microsd.h"
#include "globals.h"
#include "nrf_log.h"
#include "i2c.h"

static lcd_cb_t m_lcd_cb =
{
    .state    = NRFX_DRV_STATE_UNINITIALIZED,
    .height   = OLED_HEIGHT - 1,
    .width    = OLED_WIDTH - 1,
    .rotation = NRF_LCD_ROTATE_0
}; 

const nrf_lcd_t m_nrf_lcd =
{
    .lcd_init   = oled_init,
    .lcd_uninit = oled_uninit,
    .lcd_pixel_draw = oled_draw_pixel,
    .lcd_rect_draw = oled_draw_rectangle,
    .lcd_display = oled_display,
    .lcd_rotation_set = NULL, //oled_dummy_rotation_set,
    .lcd_display_invert = oled_invert,
    .p_lcd_cb = &m_lcd_cb
};

static inline void enqueue(display_operation_t* display_operation) {
	xQueueSendToBack(queue_display, display_operation, portMAX_DELAY);
}

void display_init(void) {
	//APP_ERROR_CHECK(nrf_gfx_init(&m_nrf_lcd));
	//nrf_gfx_screen_fill(&m_nrf_lcd, 0);
	//nrf_gfx_display(&m_nrf_lcd);

    i2c_init();
    oled_init();

    oled_display();
}

void display_text(int x, int y, const char* p_string) {
	display_operation_t display_operation;
	display_operation.operation = DISPLAY_TEXT;
	
	display_element_text_t display_element;
	display_element.p_string = p_string;
	display_element.text_position.x = x;
	display_element.text_position.y = y;
	
	display_operation.element.text = display_element;
	
	enqueue(&display_operation);
}

void display_text_on_line(int line, char *p_string){
       if(line>5 || line <1){
          line = 1;
       }

        display_rectangle(5,12*(line-1),127,12,1,1,0);//clears 1 pixel to much for some chars (_), change to font with 12pixel max prefered
        display_text(5,12*(line-1),p_string);
}

void display_point(int x, int y) {
	display_operation_t display_operation;
	display_operation.operation = DISPLAY_POINT;
	
	display_element_point_t display_element;
	display_element.point.x = x;
	display_element.point.y = y;
	
	display_operation.element.point = display_element;
	
	enqueue(&display_operation);
}

void display_line(int x_start, int y_start, int x_end, int y_end, int thickness) {
	display_operation_t display_operation;
	display_operation.operation = DISPLAY_LINE;
	
	display_element_line_t display_element;
	display_element.line.x_start = x_start;
	display_element.line.y_start = y_start;
	display_element.line.x_end = x_end;
	display_element.line.y_end = y_end;
	display_element.line.thickness = thickness;
	
	display_operation.element.line = display_element;
	
	enqueue(&display_operation);
}

void display_circle(int x, int y, int r, bool fill) {
	display_operation_t display_operation;
	display_operation.operation = DISPLAY_CIRCLE;
	
	display_element_circle_t display_element;
	display_element.circle.x = x;
	display_element.circle.y = y;
	display_element.circle.r = r;
	display_element.fill = fill;
	
	display_operation.element.circle = display_element;
	
	enqueue(&display_operation);
}

void display_rectangle(int x, int y, int width, int height, int thickness, bool fill, int color) {
	display_operation_t display_operation;
	display_operation.operation = DISPLAY_RECTANGLE;
	
	display_element_rectangle_t display_element;
	display_element.rectangle.x = x;
	display_element.rectangle.y = y;
	display_element.rectangle.width = width;
	display_element.rectangle.height = height;
	display_element.thickness = thickness;	
	display_element.fill = fill;
        display_element.color = color;

	display_operation.element.rectangle = display_element;
	
	enqueue(&display_operation);
}
	
void display_clear(void) {
	display_operation_t display_operation;
	display_operation.operation = DISPLAY_CLEAR;
	
	enqueue(&display_operation);	
}