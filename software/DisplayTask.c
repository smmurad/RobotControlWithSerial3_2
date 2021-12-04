#include "DisplayTask.h"

#include "nrf_lcd.h"
#include "nrf_gfx.h"
#include "FreeRTOS.h"
#include "queue.h"

#include "nrf_spi_mngr.h"
#include "nrf_lcd.h"
#include "nrf_gfx.h"
#include "oled20.h" //oled20
#include "display.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "microsd.h"
#include "globals.h"
#include "nrf_log.h"

QueueHandle_t queue_display;

void display_task(void *arg) {
    NRF_LOG_INFO("Display task: initializing");
    display_init();

    display_operation_t display_operation;

    NRF_LOG_INFO("Display task: init complete");
    for (;;) {
        xQueueReceive(queue_display, &display_operation, portMAX_DELAY);
        xSemaphoreTake(mutex_i2c, portMAX_DELAY);
		
        switch (display_operation.operation) {
			
        case DISPLAY_TEXT:
            APP_ERROR_CHECK(nrf_gfx_print(&m_nrf_lcd, &display_operation.element.text.text_position,
                1, display_operation.element.text.p_string, &orkney_8ptFontInfo, false));
            break;
			
        case DISPLAY_POINT:
            nrf_gfx_point_draw(&m_nrf_lcd, &display_operation.element.point.point, 1);
            break;
			
        case DISPLAY_LINE:
            APP_ERROR_CHECK(nrf_gfx_line_draw(
                &m_nrf_lcd, &display_operation.element.line.line, 1));
            break;
			
        case DISPLAY_CIRCLE:
            APP_ERROR_CHECK(nrf_gfx_circle_draw(
                &m_nrf_lcd, &display_operation.element.circle.circle,
                1, display_operation.element.circle.fill));
            break;
			
        case DISPLAY_RECTANGLE:
            APP_ERROR_CHECK(nrf_gfx_rect_draw(
                &m_nrf_lcd, &display_operation.element.rectangle.rectangle,
                display_operation.element.rectangle.thickness, display_operation.element.rectangle.color,
                display_operation.element.rectangle.fill));
            break;
			
        case DISPLAY_LOG:
            // print log buffer
            break; // not yet implemented
			
        case DISPLAY_CLEAR:
            nrf_gfx_screen_fill(&m_nrf_lcd, 0);
            break;

        default:
            NRF_LOG_INFO("COMMAND WAS EMPTY");
            break;
        }
		
        nrf_gfx_display(&m_nrf_lcd);
        xSemaphoreGive(mutex_i2c);
        vTaskDelay(50);     //20Hz

    }
}


