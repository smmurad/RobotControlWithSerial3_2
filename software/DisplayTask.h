/************************************************************************/
// File:            DisplayTask                                         //
// Author:                                                              //
// Purpose:                                                             //
//                                                                      //
/************************************************************************/

#ifndef TASK_DISPLAY_H_
#define TASK_DISPLAY_H_

#include "display.h"

extern const nrf_gfx_font_desc_t orkney_8ptFontInfo;
extern QueueHandle_t queue_display;

extern const nrf_lcd_t m_nrf_lcd;



void display_task(void *arg);

#endif