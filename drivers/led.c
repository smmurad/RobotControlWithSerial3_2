#include "nrfx_gpiote.h"
#include "app_pwm.h"

#include "nrf.h"
#include "app_error.h"
#include "bsp.h"
#include "nrf_delay.h"

#define PIN_LED1 13
#define PIN_LED2 9
#define PIN_LED3 10

void led_init()
{
	nrfx_err_t err = NRFX_SUCCESS;

	//GPIO
	if(!nrfx_gpiote_is_init())
		err = nrfx_gpiote_init();
	APP_ERROR_CHECK(err);
	nrfx_gpiote_out_config_t led_config = NRFX_GPIOTE_CONFIG_OUT_SIMPLE(0);
	nrfx_gpiote_out_init(PIN_LED1, &led_config);
    nrfx_gpiote_out_init(PIN_LED2, &led_config);
    nrfx_gpiote_out_init(PIN_LED3, &led_config);
	//END OF TEST

 }

 void led_on(int led){
     switch (led)
     {
     case 1:
         nrfx_gpiote_out_set(PIN_LED1);
         break;
    case 2:
        nrfx_gpiote_out_set(PIN_LED2);
        break;
    case 3:
        nrfx_gpiote_out_set(PIN_LED3);
        break;
     
     default:
         break;
     }
 }

 void led_off(int led){
     switch (led)
     {
     case 1:
         nrfx_gpiote_out_clear(PIN_LED1);
         break;
    case 2:
        nrfx_gpiote_out_clear(PIN_LED2);
        break;
    case 3:
        nrfx_gpiote_out_clear(PIN_LED3);
        break;
     
     default:
         break;
     }
 }