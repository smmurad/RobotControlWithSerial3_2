#include "defines.h"
#include "encoder_with_counter.h"
#include "freeRTOS.h"
#include "functions.h"
#include "math.h"
#include "motor.h"
#include "nrf_log.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "ControllerTask.h"
#include "globals.h"
//encoderTicks encoder;

void Encoder_tester_2(void *pvParameters) {
    printf("Enter encoder test");
    //for (int t=0;t<122;t++){
	//encoder_with_counter_init();
    
    //motor_forward(30);

    while(true){
		encoderTicks ticks = {0,0}; //encoder_with_counter_get_ticks_since_last_time();
        printf("Left: %ld \tRight: %ld\n\r", ticks.left, ticks.right);
//        ticks =  encoder_with_counter_get_all_ticks();
        printf("Total Left: %ld \tRight: %ld\n\r", ticks.left, ticks.right);
        vTaskDelay(500);
    }
}