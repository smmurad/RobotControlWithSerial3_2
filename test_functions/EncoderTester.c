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

void Encoder_tester(void *pvParameters) {
    NRF_LOG_INFO("Enter encoder test");
    //for (int t=0;t<122;t++){
    float dTheta = 0;
    while(true){

        //vMotorMovementSwitch(50,50);
        //motor_forward(50);
        //NRF_LOG_INFO("Motor driving");
        //vTaskDelay(3000);
        //motor_stop();
        //motor_forward(50);
        //encoderTicks ticks = encoder_with_counter_get_ticks_since_last_time();
        //(void) ticks;
        encoderTicks allTicks = encoder_with_counter_get_all_ticks();
        //int right = (int)ticks.right;
        //int left = (int)ticks.left;
        //float dLeft = (float)(right * WHEEL_FACTOR_MM);	// Distance left wheel has traveled since last sample
		//float dRight = (float)(left * WHEEL_FACTOR_MM);	// Distance right wheel has traveled since last sample
        /*
        if (ticks.rightdirection == 1 && ticks.leftdirection == -1){
            dTheta = (dRight+dLeft)/(2*WHEELBASE_MM);
        } else if (RightMotorDirection == -1 && LeftMotorDirection == 1) {
            dTheta = -(dRight+dLeft)/(2*WHEELBASE_MM);
        } else {
            dTheta = (dRight - dLeft) / WHEELBASE_MM;
        }
        */
        //dTheta = (dRight - dLeft) / WHEELBASE_MM;
        (void)dTheta;
        //vMotorMovementSwitch(25,25);
       // NRF_LOG_INFO("Motor driving");
        //vTaskDelay(1000);
        //motor_stop();
        //motor_backward(50);
        //encoderTicks ticks = encoder_get_ticks();
        //encoderTicks allTicks = encoder_get_all_ticks();

        printf("\nallLeft = %ld",allTicks.left);
        printf("\nallRight = %ld",allTicks.right);

        //printf("\nRight = %.2f",dRight);
        //printf("\ndLeft = %.2f",dLeft);
        //printf("\n---------------");
        vTaskDelay(40);
    }
}