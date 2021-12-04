#include "defines.h"
#include "freeRTOS.h"
#include "ir.h"
#include "math.h"
#include "nrf_log.h"
#include "queue.h"
#include "semphr.h"
#include "server_communication.h"
#include "servo.h"
#include "timers.h"
#include "i2c.h"
#include "functions.h"
#include "SensorTowerTask.h"
#include "globals.h"
#include "MainComTask.h"
#include <stdlib.h>
#include "microsd.h"
#include "boards.h"

char pos_test[30];
int posCounter_test = 0;

bool scan_test = false;

int time_test = 0;
int oldTime_test = 0;
char irAnalogReading_test[20];
uint8_t sensorDataCM_test[NUM_DIST_SENSORS];		 
uint16_t sensorDataMM_test[NUM_DIST_SENSORS];
int calibrationCounter_test = 0;
int distance_test = 125;
int counter_test = 0;
void Sensortower_tester(void *pvParameters) {
    //servo_init();
    vServo_setAngle(-6);
    IR_Sensor_t sensor = IR_SENSOR_4;
    while(true) {
        uint16_t ir_reading = IrAnalogToMM(ir_read_blocking(sensor),sensor);
        uint16_t k = 0;
        for (int i = 0; i<10;i++){
        uint16_t ir_raw_reading = ir_read_blocking(sensor);
        k += ir_raw_reading;
        vTaskDelay(40);
        }
    //    printf("\n%d",k/10);
        printf("\n%d", ir_reading);
        vTaskDelay(2000);
    }
}