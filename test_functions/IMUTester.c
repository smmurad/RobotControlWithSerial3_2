#include "ICM_20948.h"
#include "defines.h"
#include "freeRTOS.h"
#include "functions.h"
#include "stdio.h"
#include "math.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "microsd.h"
#include "kalmanFilter.h"
#include "display.h"
#include "server_communication.h"
#include "EstimatorTask.h"
#include "globals.h"
#include "ControllerTask.h"
#include "boards.h"
#include "IMUTester.h"
bool gyroCalibrate = true;
float accelXoffset = 0;
float accelYoffset = 0 ;
float gyroOffset = 0.0;

void IMU_tester(void *pvParameters){
    NRF_LOG_INFO("Enter IMU tester");
    //for(int t = 0;t<1000;t++){ //only for calibration data collection
    while(1){
        IMU_reading_t gyro;
        IMU_reading_t accel;
            if(gyroCalibrate){//neccesary to complete this part only once
            uint16_t i;
            uint16_t samples = 300;
            float gyroF = 0;
            float accelFX = 0;
            float accelFY = 0;
            int fails = 0;
            int sucsess = 0;
            NRF_LOG_INFO("IMU calib init done");
            NRF_LOG_INFO("Enter IMU calibration");
            for (i = 0; i < samples; i++){
                IMU_read(); //needs to be called to get new gyro data
                gyro = IMU_getGyro();
                accel = IMU_getAccel();
                gyroF += gyro.z;
                accelFX += accel.x;
                accelFY += accel.y;

                vTaskDelay(40);

                sucsess++;
        
                while (!IMU_new_data()){
                    vTaskDelay(20); // wait for new data
                    fails++;
                    NRF_LOG_INFO("Waiting for new IMU data");
                }
            }
            gyroOffset = gyroF / (float)samples;
            accelXoffset = accelFX/(float)samples;
            accelYoffset = accelFY /(float)samples;
            gyroCalibrate = false;
        }
        IMU_read();
        gyro = IMU_getGyro();
        accel = IMU_getAccel();
       // float g_f = gyro.z*10000;
       // int g_z = (int)g_f;
        
       /* float ax_f = accel.x*10000;
        int ax = (int)ax_f;
        float ay_f = accel.y*10000;
        int ay = (int)ay_f; */
        
        //printf("\n%d",g_z);
        printf("\nAccel X = %.2f",accel.x-accelXoffset);
        printf("\nAccel Y = %.2f",accel.y-accelYoffset);
       // NRF_LOG_INFO("gyro z: " NRF_LOG_FLOAT_MARKER "\t\t\r\n", NRF_LOG_FLOAT(gyro.z));
        vTaskDelay(250);
    }
}