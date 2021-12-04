/**@brief IR sensor task.
 *
 * @details Task for obtaining ADC measurements from the IR sensor and printing the result on the display.

 */
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
#include "microsd.h"

#include "DisplayTask.h"
#include "mag3110.h"
#include "task.h"
#include "positionEstimate.h"

extern uint8_t gHandshook;
extern uint8_t gPaused;
extern QueueHandle_t scanStatusQ;
extern QueueHandle_t poseControllerQ;
extern QueueHandle_t queue_microsd;
extern SemaphoreHandle_t xPoseMutex;

//INIT DISPLAY servo AND IR SENSOR BEFORE USING
void ir_task(void *arg) {
    //ir_init(IR_RESOLUTION_10);
    //vServo_setAngle(0);

    char strIR1[10];
    char strIR2[10];
    char strIR3[10];
    char strIR4[10];
    for (;;) {
        int sensor1 = IrAnalogToMM(ir_read_blocking(IR_SENSOR_1),IR_SENSOR_1);
        int sensor2 = IrAnalogToMM(ir_read_blocking(IR_SENSOR_2),IR_SENSOR_2);
        int sensor3 = IrAnalogToMM(ir_read_blocking(IR_SENSOR_3),IR_SENSOR_3);
        int sensor4 = IrAnalogToMM(ir_read_blocking(IR_SENSOR_4),IR_SENSOR_4);

        sprintf(strIR1, "ir1: %i", sensor1); // display on screen
        display_text_on_line(2, strIR1);
        sprintf(strIR2, "ir2: %i", sensor2); // display on screen
        display_text_on_line(3, strIR2);
        sprintf(strIR3, "ir3: %i", sensor3); // display on screen
        display_text_on_line(4, strIR3);
        sprintf(strIR4, "ir4: %i", sensor4); // display on screen
        display_text_on_line(5, strIR4);

        vTaskDelay(100);
    }
}


/**@brief Magnetometer task
 *
 * @details Task to initialize, calibrate and read print the heading from magnetometer on display.
 *			Note that the calibration requires the full physical rotation of the
 *			control system, and has therefore been commented out by default.
 */

 //init i2c display and magnetometer before using
void mag_task(void *arg) {
    mag_init(MAG_OS_128);
    //mag_calibrate();
    char str[20];
    for (;;) {
        float heading = mag_heading();
        sprintf(str, "HEADING: %d", (int)heading);
        display_text_on_line(4, str);
        // NRF_LOG_INFO("Heading:" NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(mag_heading()));
        vTaskDelay(1000);
    }
}

struct sCartesian {
    float x;
    float y;
};

//task used to calibrate ir sensors by driving away /towards wall and loging to sd
void vMainSensorCalibrationTask(void *pvParameters) {
    char strLOG[150];
    /* Task init */
    float theta_hat = 0;
    float xhat = 0;
    float yhat = 0;
    //uint8_t rotationDirection = moveCounterClockwise;
    //uint8_t servoResolution = 1;
    uint8_t robotMovement = moveStop;


    int16_t waypoint[2];
    waypoint[0]=0;
    waypoint[1]=0;

    uint8_t scan= 0;
    while (true) {
        if ((gHandshook == true) && (gPaused == false)) {

            // Set scanning resolution depending on which movement the robot is executing.
            if (xQueueReceive(scanStatusQ, &robotMovement, 150) == pdTRUE) {
                // Set servo step length according to movement,
                // Note that the iterations are skipped while robot is rotating (see further downbelow)
                switch (robotMovement) {
                case moveStop:
                    scan = true;
                    break;
                case moveForward:
                case moveBackward:

                    break;
                case moveClockwise:
                case moveCounterClockwise:


                    break;
                default:
                    break;
                }
            }

            if(scan){
            vServo_setAngle(0);
            vTaskDelay(800); // Wait total of 200 ms for servo to reach set point
            taskYIELD();
            uint16_t sensor16[4];
            sensor16[0] = ir_read_blocking(IR_SENSOR_1);
            sensor16[1] = ir_read_blocking(IR_SENSOR_2);
            sensor16[3] = ir_read_blocking(IR_SENSOR_3);
            sensor16[2] = ir_read_blocking(IR_SENSOR_4);


            xSemaphoreTake(xPoseMutex, 40);
            theta_hat = get_position_estimate_heading();
            xhat = get_position_estimate_x() * 1000; //m to mm
            yhat = get_position_estimate_y() * 1000;
            xSemaphoreGive(xPoseMutex);
            microsd_write_operation_t write;
            sprintf(strLOG,"Ta:0,x:%f,y:%f,T:%f,F:%u,L:%u,B:%u,R:%u\n",xhat,yhat,theta_hat,sensor16[0],sensor16[1],sensor16[2],sensor16[3]);
            write.filename = "SC";
            write.content = strLOG;
            xQueueSendToBack(queue_microsd, &write, portMAX_DELAY);  
            

            vServo_setAngle(90);
            vTaskDelay(1500); // Wait total of 200 ms for servo to reach set point
            taskYIELD();
			
            sensor16[0] = ir_read_blocking(IR_SENSOR_1);
            sensor16[1] = ir_read_blocking(IR_SENSOR_2);
            sensor16[3] = ir_read_blocking(IR_SENSOR_3);
            sensor16[2] = ir_read_blocking(IR_SENSOR_4);


            xSemaphoreTake(xPoseMutex, 40);
            theta_hat = get_position_estimate_heading();
            xhat = get_position_estimate_x() * 1000; //m to mm
            yhat = get_position_estimate_y() * 1000;
            xSemaphoreGive(xPoseMutex);
            sprintf(strLOG,"Ta:90,x:%f,y:%f,T:%f,F:%u,L:%u,B:%u,R:%u\n",xhat,yhat,theta_hat,sensor16[0],sensor16[1],sensor16[2],sensor16[3]);
            write.filename = "SC";
            write.content = strLOG;
            xQueueSendToBack(queue_microsd, &write, portMAX_DELAY);  
            struct sCartesian Setpoint = {(waypoint[0])/10, (waypoint[1])/10};
            xQueueSend(poseControllerQ, &Setpoint, 100);
            waypoint[0]+=25;
            vServo_setAngle(0);
            vTaskDelay(1000);
            scan = false;
            }
        }

        else { // Disconnected or unconfirmed
            vServo_setAngle(0);
            // Reset servo incrementation
            //rotationDirection = moveCounterClockwise;
            vTaskDelay(100);
        }
    } // While end
}