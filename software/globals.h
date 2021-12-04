/************************************************************************/
// File:            globals.h                                      //
// Author:                                                        //
// Purpose:         Organize all global stash and shared stuff          //
//                                                                      //
/************************************************************************/

#ifndef GLOBALS_H
#define GLOBALS_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

extern TaskHandle_t handle_display_task,
    handle_user_task,
    handle_microsd_task,
    pose_estimator_task,
    pose_controller_task,
    communication_task,
    sensor_tower_task,
    arq_task,
    motor_speed_controller_task;

/* Semaphore handles */
extern SemaphoreHandle_t xScanLock;
extern SemaphoreHandle_t xPoseMutex;
extern SemaphoreHandle_t xTickBSem;
extern SemaphoreHandle_t xControllerBSem;
extern SemaphoreHandle_t xCommandReadyBSem;
extern SemaphoreHandle_t mutex_spi;
extern SemaphoreHandle_t mutex_i2c;
//extern SemaphoreHandle_t xCollisionMutex;

/* Queues */
//QueueHandle_t movementQ = 0;
extern QueueHandle_t poseControllerQ;
extern QueueHandle_t scanStatusQ;
extern QueueHandle_t queue_microsd;

QueueHandle_t encoderTicksToMotorSpeedControllerQ;
QueueHandle_t encoderTicksToMotorPositionControllerQ;
QueueHandle_t encoderTicksToEstimatorTaskQ;

// Flag to indicate connection status. Interrupt can change handshook status
extern uint8_t gHandshook;
extern uint8_t gPaused;


//Globals for direction
extern int LeftMotorDirection;
extern int RightMotorDirection;


/* STRUCTURE */
typedef struct sPolar {
    float heading;
    int16_t distance;
} polar;

typedef struct sCartesian {
    float x;
    float y;
} cartesian;










#endif  /* GLOBALS_H */