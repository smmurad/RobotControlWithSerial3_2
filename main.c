
/**
 * Copyright (c) 2017 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup freertos_coap_server_example_main main.c
 * @{
 * @ingroup freertos_coap_server_example
 *
 * @brief Thread CoAP server example with FreeRTOS Application main file.
 *
 * This file contains the source code for a sample application using Thread CoAP server and FreeRTOS.
 *
 */
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "sensorsim.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_freertos.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "FreeRTOS.h"
#include "task.h"
#include "boards.h"
#include "timers.h"
#include "semphr.h"
#include "fds.h"
#include "nrf_drv_clock.h"
#include "app_uart.h"
#include "task.h"

#define NRF_LOG_MODULE_NAME APP
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
NRF_LOG_MODULE_REGISTER();

///Missing h-files
#include "nrf_delay.h"
#include "nrf_drv_ppi.h"
#include "nrf_twi_mngr.h"
#include <math.h>
#include <stdio.h>


//Own files

#include "network.h"
#include "server_communication.h"
#include "simple_protocol.h" 

//Drivers
#include "DisplayTask.h"
//#include "MPU6050.h"
#include "defines.h"
#include "software/globals.h"
//#include "encoder.h"
#include "encoder_with_counter.h"
#include "i2c.h"
#include "ir.h"
//#include "mag3110.h"
#include "microsd.h"
#include "motor.h"
#include "servo.h"
#include "oled20.h"
#include "led.h"
#include "ICM_20948.h"

//Software
#include "DebugFunctions.h"
#include "SensorTowerTask.h"
#include "ControllerTask.h"
#include "EstimatorTask.h"
#include "NewEstimatorTask.h"
#include "MainComTask.h"
#include "globals.h"
#include "EncoderTester.h"
#include "EncoderWithCounterTester.h"
#include "IMUTester.h"
#include "SensorTowerTester.h"
#include "positionEstimate.h"
#include "MotorSpeedControllerTask.h"

#include "robot_config.h"

#include "app_timer.h"
#include "bsp_thread.h"

#include "thread_coap_utils.h"
#include "thread_utils.h"

#include <openthread/instance.h>
#include <openthread/thread.h>

#ifdef MBEDTLS_THREADING
#include "freertos_mbedtls_mutex.h"
#endif

#define THREAD_STACK_TASK_STACK_SIZE     (( 1024 * 8 ) / sizeof(StackType_t))   /**< FreeRTOS task stack size is determined in multiples of StackType_t. */
#define LOG_TASK_STACK_SIZE              ( 1024 / sizeof(StackType_t))          /**< FreeRTOS task stack size is determined in multiples of StackType_t. */
#define THREAD_STACK_TASK_PRIORITY       2
#define LOG_TASK_PRIORITY                1
#define LED1_TASK_PRIORITY               1
#define LED2_TASK_PRIORITY               1
#define LED1_BLINK_INTERVAL              427
#define LED2_BLINK_INTERVAL              472

#if NRF_LOG_ENABLED
static TaskHandle_t m_logger_task;  /**< Definition of Logger task. */
#endif

typedef struct
{
    TaskHandle_t     thread_stack_task;     /**< Thread stack task handle */
    TaskHandle_t     logger_task;           /**< Definition of Logger task. */
    TaskHandle_t     led1_task;             /**< LED1 task handle*/
    TaskHandle_t     led2_task;             /**< LED2 task handle*/

} application_t;


application_t m_app =
{
    .thread_stack_task = NULL,
    .logger_task       = NULL,
    .led1_task         = NULL,
    .led2_task         = NULL,

};

/***************************************************************************************************
 * @section CoAP
 **************************************************************************************************/
#define NRF_LOG_ENABLED 1

#if NRF_LOG_ENABLED
static TaskHandle_t m_logger_thread;                                /**< Definition of Logger thread. */

/**@brief Thread for handling the logger.
 *
 * @details This thread is responsible for processing log entries if logs are deferred.
 *          Thread flushes all log entries and suspends. It is resumed by idle task hook.
 *
 * @param[in]   arg   Pointer used for passing some arbitrary information (context) from the
 *                    osThreadCreate() call to the thread.
 */
static void logger_thread(void * arg)
{
    UNUSED_PARAMETER(arg);

    while (1)
    {
        NRF_LOG_FLUSH();

        // Deactivated this as the thread did not activate itself - Sigurd
        // vTaskSuspend(NULL); // Suspend myself
    }
}
#endif //NRF_LOG_ENABLED


/* DEFINING GLOBAL AND SHARED FLAG VARIABLES */
TaskHandle_t handle_display_task,
    handle_user_task,
    handle_microsd_task,
    pose_estimator_task,
    pose_controller_task,
    motor_speed_controller_task,
    communication_task,
    sensor_tower_task,
    arq_task,
    imu_tester,
    encoder_tester,
    encoder_with_counter_tester,
    sensor_tower_tester;

/* Semaphore handles */
SemaphoreHandle_t xScanLock;
SemaphoreHandle_t xPoseMutex;
SemaphoreHandle_t xTickBSem;
SemaphoreHandle_t xControllerBSem;
SemaphoreHandle_t xCommandReadyBSem;
SemaphoreHandle_t mutex_spi;
SemaphoreHandle_t mutex_i2c;
//SemaphoreHandle_t xCollisionMutex;

/* Queues */
QueueHandle_t poseControllerQ = 0;
QueueHandle_t scanStatusQ = 0;
QueueHandle_t queue_microsd = 0;

// Flag to indicate connection status. Interrupt can change handshook status
uint8_t gHandshook = false;
uint8_t gPaused = false;

// all SPI driver interaction occurs within mutex, so can safely use global bool
bool shared_SPI_init = false;


static inline void light_on(void)
{
    vTaskResume(m_app.led1_task);
    vTaskResume(m_app.led2_task);
}


static inline void light_off(void)
{
    vTaskSuspend(m_app.led1_task);
    LEDS_OFF(BSP_LED_2_MASK);

    vTaskSuspend(m_app.led2_task);
    LEDS_OFF(BSP_LED_3_MASK);
}


static inline void light_toggle(void)
{
    if (!thread_coap_utils_light_is_led_blinking())
    {
        light_on();
    }
    else
    {
        light_off();
    }
}


static void on_light_change(thread_coap_utils_light_command_t command)
{
    switch (command)
    {
        case THREAD_COAP_UTILS_LIGHT_CMD_ON:
            light_on();
            break;

        case THREAD_COAP_UTILS_LIGHT_CMD_OFF:
            light_off();
            break;

        case THREAD_COAP_UTILS_LIGHT_CMD_TOGGLE:
            light_toggle();
            break;

        default:
            ASSERT(false);
            break;
    }
}

/***************************************************************************************************
 * @section Signal handling
 **************************************************************************************************/

void otTaskletsSignalPending(otInstance * p_instance)
{
    if (m_app.thread_stack_task == NULL)
    {
        return;
    }

    UNUSED_RETURN_VALUE(xTaskNotifyGive(m_app.thread_stack_task));
}


void otSysEventSignalPending(void)
{
    static BaseType_t xHigherPriorityTaskWoken;

    if (m_app.thread_stack_task == NULL)
    {
        return;
    }

    vTaskNotifyGiveFromISR(m_app.thread_stack_task, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/***************************************************************************************************
 * @section State change handling
 **************************************************************************************************/

 static void thread_state_changed_callback(uint32_t flags, void * p_context)
{
    if (flags & OT_CHANGED_THREAD_ROLE)
    {
        switch(otThreadGetDeviceRole(p_context))
        {
            case OT_DEVICE_ROLE_CHILD:
            case OT_DEVICE_ROLE_ROUTER:
            case OT_DEVICE_ROLE_LEADER:
                break;

            case OT_DEVICE_ROLE_DISABLED:
            case OT_DEVICE_ROLE_DETACHED:
            default:
                thread_coap_utils_provisioning_enable_set(false);
                break;
        }
    }

    NRF_LOG_INFO("State changed! Flags: 0x%08x Current role: %d\r\n", flags, otThreadGetDeviceRole(p_context));
}

/***************************************************************************************************
 * @section Buttons
 **************************************************************************************************/
static thread_coap_utils_light_command_t m_command = THREAD_COAP_UTILS_LIGHT_CMD_OFF; /**< This variable stores command that has been most recently used. */

static void bsp_event_handler(bsp_event_t event)
{
    switch (event)
    {
        case BSP_EVENT_KEY_0:
            thread_coap_utils_unicast_light_request_send(THREAD_COAP_UTILS_LIGHT_CMD_TOGGLE);
            break;
        case BSP_EVENT_KEY_1:
        {
            m_command = ((m_command == THREAD_COAP_UTILS_LIGHT_CMD_OFF) ? THREAD_COAP_UTILS_LIGHT_CMD_ON :
                                                                          THREAD_COAP_UTILS_LIGHT_CMD_OFF);

            thread_coap_utils_multicast_light_request_send(m_command,
                                                           THREAD_COAP_UTILS_MULTICAST_REALM_LOCAL);
            break;
        }

            break;
        case BSP_EVENT_KEY_3:
            NRF_LOG_INFO("Pressing button 3");
            break;

        default:
            return;
    }
}

/***************************************************************************************************
 * @section Initialization
 **************************************************************************************************/

 /**@brief Function for initializing the Application Timer Module
 */
static void timer_init(void)
{
    uint32_t error_code = app_timer_init();
    APP_ERROR_CHECK(error_code);
}


/**@brief Function for initializing the Thread Board Support Package
 */
static void thread_bsp_init(void)
{
    uint32_t error_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(error_code);

    error_code = bsp_thread_init(thread_ot_instance_get());
    APP_ERROR_CHECK(error_code);
}


/**@brief Function for initializing the Thread Stack
 */
static void thread_instance_init(void)
{
    thread_configuration_t thread_configuration =
    {
        .radio_mode        = THREAD_RADIO_MODE_RX_ON_WHEN_IDLE,
        .autocommissioning = true,
    };

    thread_init(&thread_configuration);
    thread_cli_init();
    thread_state_changed_callback_set(thread_state_changed_callback);
}


/**@brief Function for initializing the Constrained Application Protocol Module
 */
static void thread_coap_init(void)
{
    thread_coap_utils_configuration_t thread_coap_configuration =
    {
        .coap_server_enabled               = true,
        .coap_client_enabled               = true,
        .configurable_led_blinking_enabled = false,
    };

    thread_coap_utils_init(&thread_coap_configuration);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}
static void thread_stack_task(void * arg)
{
    UNUSED_PARAMETER(arg);

    while (1)
    {
        NRF_LOG_INFO("Processing thread");
        thread_process();
        UNUSED_RETURN_VALUE(ulTaskNotifyTake(pdTRUE, portMAX_DELAY));
    }
}

/***************************************************************************************************
 * @section Leds
 **************************************************************************************************/

static void led1_task(void * arg)
{
    UNUSED_PARAMETER(arg);

    while (1)
    {
        LEDS_INVERT(BSP_LED_2_MASK);
        vTaskDelay(LED1_BLINK_INTERVAL);
    }
}


static void led2_task(void * arg)
{
    UNUSED_PARAMETER(arg);

    while (1)
    {
        LEDS_INVERT(BSP_LED_3_MASK);
        vTaskDelay(LED2_BLINK_INTERVAL);
    }
}

/***************************************************************************************************
 * @section Idle hook
 **************************************************************************************************/

/**@brief A function which is hooked to idle task.
 * @note Idle hook must be enabled in FreeRTOS configuration (configUSE_IDLE_HOOK).
 */
void vApplicationIdleHook( void )
{
#if NRF_LOG_ENABLED
    if (m_logger_task)
    {
        vTaskResume(m_logger_task);
    }
#endif
}

/**@brief Function for initializing the clock.
 */
static void clock_init(void) {
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
}

//globals for encoder
int RightMotorDirection = 1;
int LeftMotorDirection = 1;


/**@brief User task
 *
 * @details Task for miscellaneous operations. Currently only adding display ops
 *			to the display_task's queue.
 */
 
static void user_task(void *arg) {

/*	Template for writing to microsd card. If logging faster than 400ms to sd card it can slow the robot down considerably.  
    microsd_write_operation_t write;
    write.filename = "Log.txt";			// Filename can be anything
    write.content = "Starting slam application \n";
    xQueueSendToBack(queue_microsd, &write, portMAX_DELAY);
*/
   
    NRF_LOG_INFO("user task: initializing");
    //UNUSED_PARAMETER(arg);
    //initialization of modules should be done after FreeRtos startup
    taskENTER_CRITICAL();
    NRF_LOG_INFO("user task: after enter critical");
    
    motor_init();
    servo_init();
    // encoder_init_int(); // <- however this works
    // encoder_with_counter_init(); //crashes the thread
    taskEXIT_CRITICAL();
    i2c_init();
    vTaskDelay(30);
    IMU_init();

    vTaskPrioritySet(handle_user_task, 1);
    vTaskDelay(5000);

    // NRF_LOG_INFO("IMU reading: %d\n\r", g_IMU_float_gyroX());

    //mag_init(MAG_OS_128);//oversampling rate used to set datarate 16->80hz 32->40hz 64->20hz 128->10hz
    //the rest of this is just used for testing and displaying values
    //vTaskSuspend(NULL);//no need to run more here except for debugging purposes

    //vTaskDelay(5000);

    //char str1[20];
    //char str2[20];
    //char str3[20];
    //char str4[20];
	//char str5[20];
	
	float targetX = 50;
	float targetY = 0;
	bool sent = false;
	bool testWaypoint = false;
	
    NRF_LOG_INFO("User task: init complete");
    while(true){
        vTaskDelay(1000);
		continue;
		// Test-function, sends targetX and targetY to controller some time after initialization, used to test waypoints without server running.
		if(testWaypoint){
			int time = (xTaskGetTickCount()/1000);
			//NRF_LOG_INFO("Time: %i", (int)time);
		
			if ((time > 40) && (sent == false)){
				cartesian target = {targetX, targetY};
				xQueueSend(poseControllerQ, &target, 100); //Sends target to poseControllerQ, which is received and handled by ControllerTask
				sent = true;
				time = 0;
			}
		}
    }
}


int main(void) {
    ir_init();
    log_init();
    clock_init();
    timer_init();

    // ADDING COAP INIT //
    #ifdef MBEDTLS_THREADING
    freertos_mbedtls_mutex_init();
    #endif
    thread_instance_init();
    thread_coap_init();
    thread_bsp_init();
    thread_coap_utils_light_command_handler_set(on_light_change);

    //COAP init end//
    // Start thread stack execution.
    if (pdPASS != xTaskCreate(thread_stack_task, "THR", THREAD_STACK_TASK_STACK_SIZE, NULL, 2, &m_app.thread_stack_task))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    // Start execution.
    if (pdPASS != xTaskCreate(led1_task, "LED1", configMINIMAL_STACK_SIZE, NULL, 1, &m_app.led1_task))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    // Start execution.
    if (pdPASS != xTaskCreate(led2_task, "LED2", configMINIMAL_STACK_SIZE, NULL, 1, &m_app.led2_task))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    
    position_estimate_t pos_est = {0,0,0};
    set_position_estimate(&pos_est);

    #if NRF_LOG_ENABLED
		if (pdPASS != xTaskCreate(logger_thread, "LOGGER", 256, NULL, 1, &m_logger_thread))
			APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
	#endif
    
    //initialize queues
	queue_display = xQueueCreate(5, sizeof(display_operation_t));       //For sending things to display
	queue_microsd = xQueueCreate(5, sizeof(microsd_write_operation_t)); //For writing things to micro SD
	poseControllerQ = xQueueCreate(1, sizeof(struct sCartesian));       // For setpoints to controller
	scanStatusQ = xQueueCreate(1, sizeof(uint8_t));                     // For robot status
    encoderTicksToMotorSpeedControllerQ = xQueueCreate(100, sizeof(encoderTicks)); 
    encoderTicksToMotorPositionControllerQ = xQueueCreate(100, sizeof(encoderTicks)); 
    encoderTicksToEstimatorTaskQ = xQueueCreate(100, sizeof(encoderTicks)); 

	//initialize mutexes
	mutex_spi = xSemaphoreCreateMutex();
    mutex_i2c = xSemaphoreCreateMutex();
	xPoseMutex = xSemaphoreCreateMutex();       // Global variables for robot pose. Only updated from estimator, accessed from many
	xTickBSem = xSemaphoreCreateBinary();       // Global variable to hold robot tick values
    xSemaphoreGive(xTickBSem);
	xControllerBSem = xSemaphoreCreateBinary(); // Estimator to Controller synchronization
	xCommandReadyBSem = xSemaphoreCreateBinary();
	
    if (pdPASS != xTaskCreate(display_task, "DISP", 128, NULL, 1, &handle_display_task))
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM); 

    if (pdPASS != xTaskCreate(user_task, "USER", 512, NULL, 4, &handle_user_task)) //needs elevated priority because init functions
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);

    /********************************************************************************************************************************************************* 
     *Tasks used for testing drivers and hardware
     * Not to be used in final application
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

    /*
	if (pdPASS != xTaskCreate(IMU_tester, "IMU_Test", 256, NULL, 1 ,&imu_tester)){
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    */
   
    /*
    if (pdPASS != xTaskCreate(Encoder_tester, "Encoder Test", 256, NULL, 1, &encoder_tester)){
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    */
    
    /*
    if (pdPASS != xTaskCreate(Encoder_tester_2, "Encoder Cnt Test", 256, NULL, 1, &encoder_with_counter_tester)){
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    } 
    */
    
    /*
	if (pdPASS != xTaskCreate(IMU_tester, "IMU_Test", 256, NULL, 1 ,&imu_tester)){
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    */

    // if (pdPASS != xTaskCreate(Sensortower_tester, "SensortowerTest", 256, NULL, 1, &sensor_tower_tester)){
    //     APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    // }


    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
     * End used for testing drivers and hardware
     * Not to be used in final application
     * ***********************************************************************************************************************************************************/
    if(USE_NEW_ESTIMATOR){
        if (pdPASS != xTaskCreate(vNewMainPoseEstimatorTask, "POSE", 256, NULL, 3, &pose_estimator_task))
            APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }else{
        if (pdPASS != xTaskCreate(vMainPoseEstimatorTask, "POSE", 256, NULL, 3, &pose_estimator_task))
            APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }  
    if(USE_SPEED_CONTROLLER)
    {
        if (pdPASS != xTaskCreate(vMotorSpeedControllerTask, "SPEED", 256, NULL, 2, &motor_speed_controller_task))
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM); 
    }

    if (pdPASS != xTaskCreate(vMainPoseControllerTask, "POSC", 512, NULL, 1, &pose_controller_task))
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);

    if (pdPASS != xTaskCreate(vMainSensorTowerTask, "SnsT", 256, NULL, 1, &sensor_tower_task))
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM); 

    if (pdPASS != xTaskCreate(vMainCommunicationTask, "COM", 256, NULL, 1, &communication_task)) // Moved to this loop in order to use it for thread communications aswell
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM); 

    size_t freeHeapSize5 = xPortGetMinimumEverFreeHeapSize();
    NRF_LOG_INFO("EverFreeHeapSize5 %d", freeHeapSize5);
    NRF_LOG_INFO("\nInitialization done. SLAM application now starting.\n.");
    vTaskStartScheduler();
    for (;;) {
        /**
		* vTaskStartSchedule returns only if the system failed to allocate heap
		* memory. Either reduce heap memory usage, or increase the allocation
		* of heap memory in FreeRTOSConfig.h via configTOTAL_HEAP_SIZE
		*/
        APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN); //See comment above
    }
}


/**
 *@}
 **/
