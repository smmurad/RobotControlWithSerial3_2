
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
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

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

// Coap includes
#include "app_scheduler.h"
#include "app_timer.h"
#include "bsp_thread.h"

#include "thread_coap_utils.h"
#include "thread_utils.h"

#include <openthread/instance.h>
#include <openthread/thread.h>

#define THREAD_STACK_TASK_STACK_SIZE     (( 1024 * 8 ) / sizeof(StackType_t))   /**< FreeRTOS task stack size is determined in multiples of StackType_t. */
#define LOG_TASK_STACK_SIZE              ( 1024 / sizeof(StackType_t))          /**< FreeRTOS task stack size is determined in multiples of StackType_t. */
#define THREAD_STACK_TASK_PRIORITY       2
#define LOG_TASK_PRIORITY                1
#define LED1_TASK_PRIORITY               1
#define LED2_TASK_PRIORITY               1
#define LED1_BLINK_INTERVAL              427
#define LED2_BLINK_INTERVAL              472
#define SCHED_QUEUE_SIZE                 32                                      /**< Maximum number of events in the scheduler queue. */
#define SCHED_EVENT_DATA_SIZE            APP_TIMER_SCHED_EVENT_DATA_SIZE        

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
    .led2_task         = NULL,
};

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

/////////////////// ADDDING COAP CODE ////////////////////

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

static void bsp_event_handler(bsp_event_t event)
{
    switch (event)
    {
        case BSP_EVENT_KEY_3:
            thread_coap_utils_provisioning_enable_set(true);
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
    // we dont need buttons, also this crashes the software -Sigurd
    if(false)
    {
        uint32_t error_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
        APP_ERROR_CHECK(error_code);
    }

    uint32_t error_code1 = bsp_thread_init(thread_ot_instance_get());
    APP_ERROR_CHECK(error_code1);
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
        .coap_client_enabled               = false,
        .configurable_led_blinking_enabled = true,
    };

    thread_coap_utils_init(&thread_coap_configuration);
}

static void thread_stack_task(void * arg)
{
    UNUSED_PARAMETER(arg);

    while (1)
    {
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

/////////////////// COAP END ////////////////////


/**@brief A function which is hooked to idle task.
 * @note Idle hook must be enabled in FreeRTOS configuration (configUSE_IDLE_HOOK).
 */
void vApplicationIdleHook(void) {
#if NRF_LOG_ENABLED
     vTaskResume(m_logger_thread);
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
    motor_init();
    servo_init();
    //encoder_init_int();
    encoder_with_counter_init();
    taskEXIT_CRITICAL();
    i2c_init();
    vTaskDelay(30);
    IMU_init();

    //vTaskDelay(1500);
    //setMotorSpeedReference(60,60);

    vTaskPrioritySet(handle_user_task, 1);
    vTaskDelay(5000);

    NRF_LOG_INFO("IMU reading: %d\n\r", g_IMU_float_gyroX());

    //mag_init(MAG_OS_128);//oversampling rate used to set datarate 16->80hz 32->40hz 64->20hz 128->10hz
    //the rest of this is just used for testing and displaying values
    //vTaskSuspend(NULL);//no need to run more here except for debugging purposes

    //vTaskPrioritySet(handle_user_task, 1);
    
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
    //cartesian target;
    
	
    NRF_LOG_INFO("User task: init complete");
    while(true){
        vTaskDelay(1000);
		
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

static void log_init(void) {
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void led_toggle_task_function (void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    while (true)
    {
        bsp_board_led_invert(BSP_BOARD_LED_0);

        /* Delay a task for a given number of ticks */
        vTaskDelay(1000);

        /* Tasks must be implemented to never return... */
    }
}

TaskHandle_t  led_toggle_task_handle;   /**< Reference to LED0 toggling FreeRTOS task. */
TimerHandle_t led_toggle_timer_handle;  /**< Reference to LED1 toggling FreeRTOS timer. */

#define TASK_DELAY        200           /**< Task delay. Delays a LED0 task for 200 ms */
#define TIMER_PERIOD      1000          /**< Timer period. LED1 timer will expire after 1000 ms */

int main(void) {
    // bsp_board_init(BSP_INIT_LEDS);
    clock_init();
    ir_init();
    log_init();

    // ADDING COAP INIT //
    #ifdef MBEDTLS_THREADING
    freertos_mbedtls_mutex_init();
    #endif
    timer_init();
    thread_instance_init();
    thread_coap_init();
    thread_bsp_init();
    if(false)
    {

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
    }

    /* Create task for LED0 blinking with priority set to 2 */
    if (false)
        UNUSED_VARIABLE(xTaskCreate(led_toggle_task_function, "LED0", configMINIMAL_STACK_SIZE + 200, NULL, 2, &led_toggle_task_handle));

    
    position_estimate_t pos_est = {0,0,0};
    set_position_estimate(&pos_est);

/**
	* The reset-reason functions are extremely useful as they help you rule out
	* issues with power supply without having to resort to a oscilloscope.
	*/
#if !NRF_LOG_ENABLED || !NRF_LOG_BACKEND_RTT_ENABLED
    //char const * error_msg;
    //vTraceEnable(TRC_START_AWAIT_HOST);
    //vTraceEnable(TRC_INIT);
    //error_msg = xTraceGetLastError();
#endif

/*
	* The reset-reason functions are extremely useful as they help you rule out
	* issues with power supply without having to resort to a oscilloscope.
	*/
    //sd_power_reset_reason_get(&reset_reason);
    //sd_power_reset_reason_clr(0xFFFFFFFF);

/**
	* The reset-reason functions are extremely useful as they help you rule out
	* issues with power supply without having to resort to a oscilloscope.
	*/
// RecorderDataPtr
#if !NRF_LOG_ENABLED || !NRF_LOG_BACKEND_RTT_ENABLED
   //vTraceEnable(TRC_START);
   //vTraceEnable(TRC_START_AWAIT_HOST);
   //error_msg = xTraceGetLastError();
#endif


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
	
    // Disable because it crashed the program -Sigurd 
    // if (pdPASS != xTaskCreate(display_task, "DISP", 128, NULL, 1, &handle_display_task))
    //     APP_ERROR_HANDLER(NRF_ERROR_NO_MEM); 

    if (pdPASS != xTaskCreate(user_task, "USER", 128, NULL, 4, &handle_user_task)) //needs elevated priority because init functions
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

    if (pdPASS != xTaskCreate(microsd_task, "SD", 256, NULL, 1, &handle_microsd_task))
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    
   	
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
    NRF_LOG_INFO("EverFreeHeapSize5 %d", freeHeapSize5); //If 
    NRF_LOG_INFO("\nInitialization done. SLAM application now starting.\n.");
    if(PRINT_DEBUG)printf("Application starting");
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



