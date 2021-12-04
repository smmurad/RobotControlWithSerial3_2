
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

//Communication
#include "arq.h"

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

        vTaskSuspend(NULL); // Suspend myself
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

/*

static void log_init(void) {
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
} */

int main(void) {
    bsp_board_init(BSP_INIT_LEDS);
    clock_init();
    ir_init();

    
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

    // Do not start any interrupt that uses system functions before system initialisation.
    // The best solution is to start the OS before any other initalisation.
    arq_init();

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
   /*
    if (pdPASS != xTaskCreate(Sensortower_tester, "SensortowerTest", 256, NULL, 1, &sensor_tower_tester)){
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    */

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



