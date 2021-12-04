/***************************************************************************/
// File:            MainComTask.c											//
// Author:          Stenset, Spring 2020									//
// Purpose:         Keep the main communication task and its functions		//
//					in its own source file, previously located in main.c	//
//																			//
/***************************************************************************/

#include <stdint.h>
#include <stdio.h>

#include "MainComTask.h"
#include "server_communication.h"
#include "FreeRTOS.h"
#include "task.h"
#include "arq.h"
#include "globals.h"
#include "nrf_log.h"
#include "i2c.h"
#include "functions.h"
#include "display.h"
#include "math.h"
#include "boards.h"
#include "positionEstimate.h"
#include "robot_config.h"


message_t message_in;

uint8_t counter = 0;

/* Current position variables */
float thetahat = 0;
int16_t xhat = 0;
int16_t yhat = 0;


void vMainCommunicationTask(void *pvParameters){
	NRF_LOG_INFO("Main Com Task: Initializing");
    struct sCartesian Setpoint = {0, 0}; // Struct for setpoints from server
    
	if (USEBLUETOOTH)
	{
		message_t command_in; // Buffer for recieved messages
		server_communication_init();
		vTaskResume(arq_task);
		uint8_t success = 0;

		while (!success) 
		{
			success = server_connect();
			NRF_LOG_INFO("Main Com Task: Waiting for server connect");
			vTaskDelay(1000);
		}
	
		//bsp_board_led_invert(3);
		NRF_LOG_INFO("success: %d", success);
		vTaskDelay(200);
		send_handshake();
		NRF_LOG_INFO("Main Com Task: Init complete");
		while (true) 
		{
			if (xSemaphoreTake(xCommandReadyBSem, portMAX_DELAY) == pdTRUE)
			{ //TODO Is this semaphore used anywhere else?
				NRF_LOG_INFO("NEW Command in");
				// We have a new command from the server, copy it to the memory
				//vTaskSuspendAll ();       // Temporarily disable context switching
				//taskENTER_CRITICAL();
				command_in = message_in;
				//taskEXIT_CRITICAL();
				//xTaskResumeAll ();      // Enable context switching
				switch (command_in.type) 
				{
				case TYPE_CONFIRM:
					NRF_LOG_INFO("MESSAGE WAS: TYPE_CONFIRM");
					//taskENTER_CRITICAL();
					gHandshook = true; // Set start flag true
					//taskEXIT_CRITICAL();

					break;
				
				case TYPE_PING:
					NRF_LOG_INFO("MESSAGE WAS: TYPE_PING");
					send_ping_response();
					break;
				
				case TYPE_ORDER:
					NRF_LOG_INFO("MESSAGE WAS: TYPE_ORDER x: %d,Y:%d",command_in.message.order.x,command_in.message.order.y);
					Setpoint.x = command_in.message.order.x;
					Setpoint.y = command_in.message.order.y;
					/* Relay new coordinates to position controller */
					xQueueSend(poseControllerQ, &Setpoint, 100);
					break;
				
				case TYPE_PAUSE:
					NRF_LOG_INFO("MESSAGE WAS: TYPE_PAUSE");
					// Stop sending update messages
					//taskENTER_CRITICAL();
					gPaused = true;
					//taskEXIT_CRITICAL();
					// Stop controller
					Setpoint.x = 0;
					Setpoint.y = 0;
					xQueueSend(poseControllerQ, &Setpoint, 100);
					break;
				
				case TYPE_UNPAUSE:
					NRF_LOG_INFO("MESSAGE WAS: TYPE_UNPAUSE");
					//taskENTER_CRITICAL();
					gPaused = false;
				// taskEXIT_CRITICAL();
					break;
				
				case TYPE_FINISH:
					NRF_LOG_INFO("MESSAGE WAS: TYPE_FINISH");
					//taskENTER_CRITICAL();
					gHandshook = false;
					//taskEXIT_CRITICAL();
					break;

				default:
					NRF_LOG_INFO("message:case default No type %d", command_in.type);
				}
			} 
		}
	}	
	else // If USEBLUETOOTH = false, uses NRF52840 Dongle with thread and C++ server
	{
		uint8_t message[8] = {0};
		int16_t oldwaypoint[2] = {0};
		int16_t waypoint[2] = {0};
		bool isValidWaypoint;
		
		while(true){
			i2c_reciveNOADDR(I2C_DEVICE_DONGLE, message, 8);
			
			if(USE_NEW_SERVER){
				switch(message[1]){ // message[0] is the adress of the dongle, 0x72, if the received data is new
			
					case START_POSITION: 
					
						if(xSemaphoreTake(xPoseMutex, 20) == pdTRUE){
							set_position_estimate_x(*((int16_t*)&message[2]));
							set_position_estimate_y(*((int16_t*)&message[4]));
							set_position_estimate_heading(*((int16_t*)&message[6])*DEG2RAD); // Will maybe be unnecessary to implement
							//TODO This may need some condition variables so the scanning and stuff dont start before this is received.
							xSemaphoreGive(xPoseMutex);
						}else{
							NRF_LOG_INFO("xPoseMutex not available!");
						}
					
						break;
					
			
					case NEW_WAYPOINT_NEW_SERVER:  
						oldwaypoint[0] = waypoint[0];
						oldwaypoint[1] = waypoint[1];
						waypoint[0] = *((int16_t*)&message[2]); //WP_X in message[2] for new server
						waypoint[1] = *((int16_t*)&message[4]);	//WP_Y ins message[4] for new server
						
						xSemaphoreTake(xPoseMutex, 20);
						thetahat = get_position_estimate_heading();
						xhat = get_position_estimate_x();
						yhat = get_position_estimate_y();
						xSemaphoreGive(xPoseMutex);
						
						if(VALIDATE_WAYPOINT){
							int16_t wpAngle = (atan2(waypoint[1]-yhat, waypoint[0]-xhat) - thetahat)*RAD2DEG;	// WpAngle in robot-frame
							isValidWaypoint = validWaypoint(wpAngle);
						}else{
							isValidWaypoint = true;
						}
					
						if(((oldwaypoint[0] != waypoint[0]) || (oldwaypoint[1] != waypoint[1])) && isValidWaypoint){ 
							sendScanBorder(); // used to signalize that the robot has a new waypoint to the server.
							struct sCartesian target = {waypoint[0]/10, waypoint[1]/10};
							xQueueSend(poseControllerQ, &target, 100);
						}
						break;
						
					case UPDATE_POSITION:
						break;
					default:
					
						break;
				}
			}else{ // There is only one message coming from the old C++ server, which is a new waypoint
				oldwaypoint[0] = waypoint[0];
				oldwaypoint[1] = waypoint[1];
				waypoint[0] = *((int16_t*)&message[1]);
				waypoint[1] = *((int16_t*)&message[3]);
				
				xSemaphoreTake(xPoseMutex, 20);
				thetahat = get_position_estimate_heading();
				xhat = get_position_estimate_x();
				yhat = get_position_estimate_y();
				xSemaphoreGive(xPoseMutex);
				
				if(VALIDATE_WAYPOINT){
					int16_t wpAngle = (atan2(waypoint[1]-yhat, waypoint[0]-xhat) - thetahat)*RAD2DEG;
					isValidWaypoint = validWaypoint(wpAngle);
				}else{
					isValidWaypoint = true;
				}
				
				
			
				if(((oldwaypoint[0] != waypoint[0]) || (oldwaypoint[1] != waypoint[1])) && isValidWaypoint){ //Add the line that discardes the waypoint.
					sendScanBorder(); // used to signalize that the robot has a new waypoint to the server.
					struct sCartesian target = {waypoint[0]/10, waypoint[1]/10};
					xQueueSend(poseControllerQ, &target, 100);
				}
			
			}
			vTaskDelay(500);
		} 
	}
}