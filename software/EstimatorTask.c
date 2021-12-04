/************************************************************************/
// File:            EstimatorTask.c                                     //
// Author:																//
// Purpose:                                                             //
//                                                                      //
/************************************************************************/

//#include "MPU6050.h"
#include "ICM_20948.h"
#include "defines.h"
#include "freeRTOS.h"
#include "functions.h"
#include "mag3110.h"
#include "math.h"
#include "nrf_log.h"
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

#include "encoder_with_counter.h"
//#include "encoder.h"
#include "positionEstimate.h"

int8_t gyroBiasGuard = 1;

IMU_reading_t gyro;
IMU_reading_t accel;
position_estimate_t kf_state;


/* Pose estimator task */
void vMainPoseEstimatorTask(void *pvParameters) {
	NRF_LOG_INFO("mainPoseEstimatorTask: initializing");
	printf("USING OLD ESTIMATOR");
	int count = 0;
	//float period_in_S = PERIOD_ESTIMATOR_MS / 1000.0;
	float accelXoffset = 0;
	float accelYoffset = 0 ;
	float gyroOffset = 0.0;
	//float compassOffset = 0.0;

	//int KFcounter = 0;
	//int KFheading = 0;
	//int time = 0;
	// float variance_gyro = 0.0482f;                                             // [rad] calculated offline, see report
	//float variance_encoder = (2.0f * WHEEL_FACTOR_MM) / (WHEELBASE_MM / 2.0f); // approximation, 0.0257 [rad]

	// float variance_gyro_encoder = (variance_gyro + variance_encoder) * period_in_S; // (Var gyro + var encoder) * timestep

	// For microsd-logging
	//char gyrodata[20];
	//int headingTime = 0;
	//int headingLogCounter = 0;
	//bool headingLogDone = false;

	//uint8_t robotMovement = moveStop;

	//char str2[20];
	//char str3[20];

	// test variabler
	float gyrZ = 0;

	// Initialise the xLastWakeTime variable with the current time.
	TickType_t xLastWakeTime = xTaskGetTickCount();
	TickType_t ticks_since_startup = xTaskGetTickCount();
	//TickType_t xLastWakeTime2 = xLastWakeTime;
	
	float Z[7];
	kf_init(7,8);


	float gyroSum = 0;
	float gyroLimit = 0.1;

	NRF_LOG_INFO("mainPoseEstimatorTask: init complete");
	
	while (true) {
		// Loop
		/*
		if (gHandshook != true){
			vTaskDelay(4000);
		}else
		{
			vTaskDelayUntil(&xLastWakeTime, 40);
		}
		*/
		TickType_t ticks_since_startup_prev = ticks_since_startup;
		vTaskDelayUntil(&xLastWakeTime, 40); //This delays for 40 tics, not ms. However, FreeRTOS uses CPU clock hz = 1000
		count+=1;
		//The constant portTICK_PERIOD_MS can be used to calculate real time from the tick rate – with the resolution of one tick period
		//TickType_t ticks_since_startup_prev = ticks_since_startup;
		ticks_since_startup = xTaskGetTickCount();
		float delta_t = (ticks_since_startup - ticks_since_startup_prev)*1.0 / configTICK_RATE_HZ;
		if (gHandshook) { // gHandshook set to true in IMU calibration further down
			//int16_t leftWheelTicks = 0;
			//int16_t rightWheelTicks = 0;
			float dRobot = 0;
			float dTheta = 0;
		
			/* DATA COLLECTION*/
			// Get encoder data, protect the global tick variables
			encoder_with_counter_get_ticks_since_last_time();

        	encoderTicks encoder_ticks, encoder_ticks_temp;
        	encoder_ticks.left = 0;
        	encoder_ticks.right = 0;

        	while(xQueueReceive(encoderTicksToEstimatorTaskQ, &encoder_ticks_temp, 1) == pdTRUE)
        	{
            	//NRF_LOG_INFO("Read ticks");
            	encoder_ticks.left += encoder_ticks_temp.left;
            	encoder_ticks.right += encoder_ticks_temp.right;
        	}
			
			float dLeft = (float)(encoder_ticks.left * WHEEL_FACTOR_MM);	// Distance left wheel has traveled since last sample
			float dRight = (float)(encoder_ticks.right * WHEEL_FACTOR_MM);	// Distance right wheel has traveled since last sample
			dRobot = (dLeft + dRight) / 2;

			//Alternative to get heading using encoders
			/*
			if (RightMotorDirection == 1 && LeftMotorDirection == -1){
				dTheta = (dRight+dLeft)/(2*WHEELBASE_MM);
			} else if (RightMotorDirection == -1 && LeftMotorDirection == 1) {
				dTheta = -(dRight+dLeft)/(2*WHEELBASE_MM);
			} else {
				dTheta = (dRight - dLeft) / WHEELBASE_MM;
			}
			*/
			dTheta = (dRight - dLeft) / WHEELBASE_MM;					// Get angle from encoders, dervied from arch of circles formula NB! When turning in place this will only give 0
			if(PRINT_DEBUG)printf("\nLeftDir = %d", LeftMotorDirection);
			if(PRINT_DEBUG)printf("\nRightDir = %d", RightMotorDirection);
			if(PRINT_DEBUG)printf("\ndTheta = %.2f", dTheta);
			//NRF_LOG_INFO("dTheta: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(dTheta));
		
		
			// Get IMU data:
			if (IMU_new_data()){
				IMU_read();                      
				gyro = IMU_getGyro();
				accel = IMU_getAccel();
				accel.x -=  accelXoffset;
				accel.y -=  accelYoffset;
				gyrZ = gyro.z - gyroOffset;		// [deg/sec]
			
				if(fabs(gyrZ) < gyroLimit){		// Compensate for noise from the gyro
					gyrZ = 0.0;
				}
			} else {
				//NRF_LOG_INFO("No new data from IMU");
				gyrZ = 0.0;
			}
		
			/* Alternative to Kalman Filter for heading. */
			gyroSum += gyrZ*DEG2RAD*(float)(40.0/1000.0);
		
			
			
			/* Used to check timing issues */
			//TickType_t tickdiff =xTaskGetTickCount()- xLastWakeTime2;
			//float delta_t = tickdiff*1.0/configTICK_RATE_HZ;
			//xLastWakeTime2 = xTaskGetTickCount(); 
			//float delta_t = tickdiff*1.0/configTICK_RATE_HZ;
			//NRF_LOG_INFO("ET:%u",(uint32_t) tickdiff);
		
		
		
			//get MAGNETOMETER data:
			// MAG_reading_t mag = mag_read();
		
			/* SENSOR DATA collected*/ 
		
			/*step kalman filter*/       
			float cosTheta = cos(get_position_estimate_heading());
			float sinTheta = sin(get_position_estimate_heading());  

			// inputs in meters and radians
			
			Z[0]=dRobot*cosTheta*0.025;		// encoder speed, x-axis // (distance /(40/1000))/1000
			Z[1]=(accel.x*cosTheta - accel.y*sinTheta)*9.81;//*1.03;		// +accel.y*sinTheta; //accelerometer  readings x-axis

			Z[2]=dRobot*sinTheta*0.025;		// encoder speed y-axis
			Z[3]=(accel.x*sinTheta + accel.y*cosTheta)*9.81;//*1.03;		//TODO +accel.y*cosTheta; //accellerometer reading y-axis

			Z[4]=dTheta*25.0;				// encoder rotation speed (theta_hat)
			Z[5]=gyrZ*DEG2RAD;				// Changed to decouple the gyro from the Kalman filter.gyrZ*DEG2RAD; //gyro rotation speed (theta_hat)
			Z[6]=0;							// magnetometer HEADING( unused(come up with clever way to use (static field detection ?)also magnetometer sample rate is lower than filter))  
			

			if((fabs(accel.x) < 0.05) | (fabs(accel.y) < 0.05) | (dRobot == 0)){ //Was if(dRobot == 0)
				//kf_setGyroVar(1);//if wheels arent turning have less trust in gyro
				Z[1]=0.0;	//if wheels arent turning dont trust accelerometer  
				Z[3]=0.0;
				accelXoffset += (accel.x/1000);		//try to correct sensor offset drifting
				
			}else{
				//kf_setGyroVar(0.0134);
				if(fabs(Z[5]-Z[4])>0.2*Z[5]){ kf_setEncoderVar(1);}		// if gyro and encoders mismatch (stuck?) trust gyro
				else {kf_setEncoderVar(0.03);}
			}


			kf_step(Z);//step the filter  
			kf_state = kalmanGetState();//extract state


			// Update global pose
			vFunc_Inf2pi(&(kf_state.heading)); // Places angles inside -pi to pi
			vFunc_Inf2pi(&(gyroSum));
			
			/*
			if(count > 40){
				
				headingTime = (xTaskGetTickCount());
				sprintf(str2, "%d, %d, %d", (int)(kf_state.heading*RAD2DEG), (int)(gyroIntegral*RAD2DEG), (int)(headingTime));
				NRF_LOG_INFO("%s", str2);
		
				count = 0;
			}
			
			*/
			xSemaphoreTake(xPoseMutex, 15);

			set_position_estimate_heading(kf_state.heading);					// previously: gTheta_hat = kf_state.heading;  replaced with: gTheta_hat = gyroSum;
			set_position_estimate_x((kf_state.x));		
			set_position_estimate_y( (kf_state.y));

			//gLeft = dLeft;
			//gRight = dRight;
			xSemaphoreGive(xPoseMutex);

			if(PRINT_DEBUG)NRF_LOG_INFO("KF_x: " NRF_LOG_FLOAT_MARKER "\t\r\n", NRF_LOG_FLOAT(1000*kf_state.x));
			if(PRINT_DEBUG)NRF_LOG_INFO("KF_y: " NRF_LOG_FLOAT_MARKER "\t\r\n", NRF_LOG_FLOAT(1000*kf_state.y));
			if(PRINT_DEBUG)NRF_LOG_INFO("KF_heading: " NRF_LOG_FLOAT_MARKER "\t\r\n", NRF_LOG_FLOAT(kf_state.heading));
			if(PRINT_DEBUG)NRF_LOG_INFO("GYroSum: " NRF_LOG_FLOAT_MARKER "\t\r\n", NRF_LOG_FLOAT(gyroSum));
/*			if(PRINT_DEBUG)printf("x_hat: %d\n", gX_hat);
			if(PRINT_DEBUG)printf("y_hat: %d\n", gY_hat);
			if(PRINT_DEBUG)printf("theta_hat %d\n", gTheta_hat);
*/
			if(LOG_MEASUREMENT_DATA){
				//Accel.x and Gyrz is treated with offset
				//double time_since_startup = ticks_since_startup * 1.0 / configTICK_RATE_HZ;
				printf("\n%f %f %f %ld %ld %f %f %f",delta_t,accel.x,gyrZ,encoder_ticks.left,encoder_ticks.right,kf_state.heading,kf_state.x,kf_state.y);
			}
			// Send semaphore to controller
			xSemaphoreGive(xControllerBSem);
		}else{
			// IMU calibration
			if(gyroBiasGuard){//neccesary to complete this part only once
				NRF_LOG_INFO("Gyrobiasguard == 1");
				// Not connected, getting heading and gyro bias
				//char str4[20];
				uint16_t i;
				uint16_t samples = 300;
				float gyroF = 0;
				float accelFX = 0;
				float accelFY = 0;
				int fails = 0;
				int sucsess = 0;
				NRF_LOG_INFO("IMU calib init done");
				vTaskDelay(150);//use delay so we dont write before i2c is initialized
				NRF_LOG_INFO("Enter IMU calibration");
				for (i = 0; i < samples; i++){
					IMU_read(); //needs to be called to get new gyro data
					gyro = IMU_getGyro();
					accel = IMU_getAccel();
					gyroF += gyro.z;
					accelFX += accel.x;
					accelFY += accel.y;

					vTaskDelay(40);
					/*
					sprintf(str4,"cal F:%i S:%i",fails,sucsess);
					display_text_on_line(4,str4);
					*/
					sucsess++;
			
					while (!IMU_new_data()){
						vTaskDelay(20); // wait for new data
						fails++;
						NRF_LOG_INFO("Waiting for new IMU data");
						/*
						sprintf(str4,"cal F:%i S:%i",fails,sucsess);
						display_text_on_line(4,str4); 
						*/
					}
				}
				//NRF_LOG_INFO("aFX: %i aFY: %i gF: %i", gyroF, accelFX, gyroOffset);
				NRF_LOG_INFO("Calib.i: %i", i);
				gyroOffset = gyroF / (float)samples;
				accelXoffset = accelFX/(float)samples;
				accelYoffset = accelFY /(float)samples;
				gyroBiasGuard = 0;
				NRF_LOG_INFO("gyroOffset: " NRF_LOG_FLOAT_MARKER "\t\t\r\n", NRF_LOG_FLOAT(gyroOffset));
				NRF_LOG_INFO("accelXOffset: " NRF_LOG_FLOAT_MARKER "\t\t\r\n", NRF_LOG_FLOAT(accelXoffset));
				NRF_LOG_INFO("accelYOffset: " NRF_LOG_FLOAT_MARKER "\t\t\r\n", NRF_LOG_FLOAT(accelYoffset));
				//NRF_LOG_INFO("aX: %i aY: %i g: %i", accelXoffset, accelYoffset, gyroOffset);

				if(!USEBLUETOOTH){
					gHandshook = true;
				}
			}

		}

	} // While(1) end

}