/************************************************************************
* File:			robot_config.h
* Author:		Eivind Jølsgard	
* Date:			Autumn 2020					
* 
************************************************************************/

#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "nrfx_saadc.h"

//Select robot to use
//This changes controller parameters, max duty for motors etc. 

//TODO Add more config parameters to this file from other files


/* **************************************************************************************************************
 * Robot selection
 *
 ****************************************************************************************************************/

//#define NRF_ROBOT_1
#define NRF_ROBOT_2
//#define NRF_ROBOT_3

/* **************************************************************************************************************
 * Logging via printf
 *
 ****************************************************************************************************************/
#define LOG_MEASUREMENT_DATA 0
#define PRINT_DEBUG 0
#define PRINT_DEBUG_IR 0
#define LOG_MOTOR_SPEED_CONTROLLER 0
#define LOG_ROBOT_POSITION_CONTROLLER 0

/* **************************************************************************************************************
 * Select controller and estimator
 *
 ****************************************************************************************************************/
#define USE_NEW_ESTIMATOR 1
#define USE_SPEED_CONTROLLER 1


/* **************************************************************************************************************
 * Robot name for server connection
 *
 ****************************************************************************************************************/

#if defined (NRF_ROBOT_1)
	#define ROBOT_NAME "NRF_1"
	#define ROBOT_NAME_LENGTH 5

#elif defined (NRF_ROBOT_2)
	#define ROBOT_NAME "NRF_2"
	#define ROBOT_NAME_LENGTH 5

#elif defined (NRF_ROBOT_3)
	#define ROBOT_NAME "NRF_3"
	#define ROBOT_NAME_LENGTH 5

#else 						//Default parameters
	#define ROBOT_NAME "NRF"
	#define ROBOT_NAME_LENGTH 3
#endif

/* **************************************************************************************************************
 * Communication
 *
 ****************************************************************************************************************/

#define USEBLUETOOTH 1 			// For switching between nRF51 bluetooth dongle and NRF52840 Thread dongle
#define USE_NEW_SERVER 0		// For switching between Grindvik and Mullins' server versions
#define VALIDATE_WAYPOINT 0		// If false, all waypoints are processed. If true, waypoints inside collision sectors are discarded.



/******** I2C dongle********/
#define I2C_DEVICE_DONGLE         0x72



/* **************************************************************************************************************
 * IR sensor configuration
 *
 ****************************************************************************************************************/

///IR distance in millimeters is computed as (measurement/IR_DIVIDER) ^ IR_EXPONENT 
	//Where measurement is the measured value and the rest are parameters configured below
/*
IR sensor directions
IR1 TOP LEFT      
IR2 TOP RIGHT     
IR3 BOTTOM LEFT   
IR4 BOTTOM RIGHT

*/

#define NUM_DIST_SENSORS			4

#if defined (NRF_ROBOT_1)

	//Connections 
	#define IR_SENSOR_1_ANALOG_INPUT 	NRF_SAADC_INPUT_AIN7
	#define IR_SENSOR_2_ANALOG_INPUT 	NRF_SAADC_INPUT_AIN5
	#define IR_SENSOR_3_ANALOG_INPUT 	NRF_SAADC_INPUT_AIN6
	#define IR_SENSOR_4_ANALOG_INPUT 	NRF_SAADC_INPUT_AIN4
	//Sensor calibration
	#define IR_MAX_DETECT_DISTANCE_MM 	300				//Update to 300? Lead to trouble with collision detection
	#define COLLISION_THRESHOLD_MM		190	
	#define IR_SENSOR_1_DIVIDER			54497.0
	#define IR_SENSOR_1_EXPONENT		-0.8693
	#define IR_SENSOR_2_DIVIDER			46158.25
	#define IR_SENSOR_2_EXPONENT		-0.84968
	#define IR_SENSOR_3_DIVIDER			61845.11
	#define IR_SENSOR_3_EXPONENT		-0.88626
	#define IR_SENSOR_4_DIVIDER			41548.35
	#define IR_SENSOR_4_EXPONENT		-0.83468


#elif defined (NRF_ROBOT_2)
	//Connections 
	#define IR_SENSOR_1_ANALOG_INPUT 	NRF_SAADC_INPUT_AIN7
	#define IR_SENSOR_2_ANALOG_INPUT 	NRF_SAADC_INPUT_AIN5
	#define IR_SENSOR_3_ANALOG_INPUT 	NRF_SAADC_INPUT_AIN6
	#define IR_SENSOR_4_ANALOG_INPUT 	NRF_SAADC_INPUT_AIN4
	//Sensor calibration
	#define IR_MAX_DETECT_DISTANCE_MM 	300				//Update to 300? Lead to trouble with collision detection
	#define COLLISION_THRESHOLD_MM		200
	#define IR_SENSOR_1_DIVIDER			41215.58
	#define IR_SENSOR_1_EXPONENT		-0.8334
	#define IR_SENSOR_2_DIVIDER			47386.39
	#define IR_SENSOR_2_EXPONENT		-0.8512
	#define IR_SENSOR_3_DIVIDER			60542.74
	#define IR_SENSOR_3_EXPONENT		-0.884
	#define IR_SENSOR_4_DIVIDER			37765
	#define IR_SENSOR_4_EXPONENT		-0.81929

	//TODO Add calibrated values for nRF3
#else 						//Default parameters

	#define COLLISION_THRESHOLD_MM		600
	//Connections 
	#define IR_SENSOR_1_ANALOG_INPUT 	NRF_SAADC_INPUT_AIN7
	#define IR_SENSOR_2_ANALOG_INPUT 	NRF_SAADC_INPUT_AIN5
	#define IR_SENSOR_3_ANALOG_INPUT 	NRF_SAADC_INPUT_AIN6
	#define IR_SENSOR_4_ANALOG_INPUT 	NRF_SAADC_INPUT_AIN4
	//Sensor calibration
	#define IR_MAX_DETECT_DISTANCE_MM 	800
	#define IR_SENSOR_1_DIVIDER			16250.0
	#define IR_SENSOR_1_EXPONENT		-1.1
	#define IR_SENSOR_2_DIVIDER			16250.0
	#define IR_SENSOR_2_EXPONENT		-1.1
	#define IR_SENSOR_3_DIVIDER			16250.0
	#define IR_SENSOR_3_EXPONENT		-1.1
	#define IR_SENSOR_4_DIVIDER			16250.0
	#define IR_SENSOR_4_EXPONENT		-1.1

#endif

/* **************************************************************************************************************
 * Anti collision parameters
 *
 ****************************************************************************************************************/


#define COLLISION_THRESHOLD_CM		(COLLISION_THRESHOLD_MM/10)
#define COLLISION_SECTOR_OFFSET		60		//[degrees] (to each side)

/* **************************************************************************************************************
 * Motor, wheel and encoder config
 *
 ****************************************************************************************************************/
typedef enum {
  FORWARD,
  BACKWARD
} MOTOR_DIRECTION;



	/******* Motors *********/
	#if defined (NRF_ROBOT_3)
		#define MOTOR_PWM_MAX_DUTY		50  //Allow 50% duty for 6V motors	
	#else 						//Default parameters
		#define MOTOR_PWM_MAX_DUTY		100  //Allow 100% duty	
	#endif


	/****** Wheels ******/
	#define WHEEL_DIAMETER_MM			65   //Updated 09.10.2020
	#define WHEEL_CIRCUMFERENCE_MM		204.5  //Updated 09.10.2020

	/****** Encoders ******/
	#define ENCODER_TICKS_PER_ROT        224  //     224  //Updated 09.10.2020
	
	#if defined (NRF_ROBOT_1)
		#define ENCODER_PIN_LEFT_2_HIGH_DIRECTION FORWARD			//Direction of encoder, is sensor 2 high or low on rising edge of sensor 1?
		#define ENCODER_PIN_RIGHT_2_HIGH_DIRECTION BACKWARD			//change if robot move forward and encoder ticks are negative

	#elif defined (NRF_ROBOT_2)
		#define ENCODER_PIN_LEFT_2_HIGH_DIRECTION FORWARD
		#define ENCODER_PIN_RIGHT_2_HIGH_DIRECTION BACKWARD

	#elif defined (NRF_ROBOT_3)
		#define ENCODER_PIN_LEFT_2_HIGH_DIRECTION BACKWARD
		#define ENCODER_PIN_RIGHT_2_HIGH_DIRECTION FORWARD

	#else 						//Default parameters
		#define ENCODER_PIN_LEFT_2_HIGH_DIRECTION FORWARD
		#define ENCODER_PIN_RIGHT_2_HIGH_DIRECTION BACKWARD

	#endif


										
	//Parameters below follow prevoius config
										//Brackets around the division below lead to wheel_factor_mm = 0
	#define WHEEL_FACTOR_MM				1.0*WHEEL_CIRCUMFERENCE_MM/ENCODER_TICKS_PER_ROT       /* 297 [encoderticks/wheelrotation] = 11 [encoderticks/motorRotation] * 27 [gearboxRatio] */ 
	
	#define ENCODER_PIN_LEFT_2_LOW_DIRECTION !ENCODER_PIN_LEFT_2_HIGH_DIRECTION
	#define ENCODER_PIN_RIGHT_2_LOW_DIRECTION !ENCODER_PIN_RIGHT_2_HIGH_DIRECTION

/* **************************************************************************************************************
 * Motor Speed Controller PID regulator
 *
 ****************************************************************************************************************/

#define MOTOR_SPEED_CTRL_TASK_DELAY_TIME 100		//Period of the speed controller

#define MOTOR_SPEED_CTRL_ERROR_INTEGRAL_BOUNDARY 20000 //Prevent integral windup
#define MOTOR_SPEED_CTRL_MAX_OUTPUT 100 //max 100% duty
#define MOTOR_SPEED_CTRL_MAX_OUTPUT_CHANGE_SPEED 10
#define MOTOR_SPEED_CTRL_MIN_OUTPUT 10



#if defined (NRF_ROBOT_1)
	#define MOTOR_SPEED_CTRL_LEFT_FEED_FORWARD 0	//10
	#define MOTOR_SPEED_CTRL_RIGHT_FEED_FORWARD 0 	//10

	#define MOTOR_SPEED_CTRL_LEFT_K_P 100	//800
	#define MOTOR_SPEED_CTRL_LEFT_K_I 150.0
	#define MOTOR_SPEED_CTRL_LEFT_K_D 0.0005	//0.06

	#define MOTOR_SPEED_CTRL_RIGHT_K_P 100
	#define MOTOR_SPEED_CTRL_RIGHT_K_I 150.0	
	#define MOTOR_SPEED_CTRL_RIGHT_K_D 0.0005	//0.06

#elif defined (NRF_ROBOT_2)
	#define MOTOR_SPEED_CTRL_LEFT_FEED_FORWARD 0 //10
	#define MOTOR_SPEED_CTRL_RIGHT_FEED_FORWARD 0 //10

	#define MOTOR_SPEED_CTRL_LEFT_K_P 100	//800
	#define MOTOR_SPEED_CTRL_LEFT_K_I 150.0
	#define MOTOR_SPEED_CTRL_LEFT_K_D 0.0005	//0.06

	#define MOTOR_SPEED_CTRL_RIGHT_K_P 100
	#define MOTOR_SPEED_CTRL_RIGHT_K_I 150.0	
	#define MOTOR_SPEED_CTRL_RIGHT_K_D 0.0005	//0.06
#elif defined (NRF_ROBOT_3)
	#define MOTOR_SPEED_CTRL_LEFT_FEED_FORWARD 0 //5
	#define MOTOR_SPEED_CTRL_RIGHT_FEED_FORWARD 0 //5

	#define MOTOR_SPEED_CTRL_LEFT_K_P 100	//800
	#define MOTOR_SPEED_CTRL_LEFT_K_I 150.0
	#define MOTOR_SPEED_CTRL_LEFT_K_D 0.0005	//0.06

	#define MOTOR_SPEED_CTRL_RIGHT_K_P 100
	#define MOTOR_SPEED_CTRL_RIGHT_K_I 150.0	
	#define MOTOR_SPEED_CTRL_RIGHT_K_D 0.0005	//0.06

#else 						//Default parameters
	#define MOTOR_SPEED_CTRL_LEFT_FEED_FORWARD 15
	#define MOTOR_SPEED_CTRL_RIGHT_FEED_FORWARD 15

	#define MOTOR_SPEED_CTRL_LEFT_K_P 0.1
	#define MOTOR_SPEED_CTRL_LEFT_K_I 10
	#define MOTOR_SPEED_CTRL_LEFT_K_D 1

	#define MOTOR_SPEED_CTRL_RIGHT_K_P 0.1
	#define MOTOR_SPEED_CTRL_RIGHT_K_I 10
	#define MOTOR_SPEED_CTRL_RIGHT_K_D 1

#endif



/* **************************************************************************************************************
 * Robot Position Controller PID regulator
 *
 ****************************************************************************************************************/

#define ROBOT_POSITION_CTRL_TASK_DELAY_TIME 50




#define ROBOT_DISTANCE_CTRL_MAX_OUTPUT 50 //max 100
#define ROBOT_DISTANCE_CTRL_MAX_OUTPUT_CHANGE_SPEED 10
#define ROBOT_DISTANCE_CTRL_MIN_OUTPUT 10
#define ROBOT_DISTANCE_CTRL_ERROR_INTEGRAL_BOUNDARY 20000 //Prevent integral windup

#define ROBOT_HEADING_CTRL_MAX_OUTPUT 50 //max 100
#define ROBOT_HEADING_CTRL_MAX_OUTPUT_CHANGE_SPEED 10
#define ROBOT_HEADING_CTRL_MIN_OUTPUT 10
#define ROBOT_HEADING_CTRL_ERROR_INTEGRAL_BOUNDARY 200000 //Prevent integral windup


#define ROBOT_HEADING_WHILE_DRIVING_CTRL_MAX_OUTPUT 100 //max 100% duty
#define ROBOT_HEADING_WHILE_DRIVING_CTRL_MAX_OUTPUT_CHANGE_SPEED 10
#define ROBOT_HEADING_WHILE_DRIVING_CTRL_MIN_OUTPUT 10
#define ROBOT_HEADING_WHILE_DRIVING_CTRL_ERROR_INTEGRAL_BOUNDARY 20000 //Prevent integral windup


#define ROBOT_DISTANCE_CTRL_K_P 400.0
#define ROBOT_DISTANCE_CTRL_K_I 0.0
#define ROBOT_DISTANCE_CTRL_K_D 0.0

#define ROBOT_HEADING_WHILE_DRIVING_CTRL_K_P 300.0
#define ROBOT_HEADING_WHILE_DRIVING_CTRL_K_I 0.0
#define ROBOT_HEADING_WHILE_DRIVING_CTRL_K_D 0.0

#define ROBOT_HEADING_CTRL_K_P 60
#define ROBOT_HEADING_CTRL_K_I 0.00
#define ROBOT_HEADING_CTRL_K_D 0.000




/* **************************************************************************************************************
 * PHYSICAL CONSTANTS 
 * If the robot is changed these need to be changed 
 * Some of these will be sent to server during the start-up-handshake   
 *
 ***********************************************************************/


//#if defined (NRF_ROBOT_1)

//#elif defined (NRF_ROBOT_2)

//#elif defined (NRF_ROBOT_3)

//#else 				
	#define WHEELBASE_MM				157  // Updated 09.10.2020 /* Length between wheel centers  */
	#define ROBOT_TOTAL_WIDTH_MM		186  // Updated 09.10.2020 /* From outer rim to outer rim   */
	#define ROBOT_TOTAL_LENGTH_MM		194  /* Updated 09.10.2020 From front to aft, total	     */
	#define ROBOT_AXEL_OFFSET_MM		0    /* From center of square	     */
	#define SENSOR_TOWER_OFFSET_X_MM	0    /* From center of square         */
	#define SENSOR_TOWER_OFFSET_Y_MM	0    /* From center of square		     */
	#define SENSOR_OFFSET_RADIUS_MM		23   /* From center of tower		     */
	#define ROBOT_DEADLINE_MS			200  /* Interval between measurements	     */



                                    

//#endif


#define SENSOR1_HEADING_DEG			0    /* Sensor angle relative to body	     */
#define SENSOR2_HEADING_DEG			90
#define SENSOR3_HEADING_DEG			180
#define SENSOR4_HEADING_DEG			270




/*
#if defined (NRF_ROBOT_1)

#elif defined (NRF_ROBOT_2)

#elif defined (NRF_ROBOT_3)

#else 						//Default parameters

#endif
*/

#endif //ROBOT_CONFIG_H