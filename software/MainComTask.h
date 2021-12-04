/************************************************************************/
// File:            MainComTask.h                                      //
// Author:          Stenset, Spring 2020                                //
// Purpose:                                                             //
//                                                                      //
/************************************************************************/

#ifndef MAIN_COM_TASK_H
#define MAIN_COM_TASK_H

#include <stdint.h>
#include "defines.h"



void vMainCommunicationTask(void *pvParameters);






/* new C++ server message codes */
#define START_POSITION				1
#define NEW_WAYPOINT_NEW_SERVER		2
#define UPDATE_POSITION				3








#endif /* MAIN_COM_TASK_H */