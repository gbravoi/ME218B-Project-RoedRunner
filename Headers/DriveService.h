/****************************************************************************
Header File for Drive Service
****************************************************************************/
#ifndef DriveService_H
#define DriveService_H


#include <stdint.h>
#include <stdbool.h>

#include "ES_Events.h"

bool InitDriveService(uint8_t Priority);
ES_Event_t RunDriveService(ES_Event_t ThisEvent);
bool PostDriveService(ES_Event_t ThisEvent);

void Drive_SetDrivePosSpeed(int16_t Speed);
void Drive_SetRotPosSpeed(int16_t Speed);

// ISRs
void DriveService_PIctrlISR(void);
  
#endif /*DriveService_H_H*/
