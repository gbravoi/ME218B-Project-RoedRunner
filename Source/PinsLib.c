/****************************************************************************
 Module
   PinsLib.c

 Revision
   1.

 Description
   Contains functions for get/set information from/to pins
   -Read team selection
   -Set mining status

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
02/24/2020      gab      initial release

****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
// the common headers for C99 types
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// the headers to access the GPIO subsystem
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"

// Timer interrupts
#include "inc/hw_timer.h"
#include "inc/hw_nvic.h"

//Framework files
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Port.h"

#include "gpio.h"
#include "PinsLib.h"


/*---------------------------- Module Defines ----------------------------*/
#define StatusPin "PF2"
#define TeamPin "PF1"


/*---------------------------- Module Functions ---------------------------*/
/*---------------------------- Module Variables ---------------------------*/

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
    InitPins

 Parameters
   None

 Returns
   None

 Description
   Initialization functions of all pins managed in this library
 Notes

 Author
   G.Bravo
****************************************************************************/

void InitPins(void){
GPIO_ConfigPin(StatusPin, Output);
  GPIO_ConfigPin(TeamPin, Input);
}

/****************************************************************************
 Function
    TurnOnMiningStatus and TurnOffMiningStatus

 Parameters
   None

 Returns
   None

 Description
   Turns on/off mining status
 Notes

 Author
   G.Bravo
****************************************************************************/
void TurnOnMiningStatus(void){
GPIO_SetPin(StatusPin, High);
}

void TurnOffMiningStatus(void){
GPIO_SetPin(StatusPin, Low);
}

/****************************************************************************
 Function
    ReadTeam

 Parameters
   None

 Returns
   Team_t CHK = 0 or GHI = 1

 Description
   get the team selection from the switch.
 Notes

 Author
   G.Bravo
****************************************************************************/
Team_t ReadTeam(void){
return ((Team_t)GPIO_ReadPin(TeamPin));

}


/***************************NOTES**********************************
1. Remember initialization in GP_MasterGameHSM
*/
