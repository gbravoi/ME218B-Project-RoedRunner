/****************************************************************************
 Module
   TemplateFSM.c

 Revision
   1.0.1

 Description
   This is a template file for implementing flat state machines under the
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/15/12 11:12 jec      revisions for Gen2 framework
 11/07/11 11:26 jec      made the queue static
 10/30/11 17:59 jec      fixed references to CurrentEvent in RunTemplateSM()
 10/23/11 18:20 jec      began conversion from SMTemplate.c (02/20/07 rev)
 02/28/2020		ram		 converted to use for bumper FSM
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "BumperFSM.h"

// the headers to access the GPIO subsystem
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"

#include "gpio.h"
#include "EncoderLib.h"
#include "GP_MasterGameHSM.h"
/*----------------------------- Module Defines ----------------------------*/
#define DEBOUNCE_TIME 200

#define FRONT_PIN "PF4"
#define REAR_PIN "PF3"
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/
static void Bumper_HWInit(void);

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
static BumperState_t  CurrentState;

static uint8_t        LastFrontBumpVal;
static uint8_t        LastRearBumpVal;

// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitTemplateFSM

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 18:55
****************************************************************************/
bool InitBumperFSM(uint8_t Priority)
{
  ES_Event_t ThisEvent;
  // call init function
  Bumper_HWInit();
  MyPriority = Priority;
  // put us into the Initial PseudoState
  CurrentState = InitBumperState;
  // post the initial transition event
  ThisEvent.EventType = ES_INIT;
  if (ES_PostToService(MyPriority, ThisEvent) == true)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/****************************************************************************
 Function
     PostTemplateFSM

 Parameters
     EF_Event_t ThisEvent , the event to post to the queue

 Returns
     boolean False if the Enqueue operation failed, True otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostBumperFSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunTemplateFSM

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event_t, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   Handles the debouncing of the bumper switches
 Notes
   uses nested switch/case to implement the machine.
 Author
   R. Merchant
****************************************************************************/
ES_Event_t RunBumperFSM(ES_Event_t ThisEvent)
{
  ES_Event_t  ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  ES_Event_t  BumpEvent;

  switch (CurrentState)
  {
    case InitBumperState:        // If current state is initial Psedudo State
    {
      if (ThisEvent.EventType == ES_INIT)    // only respond to ES_Init
      {
        // set state to Waiting4Press
        CurrentState = Waiting4Press;
      }
    }
    break;

    case Waiting4Press:        // If current state is waiting for switch press
    {
      switch (ThisEvent.EventType)
      {
        case EV_BUMPERPRESS_F:  //If event is front bumper hit
        {
          // Start debouncing timer
          ES_Timer_InitTimer(DEBOUNCE_TIMER, DEBOUNCE_TIME);
          // post front collision
          BumpEvent.EventType = EC_COLLISION_F;
          CurrentState = DebouncingFront;  //Decide what the next state will be
          PostMasterGameHSM(BumpEvent);
        }
        break;
        case EV_BUMPERPRESS_R:  //If event is rear bumper hit

        {
          // Start debouncing timer
          ES_Timer_InitTimer(DEBOUNCE_TIMER, DEBOUNCE_TIME);
          // post front collision
          BumpEvent.EventType = EC_COLLISION_R;
          CurrentState = DebouncingRear;  //Decide what the next state will be
          PostMasterGameHSM(BumpEvent);
        }
        break;

        default:
          ;
      }  // end switch on CurrentEvent
    }
    break;

    case DebouncingFront: // If state is debouncing front
    {
      switch (ThisEvent.EventType)
      {
        case ES_TIMEOUT: // if event is timeout
        {
          // set machine back to Waiting4Press
          CurrentState = Waiting4Press;
        }
        break;
        default:
          ;
      }
    }
    break;
    case DebouncingRear: // If state is debouncing rear
    {
      switch (ThisEvent.EventType)
      {
        case ES_TIMEOUT: // if event is timeout
        {
          // set machine back to Waiting4Press
          CurrentState = Waiting4Press;
        }
        break;
        default:
          ;
      }
    }
    break;

    default:
      ;
  }                                   // end switch on Current State
  return ReturnEvent;
}

/****************************************************************************
 Function
     QueryTemplateSM

 Parameters
     None

 Returns
     TemplateState_t The current state of the Template state machine

 Description
     returns the current state of the Template state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:21
****************************************************************************/

BumperState_t QueryBumperSM(void)
{
  return CurrentState;
}

/***************************************************************************
  Event Checker functions
 ***************************************************************************/
/****************************************************************************
 Function
     Check4FrontBump/ Check4RearBump

 Parameters
     None

 Returns
     Event Flag

 Description
  Checks for a falling edge from the front or rear switches to determine
  that a switch has been pressed. Posts event to service.
 Notes

 Author
     R. Merchant
****************************************************************************/
bool Check4FrontBump(void)
{
  bool    ReturnVal = false;
  // read pin
  uint8_t CurFrontBumpVal = GPIO_ReadPin(FRONT_PIN);
  // If pin has gone low
  if ((CurFrontBumpVal != LastFrontBumpVal) && (CurFrontBumpVal == 0))
  {
    // post to FSM
    ES_Event_t BumpEvent = { EV_BUMPERPRESS_F, 0 };
    PostBumperFSM(BumpEvent);
    // set return value true
    ReturnVal = true;
  }
  LastFrontBumpVal = CurFrontBumpVal;
  return ReturnVal;
}

bool Check4RearBump(void)
{
  bool    ReturnVal = false;
  // read pin
  uint8_t CurRearBumpVal = GPIO_ReadPin(REAR_PIN);
  // If pin has gone low
  if ((CurRearBumpVal != LastRearBumpVal) && (CurRearBumpVal == 0))
  {
    // post to FSM
    ES_Event_t BumpEvent = { EV_BUMPERPRESS_R, 0 };
    PostBumperFSM(BumpEvent);
    // set return value true
    ReturnVal = true;
  }
  LastRearBumpVal = CurRearBumpVal;
  return ReturnVal;
}

/***************************************************************************
 private functions
 ***************************************************************************/
/****************************************************************************
 Function
     Bumper_HWInit

 Parameters
     None

 Returns
     Nothing

 Description
     Intializes pins to read the state of the bumper switches
 Notes

 Author
     R. Merchant
****************************************************************************/
static void Bumper_HWInit(void)
{
  // Set up pins as inputs
  GPIO_ConfigPin( FRONT_PIN,  Input);
  GPIO_ConfigPin( REAR_PIN,   Input);
  // Set up pins with pullups
  HWREG(GPIO_PORTF_BASE + GPIO_O_PUR) |= (BIT3HI | BIT4HI);
  // Read state of the pin
  LastFrontBumpVal = GPIO_ReadPin(FRONT_PIN);
  LastRearBumpVal = GPIO_ReadPin(REAR_PIN);
}
