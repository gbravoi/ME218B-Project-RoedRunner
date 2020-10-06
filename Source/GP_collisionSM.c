/****************************************************************************
 Module
   GP_CollisionSM.c

 Revision
   1

 Description
   Move away after a collision is detected

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 02/26/20				gab				first code
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"

#include "GP_collisionSM.h"
#include "DriveService.h"
#include "TypesDefinition.h"
#include "GP_MasterGameHSM.h"

/*----------------------------- Module Defines ----------------------------*/
// define constants for the states for this machine
// and any other local defines

//some robots parameters needed for rotate 45 deg
  #define TURN_R_MM 150   //wheels diameter
  #define WHEEL_C_MM 236  //distance between wheels mm
  #define TICKS_PER_REV 150
  #define PI 3.14159265358979f

  #define DISTANCE_AWAY1 100  //ticks
  #define ANGLE 90            //deg
  #define DISTANCE_AWAY2 10   //ticks

//for last collision
  #define FROM_FRONT 0
  #define FROM_REAR 1

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine, things like during
   functions, entry & exit functions. They should be functions relevant to the
   behavior of this state machine
*/

static ES_Event_t DuringRotate(ES_Event_t Event);
static int16_t AngletoTicks(float dangle);

/*---------------------------- Module Variables ---------------------------*/

// everybody needs a state variable, you may need others as well
static CollisionState_t CurrentState;
static uint8_t          LastCollision;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
    RunCollisionSM

 Parameters
   ES_Event CurrentEvent: the event to process

 Returns
   ES_Event

 Description
   Implements the collision machine for the MasterGameHSM

 Author
   G. Bravo, 2/23/20
****************************************************************************/
ES_Event_t RunCollisionSM(ES_Event_t CurrentEvent)
{
  bool              MakeTransition = false;/* are we making a state transition? */
  CollisionState_t  NextState = CurrentState;
  ES_Event_t        ReturnEvent = CurrentEvent; // assume we are not consuming event

  switch (CurrentState)
  {
    case WaitingCollision:
    {
      // Call during function of the state
      //no during

      if (CurrentEvent.EventType == COLLISION)   //move away of object
      {
        //if collision from the front
        if (CurrentEvent.EventParam == Coll_Front)
        {
          //Post drive service to go back for a certain distance
          ES_Event_t NewEvent;
          NewEvent.EventType = EM_DRIVE_R_POS;
          NewEvent.EventParam = DISTANCE_AWAY1;
          PostDriveService(NewEvent);
          //save parameter of type of collision
          LastCollision = FROM_FRONT;
        }
        else         //else collision from back
        {
          ES_Event_t NewEvent;
          NewEvent.EventType = EM_DRIVE_F_POS;
          NewEvent.EventParam = DISTANCE_AWAY1;
          PostDriveService(NewEvent);
          //save parameter of type of collision
          LastCollision = FROM_REAR;
        }

        NextState = MoveAway1;                //Decide what the next state will be
        MakeTransition = true;                //mark that we are taking a transition
        ReturnEvent.EventType = ES_NO_EVENT;  // consume this event
      }
    }
    break;
    case MoveAway1:
    {
      // Call during function of the state
      //no during

      if (CurrentEvent.EventType == EM_AT_POS)
      {
        //Drive service end moving
        NextState = Rotate;                   //Decide what the next state will be
        MakeTransition = true;                //mark that we are taking a transition
        ReturnEvent.EventType = ES_NO_EVENT;  // consume this event
      }
    }

    break;
    case Rotate:
    {
      // Call during function of the state
      DuringRotate(CurrentEvent);

      if (CurrentEvent.EventType == EM_AT_POS)
      {                                       //Drive service end moving
        NextState = MoveAway2;                //Decide what the next state will be
        MakeTransition = true;                //mark that we are taking a transition
        ReturnEvent.EventType = ES_NO_EVENT;  // consume this event

        //post to move in the same direction it was moving before collision
        //if collision from the front
        if (LastCollision == FROM_FRONT)
        {
          //Post drive service to go forward for a certain distance
          ES_Event_t NewEvent;
          NewEvent.EventType = EM_DRIVE_F_POS;
          NewEvent.EventParam = DISTANCE_AWAY2;
          PostDriveService(NewEvent);
        }
        else         //else collision from back
        {
          ES_Event_t NewEvent;
          NewEvent.EventType = EM_DRIVE_R_POS;
          NewEvent.EventParam = DISTANCE_AWAY2;
          PostDriveService(NewEvent);
        }
      }
    }

    break;

    case MoveAway2:
    {
      // Call during function of the state

      if (CurrentEvent.EventType == EM_AT_POS)
      {
        //Drive service end moving backwards
        ReturnEvent.EventType = ES_NO_EVENT;         // consume this event
        ES_Event_t NewEvent;
        NewEvent.EventType = COLLISION_AVOIDED;
        PostMasterGameHSM(NewEvent);

        //move to waiting collision state
        NextState = WaitingCollision;         //Decide what the next state will be
        MakeTransition = true;                //mark that we are taking a transition
        ReturnEvent.EventType = ES_NO_EVENT;  // consume this event
      }
    }
    break;
  }

  //   If we are making a state transition
  if (MakeTransition == true)
  {
    //   Execute exit function for current state
    CurrentEvent.EventType = ES_EXIT;
    RunCollisionSM(CurrentEvent);

    CurrentState = NextState;    //Modify state variable

    //   Execute entry function for new state
    CurrentEvent.EventType = ES_ENTRY;
    RunCollisionSM(CurrentEvent);
  }
  return ReturnEvent;
}

/****************************************************************************
 Function
     StartCollisionSM

 Parameters
     ES_Event CurrentEvent

 Returns
     None

 Description
     Does any required initialization for this state machine
 Notes

 Author
     G. Bravo
****************************************************************************/
void StartCollisionSM(ES_Event_t CurrentEvent)
{
  // local variable to get debugger to display the value of CurrentEvent
  volatile ES_Event_t LocalEvent = CurrentEvent;
  CurrentState = WaitingCollision;   // always start in WaitingCollision
  // call the entry function (if any) for the ENTRY_STATE
  RunCollisionSM(CurrentEvent);
}

/***************************************************************************
 private functions
 ***************************************************************************/

static ES_Event_t DuringRotate(ES_Event_t Event)
{
  // process ES_ENTRY & ES_EXIT events
  if (Event.EventType == ES_ENTRY)
  {
    // implement any entry actions required for this state machine

    //Post drive service to rotate 45 deg
    ES_Event_t NewEvent;
    NewEvent.EventType = EM_ROT_CW_POS;
    NewEvent.EventParam = AngletoTicks(ANGLE);
    PostDriveService(NewEvent);
  }
  else if (Event.EventType == ES_EXIT)
  {
    // no exist functionality
  }
  else
  // do the 'during' function for this state
  {
    // no during function for this machine
  }
  return Event;       // Don't remap this event
}

/****************************************************************************
 Function
    AngletoTicks

 Parameters
   float angle in deg

 Returns
  number of ticks needed to rotate 45deg

 Description
   computes how many ticks are needed to rotate X deg using DriveService EM_ROT_CW_POS

 Author
   G. Bravo, 2/23/20
****************************************************************************/
static int16_t AngletoTicks(float dangle)
{
  //transform deg to rad
  dangle = (dangle * PI) / 180.0f;

  // calculate dS from r and dtheta
  int16_t dS = TURN_R_MM * dangle;
  // calculate the number of ticks for dS
  int16_t rotateTicks = (dS * TICKS_PER_REV) / WHEEL_C_MM;

  return rotateTicks;
}

/***************************NOTES**********************************
1. Remember include the Master Game HSM in the distribution list, to known when location at a position finished.
*/
