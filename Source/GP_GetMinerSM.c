/****************************************************************************
 Module
   GetMinerSM.c

 Description
   Sub level of HSM GamePlayStrategy

 Notes

****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
// Basic includes for a program using the Events and Services Framework
#include "ES_Configure.h"
#include "ES_Framework.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/

#include "GP_GetMinerSM.h"
#include "GP_StrategySM.h"
#include "GP_MasterGameHSM.h"
#include "Miner.h"
#include "DriveService.h"
#include "AccelerometerFSM.h"

/*----------------------------- Module Defines ----------------------------*/
// define constants for the states for this machine
// and any other local defines

#define ENTRY_STATE AligningWithMiner
#define ROT360_TIME 6000
#define REPOSITION_TIME 300
#define RPM_VAL 50
#define REALIGNMENT_TIME 2000
#define INCREMENT_VAL 25
#define INCREMENT_TIME 250
#define GRAB_TIME 2000
#define MAX_SPEED 100
#define ROT_TICKS 200 //how much to rotate when detecting if grabbed right miner
#define REPOSITION_RPM_F 75


/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine, things like during
   functions, entry & exit functions.They should be functions relevant to the
   behavior of this state machine
*/
static ES_Event_t DuringAligningWithMiner(ES_Event_t Event);
static ES_Event_t DuringRepositioning(ES_Event_t Event);
static ES_Event_t DuringRampingUp(ES_Event_t Event);
static ES_Event_t DuringMoving2Miner(ES_Event_t Event);
static ES_Event_t DuringGrabbingMiner(ES_Event_t Event);

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well
static GetMinerState_t CurrentState;
static Miner_t MinerOfInterest;
static int16_t CurrentSpeed = 0;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
    RunGetMinerSM

 Parameters
   ES_Event_t: the event to process

 Returns
   ES_Event_t: an event to return

 Description
   run function for SM that handles miner acquisition

 Notes
   uses nested switch/case to implement the machine.

 Author
   rpt
****************************************************************************/
ES_Event_t RunGetMinerSM(ES_Event_t CurrentEvent)
{
  bool            MakeTransition = false;/* are we making a state transition? */
  GetMinerState_t NextState = CurrentState;
  ES_Event_t      EntryEventKind = { ES_ENTRY, 0 }; // default to normal entry to new state
  ES_Event_t      ReturnEvent = CurrentEvent;       // assume we are not consuming event
  ES_Event_t      ThisEvent;

  switch (CurrentState)
  {
    case AligningWithMiner:
    {    // Execute During function for AligningWithMiner
      ReturnEvent = CurrentEvent = DuringAligningWithMiner(CurrentEvent);

      //process any events
      if (CurrentEvent.EventType != ES_NO_EVENT)      //If an event is active
      {
        switch (CurrentEvent.EventType)
        {
          case MINER_FOUND:
          {
            // first stop turning
            ThisEvent.EventType = EM_STOP;
			PostDriveService(ThisEvent);
			// stop timer
            ES_Timer_StopTimer(ALIGNING_TIMER);
			
			//Set state to RampingUp
            NextState = RampingUp;      
			//mark that we are taking a transition
            MakeTransition = true; 
            // consume event
            ReturnEvent.EventType = ES_NO_EVENT;
          }
          break;

          case ES_TIMEOUT:
          {
			if (CurrentEvent.EventParam == ALIGNING_TIMER)
			{
				//Set state to repositioning
				NextState = Repositioning;      
				//mark that we are taking a transition
				MakeTransition = true;       
				// consume
				ReturnEvent.EventType = ES_NO_EVENT;

				// move forward
				ThisEvent.EventType = EM_DRIVE_F;
				ThisEvent.EventParam = REPOSITION_RPM_F;
				PostDriveService(ThisEvent);

				// start timer
				ES_Timer_InitTimer(ALIGNING_TIMER, REPOSITION_TIME);
			}
          }
          break;

		  default:
		  {
			;
		  }
        }
      }
    }
    break;

    case Repositioning:
    {    // Execute During function for Repositioning
      ReturnEvent = CurrentEvent = DuringRepositioning(CurrentEvent);

      //process any events
      if (CurrentEvent.EventType != ES_NO_EVENT)      //If an event is active
      {
        switch (CurrentEvent.EventType)
        {
          case ES_TIMEOUT:
          {
            // stop moving forward
			ThisEvent.EventType = EM_STOP;
			PostDriveService(ThisEvent);
			//Set state to AligningWithMiner
            NextState = AligningWithMiner;
            //mark that we are taking a transition
			MakeTransition = true;       

            // consume
            ReturnEvent.EventType = ES_NO_EVENT;
          }
          break;

		  default:
		  {
			;
		  }
        }
      }
    }
    break;

    case RampingUp:
    {    // Execute During function for Moving2Miner
      ReturnEvent = CurrentEvent = DuringRampingUp(CurrentEvent);

      //process any events
      if (CurrentEvent.EventType != ES_NO_EVENT)      //If an event is active
      {
        switch (CurrentEvent.EventType)
        {
          case MINER_REACHED: // should be sent from tape sensor
          {

			// activate electromagnet!
			MinerLib_EnableEM();
            // close servo arms
            MinerLib_Grab();

            ThisEvent.EventType = EM_STOP;
            PostDriveService(ThisEvent);

            // move to GrabbingMiner
            NextState = GrabbingMiner;
            MakeTransition = true;
            // consume
            ReturnEvent.EventType = ES_NO_EVENT;
          }
          break;

		  case ES_TIMEOUT:
			if (CurrentEvent.EventParam == RAMP_TIMER)
			{
              // increment speed
              int16_t NewSpeed = CurrentSpeed + INCREMENT_VAL;
							// set speed appropriatly
              if (NewSpeed >= MAX_SPEED)
              {
                // Set speed to MAX_SPEED
                ThisEvent.EventType = EM_DRIVE_F;
                ThisEvent.EventParam = MAX_SPEED;
                PostDriveService(ThisEvent);
                // done ramping up! transition to moving2miner
                NextState = Moving2Miner;
                MakeTransition = true;
                // update CurrentSpeed
                CurrentSpeed = MAX_SPEED;

              }
              else
              {
                // Set speed to NewSpeed
                ThisEvent.EventType = EM_DRIVE_F;
                ThisEvent.EventParam = NewSpeed;
                PostDriveService(ThisEvent);
                // update CurrentSpeed
                CurrentSpeed = NewSpeed;
                // restart RAMP_TIMER
                ES_Timer_InitTimer(RAMP_TIMER, INCREMENT_TIME);
              }

			  //consume
			  ReturnEvent.EventType = ES_NO_EVENT;
			}
		  break;

		  default:
				;
        }
      }
    }
    break;

    case Moving2Miner:
    {    // Execute During function for Moving2Miner
      ReturnEvent = CurrentEvent = DuringMoving2Miner(CurrentEvent);

      //process any events
      if (CurrentEvent.EventType != ES_NO_EVENT)      //If an event is active
      {
        switch (CurrentEvent.EventType)
        {
          case MINER_REACHED: // sent from tape sensor
          {
			// activate electromagnet!
			MinerLib_EnableEM();
            // close servo arms
            MinerLib_Grab();
            ThisEvent.EventType = EM_STOP;
            PostDriveService(ThisEvent);
			// move to GrabbingMiner
			NextState = GrabbingMiner;
            MakeTransition = true;


            // do NOT consume!
          }
          break;

		  case ES_TIMEOUT:
			if (CurrentEvent.EventParam == REALIGNMENT_TIMER)
			{
				// change state
				NextState = AligningWithMiner;
				// we're making a transition
				MakeTransition = true;
				//consume
				ReturnEvent.EventType = ES_NO_EVENT;
			}
		  break;

		  default:
			  ;
        }
      }
    }
    break;

	case GrabbingMiner:
    {    // Execute During function for Moving2Miner
      ReturnEvent = CurrentEvent = DuringGrabbingMiner(CurrentEvent);

      //process any events
      if (CurrentEvent.EventType != ES_NO_EVENT)      //If an event is active
      {
        switch (CurrentEvent.EventType)
        {
		  case ES_TIMEOUT:
		  {
			if (CurrentEvent.EventParam == GRAB_TIMER)
			{
				//enable tap detection
				AcclFSM_EnableTap();
				// let higher levels know that we've acquired 
				ReturnEvent.EventType = MINER_ACQUIRED; the miner
			}
		  }
		  break;
		  default:
		  {
			;
		  }
        }
      }
    }
    break;
	default:
	{
	  ;
	}
  }
  //   If we are making a state transition
  if (MakeTransition == true)
  {
    //   Execute exit function for current state
    CurrentEvent.EventType = ES_EXIT;
    RunGetMinerSM(CurrentEvent);
    CurrentState = NextState;    //Modify state variable

    //   Execute entry function for new state
    // this defaults to ES_ENTRY
    RunGetMinerSM(EntryEventKind);
  }
  return ReturnEvent;
}

/****************************************************************************
 Function
     StartGetMinerSM

 Parameters
     None

 Returns
     None

 Description
     Does any required initialization for this state machine

 Notes

 Author
     rpt
****************************************************************************/
void StartGetMinerSM(ES_Event_t CurrentEvent)
{
  // no history, just start in AligningWithMiner
  CurrentState = AligningWithMiner;

  // call the entry function (if any) for the ENTRY_STATE
  RunGetMinerSM(CurrentEvent);
}

/****************************************************************************
 Function
     QueryGetMinerSM

 Parameters
     None

 Returns
     GetMinerState_t The current state of the state machine

 Description
     returns the current state of the state machine
 Notes

 Author
     rpt
****************************************************************************/
GetMinerState_t QueryGetMinerSM(void)
{
  return CurrentState;
}


/***************************************************************************
 private functions
 ***************************************************************************/

static ES_Event_t DuringAligningWithMiner(ES_Event_t Event)
{
  ES_Event_t ReturnEvent = Event;   // assume no re-mapping or consumption

  // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
  if ( (Event.EventType == ES_ENTRY) ||
         (Event.EventType == ES_ENTRY_HISTORY) )
  {
    MinerOfInterest=QueryMinerOfInterest();
		// enable beacon interrupt
		StartFindingMiner(MinerOfInterest);

    // start timer to rotate 360 degrees
    ES_Timer_InitTimer(ALIGNING_TIMER, ROT360_TIME);

		// start rotating
		ES_Event_t ThisEvent;
		ThisEvent.EventType = EM_ROT_CW;
		ThisEvent.EventParam = RPM_VAL;
		PostDriveService(ThisEvent);

  }
  else if (Event.EventType == ES_EXIT)
  {


  }
  else
  // do the 'during' function for this state
  {
    // nothing to do here?
  }

  return ReturnEvent;
}

static ES_Event_t DuringRepositioning(ES_Event_t Event)
{
  ES_Event_t ReturnEvent = Event;   // assume no re-mapping or consumption

  // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
  if ( (Event.EventType == ES_ENTRY) ||
         (Event.EventType == ES_ENTRY_HISTORY) )
  {
  }
  else if (Event.EventType == ES_EXIT)
  {

  }
  else
  // do the 'during' function for this state
  {
  }

  return ReturnEvent;
}

static ES_Event_t DuringRampingUp(ES_Event_t Event)
{
  ES_Event_t ReturnEvent = Event;   // assume no re-mapping or consumption

  // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
  if ( (Event.EventType == ES_ENTRY) ||
         (Event.EventType == ES_ENTRY_HISTORY) )
  {
    ES_Event_t ThisEvent;

    // start moving forward
    ThisEvent.EventType = EM_DRIVE_F;
    ThisEvent.EventParam = INCREMENT_VAL;
    PostDriveService(ThisEvent);

    // initialize current speed
    CurrentSpeed = INCREMENT_VAL;

	// start REALIGNMENT_TIMER
    ES_Timer_InitTimer(RAMP_TIMER, INCREMENT_TIME);


  }
  else if (Event.EventType == ES_EXIT)
  {
    // stop RAMP timer just in case
    ES_Timer_StopTimer(RAMP_TIMER);

  }
  else
  // do the 'during' function for this state
  {
  }

  return ReturnEvent;
}

static ES_Event_t DuringMoving2Miner(ES_Event_t Event)
{
  ES_Event_t ReturnEvent = Event;   // assume no re-mapping or consumption

  // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
  if ( (Event.EventType == ES_ENTRY) ||
         (Event.EventType == ES_ENTRY_HISTORY) )
  {
		// start REALIGNMENT_TIMER
		ES_Timer_InitTimer(REALIGNMENT_TIMER, REALIGNMENT_TIME);

  }
  else if (Event.EventType == ES_EXIT)
  {
		// stop REALIGNMENT_TIMER
		ES_Timer_StopTimer(REALIGNMENT_TIMER);

  }
  else
  // do the 'during' function for this state
  {
  }

  return ReturnEvent;
}
static ES_Event_t DuringGrabbingMiner(ES_Event_t Event)
{
  ES_Event_t ReturnEvent = Event;   // assume no re-mapping or consumption

  // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
  if ( (Event.EventType == ES_ENTRY) ||
         (Event.EventType == ES_ENTRY_HISTORY) )
  {
		// start GRAB timer
    ES_Timer_InitTimer(GRAB_TIMER, GRAB_TIME);

  }
  else if (Event.EventType == ES_EXIT)
  {
		// stop GRAB timer
		ES_Timer_StopTimer(GRAB_TIMER);

  }
  else
  // do the 'during' function for this state
  {
  }

  return ReturnEvent;
}
