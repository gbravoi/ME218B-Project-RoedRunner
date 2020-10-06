/****************************************************************************
 Module
   MasterGameHSM.c

 Revision
   1.0

 Description
   This is the top level of the game

****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "GP_MasterGameHSM.h"
#include "SPUDHSM.h"
#include "GP_StrategySM.h"

#include "Miner.h"
#include "EncoderLib.h"
#include "PinsLib.h"
#include "DriveService.h"
#include "GP_collisionSM.h"
#include "GeneralPWM.h"
#include "Color.h"

/*----------------------------- Module Defines ----------------------------*/


/*---------------------------- Module Functions ---------------------------*/
static ES_Event_t DuringWaiting4Permits(ES_Event_t Event);
static ES_Event_t DuringPlayingGame(ES_Event_t Event);
static ES_Event_t DuringHandlingCollision(ES_Event_t Event);

/*---------------------------- Module Variables ---------------------------*/
static MasterState_t  CurrentState;
// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t        MyPriority;

//my team selected
static Team_t   MyTeam;
static Miner_t  MinerOfInterest;//the miner being transported in the front

//Miner name based on OURS company
static Miner_t  OurMINER1;
static Miner_t  OurMINER2;
//miner name of enemy
static Miner_t  TheirMINER1;
static Miner_t  TheirMINER2;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitMasterGameHSM

 Parameters
     uint8_t : the priority of this service

 Returns
     boolean, False if error in initialization, True otherwise

 Description
     Saves away the priority,  and starts
     the top level state machine
 Notes

 Author
     rpt
****************************************************************************/
bool InitMasterGameHSM(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  // save our priority
  MyPriority = Priority;

  //Do all hardware initilization
  // Miner HW
  Miner_HWInit();
  // Initialize Servos
  ServoPWM_SetPosition( 0,  0);
  ServoPWM_SetPosition( 1,  180);
  // Make sure pulse control is "open"
  MinerLib_Release();
  ServoPWM_EnableOutputs();
  //Pins (Team select, status)
  InitPins();
	//enable color event checker
  EnableColorEventChecker();

  //Check which team we are
  MyTeam = ReadTeam();

  //Define Miner number based in which company is OURS
  if (MyTeam == CHK)
  {
    OurMINER1 = CHK_Miner1;
    OurMINER2 = CHK_Miner2;
    TheirMINER1 = GHI_Miner1;
    TheirMINER2 = GHI_Miner2;
  }
  else
  {
    OurMINER1 = GHI_Miner1;
    OurMINER2 = GHI_Miner2;
    TheirMINER1 = CHK_Miner1;
    TheirMINER2 = CHK_Miner2;
  }

  // Start the Master State machine
  ThisEvent.EventType = ES_ENTRY;
  StartMasterGameHSM(ThisEvent);

  return true;
}

/****************************************************************************
 Function
     PostMasterGameHSM

 Parameters
     ES_Event_t ThisEvent , the event to post to the queue

 Returns
     boolean False if the post operation failed, True otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     rpt
****************************************************************************/
bool PostMasterGameHSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunMasterGameHSM

 Parameters
   ES_Event: the event to process

 Returns
   ES_Event: an event to return

 Description
   the run function for the top level state machine
 Notes
   uses nested switch/case to implement the machine.
 Author
   rpt
****************************************************************************/
ES_Event_t RunMasterGameHSM(ES_Event_t CurrentEvent)
{
  bool          MakeTransition = false;/* are we making a state transition? */
  MasterState_t NextState = CurrentState;
  ES_Event_t    EntryEventKind = { ES_ENTRY, 0 }; // default to normal entry to new state
  ES_Event_t    ReturnEvent = { ES_NO_EVENT, 0 }; // assume no error

  switch (CurrentState)
  {
    case Waiting4Permits:
    {
      CurrentEvent = DuringWaiting4Permits(CurrentEvent);
      //process any events
      if (CurrentEvent.EventType != ES_NO_EVENT)      //If an event is active
      {
        switch (CurrentEvent.EventType)
        {
          case STATUS_UPDATE:
          {
            // if permits have been issued
						if (CurrentEvent.EventParam == PI)
            {
              // go to PlayingGame next
              NextState = PlayingGame;
              // note that we're making a transition
              MakeTransition = true;
              // go in with history
              EntryEventKind.EventType = ES_ENTRY_HISTORY;
              // turn on permit LED
              TurnOnMiningStatus();
            }
            else if ((CurrentEvent.EventParam == W4P) || 
			(CurrentEvent.EventParam == SD) || (CurrentEvent.EventParam == PE))
            {
              // stay in Waiting4Permits
              NextState = Waiting4Permits;
              // note that we're making a transition
              MakeTransition = true;
              // no history involved here
              EntryEventKind.EventType = ES_ENTRY;
            }
          }
          break;
          default:
            ;
        }
      }
    }
    break;

    case PlayingGame:
    {
      CurrentEvent = DuringPlayingGame(CurrentEvent);
      //process any events
      if (CurrentEvent.EventType != ES_NO_EVENT)      //If an event is active
      {
        switch (CurrentEvent.EventType)
        {
          case (EC_COLLISION_F):  // if collision from the front
          {
            // Query RPM from left motor
            EncData_t RPMdata;
            EncLib_GetData(0, &RPMdata);

            if (RPMdata.RPM > 0) // if we are moving forward
            {
              //post collition with param front (to be read by lower level SM)
              ES_Event_t NewEvent;
              NewEvent.EventType = COLLISION;
              NewEvent.EventParam = Coll_Front;
              PostMasterGameHSM(NewEvent);

              // next state should be HandlingCollision
              NextState = HandlingCollision;
              // note that we're making a transition
              MakeTransition = true;
              // bc no history state
              EntryEventKind.EventType = ES_ENTRY;
            }
          }
          break;

          case (EC_COLLISION_R):  // if collision from the back
          {
            // Query RPM from left motor
            EncData_t RPMdata;
            EncLib_GetData(0, &RPMdata);
            // make sure we are moving backwards here
            if (RPMdata.RPM < 0) // if we are moving
            {
              //post collition with param rear  (to be read by lower level SM)
              ES_Event_t NewEvent;
              NewEvent.EventType = COLLISION;
              NewEvent.EventParam = Coll_Rear;
              PostMasterGameHSM(NewEvent);

              // next state should be HandlingCollision
              NextState = HandlingCollision;
              // note that we're making a transition
              MakeTransition = true;
              // bc no history state
              EntryEventKind.EventType = ES_ENTRY;
            }
          }
          break;

          case STATUS_UPDATE:
          {
            if (CurrentEvent.EventParam == PE)
            {
              // Set next state to Waiting4Permits
              NextState = Waiting4Permits;
              // note that we're making a transition
              MakeTransition = true;
              // no history involved here
              EntryEventKind.EventType = ES_ENTRY;
              // stop moving
              ES_Event_t ThisEvent;
              ThisEvent.EventType = EM_STOP;
              PostDriveService(ThisEvent);
              // turn off permit LED
              TurnOffMiningStatus();
            }
          }
          break;
          default:
            ;
        }
      }
    }
    break;

    case HandlingCollision:
    {
      CurrentEvent = DuringHandlingCollision(CurrentEvent);
      //process any events
      if (CurrentEvent.EventType != ES_NO_EVENT)      //If an event is active
      {
        switch (CurrentEvent.EventType)
        {
          case COLLISION_AVOIDED:  // if we've successfully responded to a collision
          {
            // go back to PlayingGame
            NextState = PlayingGame;
            // note that we're making a transition
            MakeTransition = true;
            // history
            EntryEventKind.EventType = ES_ENTRY_HISTORY;
          }
          break;
		 case STATUS_UPDATE:
          {
            if (CurrentEvent.EventParam == PE)
            {
              // set next state to Waiting4Permits
              NextState = Waiting4Permits;
              // note that we're making a transition
              MakeTransition = true;
              // no history involved here
              EntryEventKind.EventType = ES_ENTRY;
              // stop moving
              ES_Event_t ThisEvent;
              ThisEvent.EventType = EM_STOP;
              PostDriveService(ThisEvent);
              // turn off permit LED
              TurnOffMiningStatus();
            }
          }
					break;
          default:
            ;
        }
      }
    }
    break;
    default:
      ;
  }
  //   If we are making a state transition
  if (MakeTransition == true)
  {
    //   Execute exit function for current state
    CurrentEvent.EventType = ES_EXIT;
    RunMasterGameHSM(CurrentEvent);

    CurrentState = NextState;    //Modify state variable

    // Execute entry function for new state
    // this defaults to ES_ENTRY
    RunMasterGameHSM(EntryEventKind);
  }
  // in the absence of an error the top level state machine should
  // always return ES_NO_EVENT, which we initialized at the top of func
  return ReturnEvent;
}

/****************************************************************************
 Function
     StartMasterGameHSM

 Parameters
     ES_Event CurrentEvent

 Returns
     nothing

 Description
     Does any required initialization for this state machine
 Notes

 Author
     rpt
****************************************************************************/
void StartMasterGameHSM(ES_Event_t CurrentEvent)
{
  // if there is more than 1 state to the top level machine you will need
  // to initialize the state variable
  CurrentState = Waiting4Permits;
  // now we need to let the Run function init the lower level state machines
  // use LocalEvent to keep the compiler from complaining about unused var
  RunMasterGameHSM(CurrentEvent);
  return;
}

/****************************************************************************
 Function
     QueryTeam

 Parameters
     void

 Returns
     Team_t the team we are

 Description
     Allow lower level SM know which team we are
 Notes

 Author
     rpt
****************************************************************************/
Team_t QueryTeam(void)
{
  return MyTeam;
}

/****************************************************************************
 Function
     QueryMinerID

 Parameters
     void

 Returns
     Miner_t the MinerID of the miner queried

 Description
     Allow lower level SM know which miner ID is related to a specific miner
 Notes

 Author
     gab
****************************************************************************/
Miner_t OurMiner1ID(void)
{
  return OurMINER1;
}

Miner_t OurMiner2ID(void)
{
  return OurMINER2;
}

Miner_t TheirMiner1ID(void)
{
  return TheirMINER1;
}

Miner_t TheirMiner2ID(void)
{
  return TheirMINER2;
}

/****************************************************************************
 Function
     QueryMinerOfInterest

 Parameters
     void

 Returns
     Miner_t The miner we want to move / are moving

 Description
     Allow lower level SM know with wich MINER we are interacting
 Notes

 Author
     rpt
****************************************************************************/
Miner_t QueryMinerOfInterest(void)
{
  return MinerOfInterest;
}

/****************************************************************************
 Function
     SetMinerOfInterest

 Parameters
     some miner Miner_t

 Returns
     nothing

 Description
     allows us to set what miner we're looking at
 Notes

 Author
     rpt
****************************************************************************/

void SetMinerOfInterest(Miner_t MinerID)
{
  MinerOfInterest = MinerID;
}

/***************************************************************************
 private functions
 ***************************************************************************/

static ES_Event_t DuringWaiting4Permits(ES_Event_t Event)
{
  ES_Event_t  ReturnEvent = Event;
  ES_Event_t  ThisEvent;
  // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
  if ((Event.EventType == ES_ENTRY) ||
      (Event.EventType == ES_ENTRY_HISTORY))
  {
    // query the status from the SPUD
    ThisEvent.EventType = INFO_QUERY;
    ThisEvent.EventParam = STATUS;
    PostSPUDHSM(ThisEvent);
  }
  else if (Event.EventType == ES_EXIT)
  {
    // ain't no exit functionality
  }
  else
  // do the 'during' function for this state
  {
    // also ain't no during functionality
  }
  return ReturnEvent;
}

static ES_Event_t DuringPlayingGame(ES_Event_t Event)
{
  ES_Event_t ReturnEvent = Event;   // assme no re-mapping or comsumption
  // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
  if ((Event.EventType == ES_ENTRY) ||
      (Event.EventType == ES_ENTRY_HISTORY))
  {
    // start lower level SM
    StartGamePlayStrategySM(Event);
  }
  else if (Event.EventType == ES_EXIT)
  {
    // on exit, give the lower levels a chance to clean up first
    RunGamePlayStrategySM(Event);
  }
  else
  // do the 'during' function for this state
  {
    // run any lower level state machine
    ReturnEvent = RunGamePlayStrategySM(Event);
  }
  // return either Event, if you don't want to allow the lower level machine
  // to remap the current event, or ReturnEvent if you do want to allow it.
  return ReturnEvent;
}

static ES_Event_t DuringHandlingCollision(ES_Event_t Event)
{
  ES_Event_t ReturnEvent = Event;   // assme no re-mapping or comsumption
  // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
  if ((Event.EventType == ES_ENTRY) ||
      (Event.EventType == ES_ENTRY_HISTORY))
  {
    // start lower level SM
    StartCollisionSM(Event);
  }
  else if (Event.EventType == ES_EXIT)
  {
    // on exit, give the lower levels a chance to clean up first
    RunCollisionSM(Event);
  }
  else
  // do the 'during' function for this state
  {
    // run any lower level state machine
    ReturnEvent = RunCollisionSM(Event);
  }
  // return either Event, if you don't want to allow the lower level machine
  // to remap the current event, or ReturnEvent if you do want to allow it.
  return ReturnEvent;
}
