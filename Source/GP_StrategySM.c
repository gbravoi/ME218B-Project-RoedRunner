/****************************************************************************
 Module
   GamePlaceStrategySM.c

 Revision
 1.0.1

 Description
   Handles the startegy state machine (a lower level SM of MasterGameHSM)

 Notes

****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
// Basic includes for a program using the Events and Services Framework
#include "ES_Configure.h"
#include "ES_Framework.h"

/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "GP_MasterGameHSM.h"
#include "GP_StrategySM.h"
#include "GP_GetMinerSM.h"
#include "Miner.h"
#include "GP_DecidingSM.h"
#include "GP_NavigationSM.h"

/*----------------------------- Module Defines ----------------------------*/
// define constants for the states for this machine
// and any other local defines

#define ENTRY_STATE Deciding

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine, things like during
   functions, entry & exit functions.They should be functions relevant to the
   behavior of this state machine
*/
static ES_Event_t DuringDeciding(ES_Event_t Event);
static ES_Event_t DuringGettingMiner(ES_Event_t Event);
static ES_Event_t DuringNavigating(ES_Event_t Event);

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well
static GamePlayStrategyState_t CurrentState;
static GamePlayStrategyState_t HistoryState;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
    RunGamePlayStrategySM

 Parameters
   ES_Event_t: the event to process

 Returns
   ES_Event_t: an event to return

 Description
   add your description here
 Notes
   uses nested switch/case to implement the machine.
 Author
   rpt
****************************************************************************/
ES_Event_t RunGamePlayStrategySM(ES_Event_t CurrentEvent)
{
  bool            MakeTransition = false;/* are we making a state transition? */
  GamePlayStrategyState_t NextState = CurrentState;
  ES_Event_t      EntryEventKind = { ES_ENTRY, 0 }; // default to normal entry to new state
  ES_Event_t      ReturnEvent = CurrentEvent;       // assume we are not consuming event

  switch (CurrentState)
  {
    case Deciding:
    {
      ReturnEvent = CurrentEvent = DuringDeciding(CurrentEvent);
      //process any events
      if (CurrentEvent.EventType != ES_NO_EVENT)      //If an event is active
      {
        switch (CurrentEvent.EventType)
        {
          case GO_TO_MINER:
          {
						//set miner to interest in GetMinerSM
						SetMinerOfInterest((Miner_t)CurrentEvent.EventParam);
							
            // go to GettingMiner
            NextState = GettingMiner;
            // mark that we're making a transition
            MakeTransition = true;
            // GetMiner doesn't have history so...
            EntryEventKind.EventType = ES_ENTRY;
            // consume
            ReturnEvent.EventType = ES_NO_EVENT;
          }
          break;
					
					default:
						;
        }
      }
    }
    break;

    case GettingMiner:
    {
      ReturnEvent = CurrentEvent = DuringGettingMiner(CurrentEvent);
      //process any events
      if (CurrentEvent.EventType != ES_NO_EVENT)      //If an event is active
      {
        switch (CurrentEvent.EventType)
        {
          case MINER_ACQUIRED:
          {// go to Navigating
            NextState = Navigating;
            // mark that we're making a transition
            MakeTransition = true;
            // NavigationSM doesn't have history so...
            EntryEventKind.EventType = ES_ENTRY;
            // consume
            ReturnEvent.EventType = ES_NO_EVENT;
          }
          break;
					
					default:
						;
        }
      }
    }
    break;

    case Navigating :
    {
      ReturnEvent = CurrentEvent = DuringNavigating(CurrentEvent);
      //process any events
      if (CurrentEvent.EventType != ES_NO_EVENT)      //If an event is active
      {
        switch (CurrentEvent.EventType)
        {
          case NAVIGATION_ENDED:
          {
            // go to Deciding
            NextState = Deciding;
            // mark that we're making a transition
            MakeTransition = true;
            // DecidingSM doesn't have history so...
            EntryEventKind.EventType = ES_ENTRY;
            // consume
            ReturnEvent.EventType = ES_NO_EVENT;
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
    RunGamePlayStrategySM(CurrentEvent);
    HistoryState= NextState;  //update history state, save where I was
    CurrentState = NextState;    //Modify state variable

    //   Execute entry function for new state
    // this defaults to ES_ENTRY
    RunGamePlayStrategySM(EntryEventKind);
  }
  return ReturnEvent;
}

/****************************************************************************
 Function
     StartGamePlayStrategySM

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
void StartGamePlayStrategySM(ES_Event_t CurrentEvent)
{
  // to implement entry to a history state or directly to a substate
  // you can modify the initialization of the CurrentState variable
  // otherwise just start in the entry state every time the state machine
  // is started
  if (ES_ENTRY_HISTORY != CurrentEvent.EventType)
  {
    CurrentState = ENTRY_STATE;
  }
  else
  {
    CurrentState = HistoryState;
  }
  // call the entry function (if any) for the ENTRY_STATE
	
  RunGamePlayStrategySM(CurrentEvent);
}

/****************************************************************************
 Function
     QueryGamePlayStrategySM

 Parameters
     None

 Returns
     GamePlayStrategyState_t The current state of the Strategy SM

 Description
     returns the current state of the Strategy SM
 Notes

 Author
     rpt
****************************************************************************/
GamePlayStrategyState_t QueryGamePlayStrategySM(void)
{
  return CurrentState;
}

/***************************************************************************
 private functions
 ***************************************************************************/

static ES_Event_t DuringDeciding(ES_Event_t Event)
{
  ES_Event_t ReturnEvent = Event;   // assume no re-mapping or consumption

  // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
  if ((Event.EventType == ES_ENTRY) ||
      (Event.EventType == ES_ENTRY_HISTORY))
  {
    // start any lower level machines that run in this state
    StartDecidingSM(Event);
  }
  else if (Event.EventType == ES_EXIT)
  {
    // on exit, give the lower levels a chance to clean up first
     RunDecidingSM(Event);
  }
  else
  // do the 'during' function for this state
  {
    // run any lower level state machine
    ReturnEvent = RunDecidingSM(Event);
  }
  // return either Event, if you don't want to allow the lower level machine
  // to remap the current event, or ReturnEvent if you do want to allow it.
  return ReturnEvent;
}

static ES_Event_t DuringGettingMiner(ES_Event_t Event)
{
  ES_Event_t ReturnEvent = Event;   // assume no re-mapping or consumption

  // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
  if ((Event.EventType == ES_ENTRY) ||
      (Event.EventType == ES_ENTRY_HISTORY))
  {	
    // start any lower level machines that run in this state
    StartGetMinerSM( Event );
  }
  else if (Event.EventType == ES_EXIT)
  {
    // on exit, give the lower levels a chance to clean up first
    RunGetMinerSM(Event);
  }
  else
  // do the 'during' function for this state
  {
    // run any lower level state machine
    ReturnEvent = RunGetMinerSM(Event);
  }
  // return either Event, if you don't want to allow the lower level machine
  // to remap the current event, or ReturnEvent if you do want to allow it.
  return ReturnEvent;
}

static ES_Event_t DuringNavigating(ES_Event_t Event)
{
  ES_Event_t ReturnEvent = Event;   // assume no re-mapping or consumption

  // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
  if ((Event.EventType == ES_ENTRY) ||
      (Event.EventType == ES_ENTRY_HISTORY))
  {	
    // start any lower level machines that run in this state
     StartNavigationSM( Event );
  }
  else if (Event.EventType == ES_EXIT)
  {
    // on exit, give the lower levels a chance to clean up first
     RunNavigationSM(Event);
  }
  else
  // do the 'during' function for this state
  {
    // run any lower level state machine
     ReturnEvent = RunNavigationSM(Event);
  }
  // return either Event, if you don't want to allow the lower level machine
  // to remap the current event, or ReturnEvent if you do want to allow it.
  return ReturnEvent;
}
